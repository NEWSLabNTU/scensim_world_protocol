"""Request deadlines (roadmap 077, G70): a world server that blocks ends, for a fresh one.

On Isaac the world server's main thread blocked for good inside PhysX after a world reload:
the bridge waited on a reply that never came, gave up after 10 s and restarted, and the next
run found the same world server still blocked. A thread watching from inside the process
could not end it: 241's repro (2026-11-16) blocked with neither a watchdog thread nor
faulthandler's C timer firing, whether because the block was outside the one call they
guarded or because of Kit's own handling of threads and signals.

So the watchdog is a small child process. The world server tells it, over a pipe, when each
request (and each pass of its loop between requests) begins, with a deadline by kind, and
when it ends. Past a deadline the child logs one line naming what is in flight and for how
long, takes a stack dump of the world server if it can (`py-spy dump --native` when py-spy
is installed; then SIGUSR2, on which faulthandler dumps every thread from a C handler), and
SIGKILLs it. Its supervisor (play_launch, launch_ros) starts a fresh one, and the bridge,
which ended its own wait, finds the fresh one on its next Hello. The child needs no GIL and
nothing of the world server's threads; it exits when the pipe closes (the world server is
gone) and asks the kernel to end it with its parent (SIGKILL); it ignores SIGINT and
SIGTERM, which a stop sends the whole group, so it never ends before the world server.

The child also ends the world server on a fatal line in a log file it is told to watch
(`watch_log`): Kit's own log, written by Kit's C logger, where Kit's FATAL messages land
(issue 011). Watching from here, no Python runs on Kit's log path: a Python omni.log consumer
in the world server took the GIL for every message, and a PhysX cooking task logging from a
worker thread then waited for the GIL that the main thread, waiting for the task, held
(csb issue 042). A timer in the world server cannot watch a thread that holds the GIL either,
so the world server's start is a block like any other, with its own deadline.

Deadlines: a step 180 s, far above a legitimate slow one (a first Isaac spawn took 68-75 s,
077 G65); a world load 900 s, since a first Isaac load takes minutes. `None` or 0: none.
"""
import faulthandler
import logging
import os
import shutil
import signal
import subprocess
import sys
import time

log = logging.getLogger("swb_world_server")

STEP_DEADLINE = 180.0  # s: every request and loop pass but a world load
LOAD_DEADLINE = 900.0  # s: LoadWorld
DUMP_SIGNAL = signal.SIGUSR2  # faulthandler's dump of every thread, from a C handler
DUMP_WAIT = 3.0  # s for the dumps before the kill
LOG_POLL = 0.5  # s between looks at a watched log file


class StepWatchdog:
    """`with watchdog.step(label, deadline=None): ...`: the child process kills this one
    when the block takes longer than its deadline (the watchdog's own by default)."""

    def __init__(self, deadline=STEP_DEADLINE, child=None):
        self.deadline = deadline or None
        self._proc = None
        if self.deadline is None:
            return
        try:
            faulthandler.register(DUMP_SIGNAL, all_threads=True, chain=False)
        except (RuntimeError, ValueError, OSError, AttributeError) as e:
            log.warning("request deadline: no faulthandler dump on signal %d (%s)",
                        DUMP_SIGNAL, e)
        # an embedding app's sys.executable may not be a Python (Kit's launcher): then python3
        py = sys.executable if os.path.basename(sys.executable or "").startswith("python") \
            else (shutil.which("python3") or sys.executable)
        argv = child or [py, "-m", "swb_world_server.watchdog", str(os.getpid())]
        env = dict(os.environ)
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env["PYTHONPATH"] = pkg + os.pathsep + env.get("PYTHONPATH", "")
        self._proc = subprocess.Popen(argv, stdin=subprocess.PIPE, env=env, close_fds=True)

    def step(self, label, deadline=None):
        return _Armed(self, label, deadline)

    def _send(self, line):
        if self._proc is None:
            return
        try:
            self._proc.stdin.write(line.encode())
            self._proc.stdin.flush()
        except (BrokenPipeError, ValueError, OSError):
            log.error("request deadline: the watchdog process is gone; no deadline from now on")
            self._proc = None

    def watch_log(self, path, marker):
        """From now on the child ends this process at a line of the file `path` holding
        `marker` (Kit's log and its `] [Fatal] [`), from the file's start: a fatal logged
        before the call counts too. The file need not exist yet."""
        if "\t" in marker or "\n" in marker or "\n" in str(path):
            raise ValueError("a watched log's marker and path take no tab or newline")
        self._send(f"log\t{marker}\t{path}\n")

    def _begin(self, label, deadline):
        d = deadline or self.deadline
        if d:
            self._send(f"begin {d:g} {label}\n".replace("\n", " ").rstrip() + "\n")

    def _end(self):
        self._send("end\n")

    def close(self):
        if self._proc is None:
            return
        try:
            self._proc.stdin.close()
            self._proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.kill()
        self._proc = None


class _Armed:
    def __init__(self, watchdog, label, deadline):
        self.watchdog, self.label, self.deadline = watchdog, label, deadline

    def __enter__(self):
        self.watchdog._begin(self.label, self.deadline)
        return self

    def __exit__(self, *_exc):
        self.watchdog._end()
        return False


# --- the child process --------------------------------------------------------------------


def _die_with_parent():
    # SIGKILL, not SIGTERM: the child ignores SIGTERM (main), so only this ends it with its
    # parent, should the pipe somehow stay open
    try:
        import ctypes
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, int(signal.SIGKILL), 0, 0, 0)
    except OSError:
        pass


def _say(msg):
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    sys.stderr.write(f"{stamp} ERROR swb_world_server.watchdog: {msg}\n")
    sys.stderr.flush()


def _expire(parent, label, elapsed, deadline, kill=os.kill):
    _say(f"{label} has run {elapsed:.1f} s, past its deadline of {deadline:g} s: ending world "
         f"server {parent} for a fresh one")
    _end_parent(parent, kill)


def _fatal(parent, path, line, kill=os.kill):
    _say(f"fatal in {path}: {line.strip()}: ending world server {parent} for a fresh one")
    _end_parent(parent, kill)


def _end_parent(parent, kill=os.kill):
    spy = shutil.which("py-spy")
    if spy:
        try:
            r = subprocess.run([spy, "dump", "--pid", str(parent), "--native"],
                               capture_output=True, text=True, timeout=30)
            sys.stderr.write(r.stdout + r.stderr)
            sys.stderr.flush()
        except (OSError, subprocess.TimeoutExpired) as e:
            _say(f"py-spy dump failed: {e}")
    try:
        kill(parent, DUMP_SIGNAL)
        time.sleep(DUMP_WAIT)
        kill(parent, signal.SIGKILL)
    except ProcessLookupError:
        pass


class _LogWatch:
    """A log file read as it grows, for lines holding a marker."""

    def __init__(self, path, marker):
        self.path, self.marker = path, marker.encode()
        self.f, self.buf = None, b""

    def fatal_line(self):
        """The first new line holding the marker, or None."""
        if self.f is None:
            try:
                self.f = open(self.path, "rb")
            except OSError:
                return None
        while True:
            chunk = self.f.read(65536)
            if not chunk:
                return None
            self.buf += chunk
            *lines, self.buf = self.buf.split(b"\n")
            for line in lines:
                if self.marker in line:
                    return line.decode(errors="replace")


def watch(parent, stream, clock=time.monotonic, expire=_expire, fatal=_fatal):
    """Read `begin <deadline> <label>` / `end` / `log<TAB><marker><TAB><path>` lines from
    `stream` (a binary pipe). Call `expire(parent, label, elapsed, deadline)` when a begun
    block outlives its deadline, and `fatal(parent, path, line)` at a line of a watched log
    holding its marker. Returns after either, or at end of stream (the parent closed it or
    died)."""
    import selectors

    sel = selectors.DefaultSelector()
    sel.register(stream, selectors.EVENT_READ)
    buf = b""
    current = None  # (label, started, deadline)
    logs = []
    while True:
        now = clock()
        if current is not None and now >= current[1] + current[2]:
            label, started, deadline = current
            expire(parent, label, now - started, deadline)
            return
        for lw in logs:
            line = lw.fatal_line()
            if line is not None:
                fatal(parent, lw.path, line)
                return
        timeout = None if current is None else max(0.0, current[1] + current[2] - now)
        if logs:
            timeout = LOG_POLL if timeout is None else min(timeout, LOG_POLL)
        if not sel.select(timeout):
            continue
        chunk = os.read(stream.fileno(), 4096)
        if not chunk:
            return
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace")
            if text.startswith("log\t"):
                parts = text.split("\t", 2)
                if len(parts) == 3:
                    logs.append(_LogWatch(parts[2], parts[1]))
                continue
            words = text.split(" ", 2)
            if words[0] == "begin" and len(words) == 3:
                current = (words[2], clock(), float(words[1]))
            elif words[0] == "end":
                current = None


def main():
    _die_with_parent()
    # A terminal's Ctrl-C, and a launcher's stop of the whole group, are the parent's: the
    # child outlives neither (pipe EOF, parent-death SIGKILL) and must not end before it.
    # Ending first, it left the parent logging "the watchdog process is gone" at every stop
    # (241, 2026-11-16), and left a parent that stops slowly without its deadline.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    watch(int(sys.argv[1]), sys.stdin.buffer)


if __name__ == "__main__":
    main()
