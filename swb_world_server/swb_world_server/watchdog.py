"""A step deadline (roadmap 077, G70): a world server whose step blocks ends, for a fresh one.

On Isaac the main thread once blocked for good inside a PhysX step after a world reload: the
bridge waited on a reply that never came, and the run hung instead of ending. A thread that
is blocked cannot time itself out, so `StepWatchdog` watches it from another thread. When a
step runs past its deadline it logs one line naming the step and its time, dumps every
thread's stack (faulthandler) and ends the process with exit 75, the code that asks the
supervisor for a fresh process (as `swb_carla_world` does when its CARLA is gone); the bridge
then loses the frame and the run ends `error`.

That thread needs the GIL, which a native call may hold while it blocks. So each step also
arms faulthandler's own watchdog (`dump_traceback_later`), a C thread that needs no GIL, a
margin later: it dumps the stacks and exits 1. The supervisor respawns on either code; 75
says the deadline did it, 1 after the dump says the step held the GIL.

The deadline is far above a legitimate slow step: a first spawn on Isaac took 68-75 s
(roadmap 077, G65), so the default is 180 s. `None` or 0 turns it off.
"""
import faulthandler
import logging
import os
import sys
import threading
import time

log = logging.getLogger("swb_world_server")

STEP_DEADLINE = 180.0  # s
EXIT_CODE = 75  # a fresh process, please (EX_TEMPFAIL)
BACKSTOP_MARGIN = 30.0  # s after the deadline: faulthandler's watchdog, which needs no GIL


def _expire(label, elapsed, deadline):
    log.error("%s has run %.1f s, past the step deadline of %g s: exiting %d for a fresh "
              "world server", label, elapsed, deadline, EXIT_CODE)
    for h in logging.getLogger().handlers + log.handlers:
        try:
            h.flush()
        except Exception:  # noqa: BLE001 - exiting regardless
            pass
    try:
        faulthandler.dump_traceback(file=sys.stderr, all_threads=True)
    except Exception:  # noqa: BLE001
        pass
    os._exit(EXIT_CODE)


class StepWatchdog:
    """`with watchdog.step(label): world.step(...)`; ends the process (`on_expire(label,
    elapsed, deadline)`) when the block takes longer than `deadline` seconds."""

    def __init__(self, deadline=STEP_DEADLINE, on_expire=_expire, clock=time.monotonic,
                 backstop=BACKSTOP_MARGIN):
        self.deadline = deadline or None
        self.on_expire = on_expire
        self.clock = clock
        # faulthandler's watchdog this long after the deadline; None: none
        self.backstop = backstop if self.deadline is not None else None
        self._cv = threading.Condition()
        self._label = None
        self._started = None
        self._serial = 0
        self._closed = False
        self._thread = None
        if self.deadline is not None:
            self._thread = threading.Thread(target=self._watch, name="step-watchdog",
                                            daemon=True)
            self._thread.start()

    def step(self, label):
        return _Armed(self, label)

    def _begin(self, label):
        if self.deadline is None:
            return
        with self._cv:
            self._label, self._started = label, self.clock()
            self._serial += 1
            self._cv.notify()
        if self.backstop is not None:
            try:
                faulthandler.dump_traceback_later(self.deadline + self.backstop, exit=True)
            except (RuntimeError, ValueError, OSError, AttributeError) as e:
                # a stderr without a file descriptor (an embedding app's): no backstop
                log.warning("step deadline: faulthandler's backstop unavailable (%s); the "
                            "deadline holds only while a step lets go of the GIL", e)
                self.backstop = None

    def _end(self):
        if self.deadline is None:
            return
        if self.backstop is not None:
            faulthandler.cancel_dump_traceback_later()
        with self._cv:
            self._started = None
            self._cv.notify()

    def _watch(self):
        with self._cv:
            while not self._closed:
                if self._started is None:
                    self._cv.wait()
                    continue
                serial, left = self._serial, self._started + self.deadline - self.clock()
                if left > 0:
                    self._cv.wait(left)
                    continue
                if self._started is not None and serial == self._serial:
                    label, elapsed = self._label, self.clock() - self._started
                    self._started = None
                    self.on_expire(label, elapsed, self.deadline)

    def close(self):
        if self._thread is None:
            return
        if self.backstop is not None:
            faulthandler.cancel_dump_traceback_later()
        with self._cv:
            self._closed = True
            self._cv.notify()
        self._thread.join(timeout=1.0)


class _Armed:
    def __init__(self, watchdog, label):
        self.watchdog, self.label = watchdog, label

    def __enter__(self):
        self.watchdog._begin(self.label)
        return self

    def __exit__(self, *_exc):
        self.watchdog._end()
        return False
