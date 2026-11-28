"""The world server's protocol side (roadmaps 037, 046): requests in, a `World` driven,
replies out.

Pure Python, so it is tested without a simulator (`test/test_server.py`, on a fake world).
`run()` puts a `World` on a socket: what a simulator's entry point calls (the Isaac one is
`isaac_world.world_server`).

The shell owns the protocol so a `World` does not have to: the version check, the owner lease
(one bridge at a time; `proto/scensim_world.proto`, "Ownership"), handles, request validation
against the declared capabilities, the drives an `Apply` hands the next `Step`, the clock.
"""
import logging
import math
import os
import signal
import time
import uuid

from . import lifetime
from . import protocol as pb
from .watchdog import LOAD_DEADLINE, STEP_DEADLINE, StepWatchdog
from .world import Drive, World  # noqa: F401 - re-exported for World implementations

log = logging.getLogger("scensim_world_server")

# How long an owner may say nothing before another client may take the server over, s. The
# bridge probes every few seconds between scenarios (its idle check), so a live one never
# lapses.
LEASE_SECONDS = 15.0

# How long a refused server (its port held) waits before it exits 1.
REFUSED_PAUSE = 30


def drive_of(entity_pose):
    """The `Drive` an `EntityPose` asks for."""
    return Drive(
        pose=pb.pose_tuple(entity_pose.pose) if entity_pose.HasField("pose") else None,
        twist=pb.twist_tuple(entity_pose.twist) if entity_pose.HasField("twist") else None,
        teleport=entity_pose.teleport, kinematic=entity_pose.kinematic)


def signal_dict(signal_):
    """A `Signal` of `LoadWorld` as the world takes it."""
    return {"way_id": signal_.way_id,
            "regulatory_element_ids": list(signal_.regulatory_element_ids),
            "opendrive_id": signal_.opendrive_id,
            "position": ((signal_.position.x, signal_.position.y, signal_.position.z)
                         if signal_.HasField("position") else None)}


def environment_weather(w):
    """A `ScenarioWeather` as the world takes it: only the parts given (the rest unchanged)."""
    out = {}
    if w.HasField("cloud_cover"):
        out["cloud_cover_oktas"] = w.cloud_cover.oktas
    if w.HasField("sun"):
        out["sun"] = (w.sun.azimuth, w.sun.elevation, w.sun.illuminance)
    if w.HasField("fog"):
        out["fog_visual_range"] = w.fog.visual_range
    if w.HasField("precipitation"):
        kind = pb.PRECIPITATION_NAME.get(w.precipitation.type, "DRY").lower()
        out["precipitation"] = (kind, w.precipitation.intensity)
    if w.HasField("wind"):
        out["wind"] = (w.wind.direction, w.wind.speed)
    return out


def scenario_weather(d):
    """`environment_weather`'s inverse: a dict of the parts a world shows as `ScenarioWeather`
    (092 K9, `ENVIRONMENT_APPLIED`)."""
    w = pb.ScenarioWeather()
    if "cloud_cover_oktas" in d:
        w.cloud_cover.oktas = int(d["cloud_cover_oktas"])
    if "sun" in d:
        az, el, lx = (tuple(d["sun"]) + (0.0,))[:3]
        w.sun.azimuth, w.sun.elevation, w.sun.illuminance = az, el, lx or 0.0
    if "fog_visual_range" in d:
        w.fog.visual_range = float(d["fog_visual_range"])
    if "precipitation" in d:
        kind, mm = d["precipitation"]
        w.precipitation.type = pb.PRECIPITATION[kind.upper()]
        w.precipitation.intensity = float(mm)
    if "wind" in d:
        w.wind.direction, w.wind.speed = (float(x) for x in d["wind"])
    return w


def describe(client):
    return f"{client.host}:{client.pid}" if client.pid else "(unnamed client)"


def same_client(a, b):
    return (a.host, a.pid, a.start_time) == (b.host, b.pid, b.start_time)



SCENSIM_PROPERTY_PREFIX = "scensim."
# the prefix before the rename (roadmap 080): refused by name, never read as the new one
RETIRED_PROPERTY_PREFIX = "swb."


def retired_property(name):
    """A property under the retired prefix, as its refusal names it (else None):
    `swb.zone.ford.class: renamed scensim.zone.ford.class`."""
    if name.startswith(RETIRED_PROPERTY_PREFIX):
        rest = name[len(RETIRED_PROPERTY_PREFIX):]
        return f"{name}: renamed {SCENSIM_PROPERTY_PREFIX}{rest} (roadmap 080, scensim)"
    return None

class RequestError(Exception):
    """A request the server refuses: the reply's error, the server goes on."""


class WorldServer:
    def __init__(self, world, lease_seconds=LEASE_SECONDS, clock=time.monotonic,
                 is_gone=lifetime.is_gone, watchdog=None):
        self.world = world
        # Every request under a deadline (G70); `run` gives it one, a test may give its own.
        self.watchdog = watchdog or StepWatchdog(None)
        self.load_deadline = LOAD_DEADLINE
        self.capabilities = {pb.CAPABILITY[c] for c in world.capabilities}
        self.lease_seconds = lease_seconds
        self.clock = clock
        self.is_gone = is_gone
        self.instance = uuid.uuid4().hex
        self.owner = None  # ClientId
        self.owner_seen = 0.0
        self.next_handle = 1
        self.entities = {}  # handle -> name
        # handle -> motion, for the entities no Step has run since their spawn: only these
        # take SetInitialSpeed (scensim issue 204)
        self.unstepped = {}
        self.pending = {}  # handle -> Drive, from the last Apply
        self.sim_time = 0.0
        self.frame = 0
        self.episode_id = 0  # 0 until the first LoadWorld (roadmap 051)
        # READBACK_AFTER_STEP (scensim issue 161): a SetEnvironment since the last step, and
        # the zone ids of a SetRoadCondition since it (None: none sent)
        self.environment_sent = False
        self.road_condition_sent = None
        self.loaded_world = None
        self.stopped = False
        self.step_ms = []  # wall time of each step since the last summary (038)

    def offers(self, name):
        return pb.CAPABILITY[name] in self.capabilities

    def handle(self, data: bytes) -> bytes:
        """One request's bytes to its reply's bytes. Never raises."""
        try:
            request = pb.Request.FromString(data)
        except Exception as e:  # noqa: BLE001 - a malformed request is the peer's error
            return pb.Reply(ok=False, error=f"undecodable request ({len(data)} bytes): {e}") \
                .SerializeToString()
        which = request.WhichOneof("request")
        try:
            if which is None:
                raise RequestError("empty request")
            # armed before anything of the request runs: a Hello may reset the last owner's
            # entities, an Apply or a Spawn may enter the world (G70)
            with self.watchdog.step(self._label(which, request),
                                    self.load_deadline if which == "load_world" else None):
                if which == "hello":
                    reply = self._hello(request.hello, request.client)
                else:
                    self._check_owner(which, request.client)
                    reply = getattr(self, f"_{which}")(getattr(request, which))
        except RequestError as e:
            reply = pb.Reply(ok=False, error=str(e))
        except Exception as e:  # noqa: BLE001 - the world failed; the bridge fails the frame
            log.exception("%s failed", which)
            reply = pb.Reply(ok=False, error=f"{which} failed in the world server: {e}")
        return reply.SerializeToString()

    def _label(self, which, request):
        """What the watchdog names when a request outlives its deadline."""
        if which == "step":
            r = request.step
            return (f"step {self.frame + 1} ({r.substeps} x {r.dt:g} s at sim time "
                    f"{self.sim_time:.3f} s)")
        if which == "load_world":
            return f"load_world {request.load_world.world}"
        return f"{which} request (frame {self.frame})"

    # --- Ownership -------------------------------------------------------------------------

    def _check_owner(self, which, client):
        if self.owner is None:
            raise RequestError(f"{which} before hello: say hello first")
        if not same_client(client, self.owner):
            raise RequestError(f"{which} from {describe(client)} refused: this world server is "
                               f"owned by {describe(self.owner)}")
        self.owner_seen = self.clock()

    def _take(self, client, why):
        if self.owner is not None:
            log.info("owner %s replaced by %s (%s)", describe(self.owner), describe(client), why)
        else:
            log.info("owner: %s", describe(client))
        self.owner = pb.ClientId()
        self.owner.CopyFrom(client)
        self.owner_seen = self.clock()
        self._reset()

    def _reset(self):
        """Every entity of the last owner goes; the world stays."""
        if self.entities:
            log.info("removing the %d entit%s of the last owner", len(self.entities),
                     "y" if len(self.entities) == 1 else "ies")
            self.world.reset(list(self.entities))
        self.entities.clear()
        self.unstepped.clear()
        self.pending.clear()

    def _hello(self, req, client):
        if req.protocol_version != pb.PROTOCOL_VERSION:
            raise RequestError(
                f"protocol version {req.protocol_version} refused: this world server speaks "
                f"{pb.PROTOCOL_VERSION}")
        if not req.probe:
            if self.owner is None:
                self._take(client, "first hello")
            elif same_client(client, self.owner):
                self.owner_seen = self.clock()
            elif self.is_gone(self.owner):
                self._take(client, "the owner's process is gone")
            elif self.clock() - self.owner_seen > self.lease_seconds:
                self._take(client, f"the owner was silent for more than {self.lease_seconds:g} s")
            else:
                raise RequestError(
                    f"owned by {describe(self.owner)} (seen {self.clock() - self.owner_seen:.0f} s "
                    f"ago): one bridge drives a world server at a time")
        ready, why = self.world.ready()
        reply = pb.HelloReply(
            protocol_version=pb.PROTOCOL_VERSION,
            simulator_version=self.world.simulator_version,
            simulator_name=self.world.simulator_name,
            capabilities=sorted(self.capabilities),
            instance=self.instance, max_physics_dt=self.world.max_physics_dt,
            ready=ready, not_ready_reason=why)
        if self.owner is not None:
            reply.owner.CopyFrom(self.owner)
        return pb.Reply(ok=True, hello=reply)

    def _release(self, _req):
        log.info("released by %s", describe(self.owner))
        self.step_summary()
        self._reset()
        self.owner = None
        return pb.Reply(ok=True)

    # --- The world ---------------------------------------------------------------------------

    def _load_world(self, req):
        ready, why = self.world.ready()
        if not ready:
            raise RequestError(f"not ready to load a world: {why}")
        signals = [signal_dict(s) for s in req.signals] if self.offers("SIGNALS") else []
        self.world.load(req.world, list(req.ground_paths), signals, request=req)
        self.entities.clear()
        self.unstepped.clear()
        self.pending.clear()
        # A new episode when the world changes (roadmap 051); this server's time keeps
        # running across them, which nothing may depend on.
        if self.episode_id == 0 or req.world != self.loaded_world:
            self.episode_id += 1
            self.loaded_world = req.world
        log.info("world %s loaded, episode %d", req.world or "(default)", self.episode_id)
        return pb.Reply(ok=True, load_world=pb.LoadWorldReply(
            sim_time_ns=pb.sim_ns(self.sim_time), episode_id=self.episode_id))

    def _spawn(self, req):
        if req.name in self.entities.values():
            raise RequestError(f"an entity named {req.name!r} exists")
        d = req.box_dimensions
        if not (d.x > 0 and d.y > 0 and d.z > 0):
            raise RequestError(f"{req.name!r}: box dimensions {d.x}, {d.y}, {d.z} are not a box")
        kind = pb.KIND_NAME.get(req.kind, "KIND_UNSPECIFIED")
        motion = pb.MOTION_NAME.get(req.motion, "KINEMATIC")
        needs = {"TRACKED": ("TRACKED_MOTION", "TRACKS_TARGETS"), "AUTOPILOT": ("AUTOPILOT",),
                 "COMMANDED": ("COMMANDS",)}
        if motion in needs and not any(self.offers(c) for c in needs[motion]):
            raise RequestError(f"{req.name!r}: {motion} bodies are not offered by this server")
        if motion == "EXTERNAL" and kind != "EGO" and not self.offers("AGENT_VEHICLES"):
            raise RequestError(f"{req.name!r}: externally driven vehicles other than the ego "
                               "are not offered by this server")
        handle = self.next_handle
        c = req.box_center
        body = self.world.spawn(handle, req.name, kind, req.subtype, (c.x, c.y, c.z),
                                (d.x, d.y, d.z), pb.pose_tuple(req.pose), model=req.model,
                                motion=motion)
        self.next_handle += 1
        self.entities[handle] = req.name
        self.unstepped[handle] = motion
        body = body or {}

        def vec(v):
            return pb.Vec3(x=v[0], y=v[1], z=v[2])

        reply = pb.SpawnReply(handle=handle, body=pb.Body(
            model=body.get("model", f"{self.world.simulator_name}:box"),
            box_location=vec(body.get("box_location", (c.x, c.y, c.z))),
            box_extent=vec(body.get("box_extent", (d.x / 2, d.y / 2, d.z / 2))), has_box=True,
            origin_offset=vec(body.get("origin_offset", (0.0, 0.0, 0.0))),
            declared_center=vec((c.x, c.y, c.z)), declared_dimensions=vec((d.x, d.y, d.z))))
        if body.get("spec_hash"):
            reply.body.spec_hash = body["spec_hash"]
        if body.get("wheels"):
            front_x, rear_x, track, radius = body["wheels"]
            reply.body.wheels.CopyFrom(pb.Wheels(front_x=front_x, rear_x=rear_x, track=track,
                                                 radius=radius))
        return pb.Reply(ok=True, spawn=reply)

    def step_summary(self):
        """Wall time per step since the last summary, logged and returned (p50, p95, max ms)."""
        if not self.step_ms:
            return None
        t = sorted(self.step_ms)
        p50, p95, top = t[len(t) // 2], t[min(len(t) - 1, int(0.95 * len(t)))], t[-1]
        log.info("%d step(s): wall time per step p50 %.1f ms, p95 %.1f ms, max %.0f ms",
                 len(t), p50, p95, top)
        self.step_ms.clear()
        return p50, p95, top

    def _despawn(self, req):
        if req.handle not in self.entities:
            raise RequestError(f"no entity with handle {req.handle}")
        if len(self.entities) == 1:  # the last one: SSv2 ends its scenario this way
            self.step_summary()
        self.world.despawn(req.handle)
        del self.entities[req.handle]
        self.unstepped.pop(req.handle, None)
        self.pending.pop(req.handle, None)
        return pb.Reply(ok=True)

    def _apply(self, req):
        unknown = [p.handle for p in req.poses if p.handle not in self.entities]
        if unknown:
            raise RequestError(f"poses for unknown handle(s) {unknown}")
        for p in req.poses:
            if p.HasField("pose") or p.HasField("twist") or p.kinematic:
                self.pending[p.handle] = drive_of(p)
        return pb.Reply(ok=True, apply=pb.ApplyReply())

    def _step(self, req):
        if req.substeps < 1 or not req.dt > 0:
            raise RequestError(f"step of {req.substeps} x {req.dt} s")
        if req.poses:
            raise RequestError("Step carries no poses since protocol 4: send them by Apply")
        drives, self.pending = self.pending, {}
        started = time.perf_counter()
        states, contacts = self.world.step(drives, req.substeps, req.dt, req.render,
                                           self.sim_time)
        self.unstepped.clear()
        elapsed = time.perf_counter() - started
        self.step_ms.append(1e3 * elapsed)
        self.sim_time += req.substeps * req.dt
        self.frame += 1
        now_ns = pb.sim_ns(self.sim_time)
        reply = pb.StepReply(sim_time=now_ns / 1e9, frame=self.frame, tick_seconds=elapsed,
                             sim_time_ns=now_ns, episode_id=self.episode_id)
        for handle, pose, twist, *rest in states:
            state = reply.states.add(handle=handle, pose=pb.make_pose(*pose))
            if twist is not None:
                state.twist.CopyFrom(pb.make_twist(*twist))
            command = rest[0] if rest else None
            if command is not None:      # an ego's taken command (World.step's docstring)
                state.command.CopyFrom(pb.TakenCommand(**command))
        for handle, other, path, impulse in contacts:
            reply.contacts.add(handle=handle, other=other, other_path=path, impulse=impulse)
        self._read_back_after_step(reply)
        return pb.Reply(ok=True, step=reply)

    def _read_back_after_step(self, reply):
        """READBACK_AFTER_STEP (scensim issue 161): into a step's reply, after the world's
        step, the read-back of a SetEnvironment or SetRoadCondition sent since the last one."""
        zone_ids, self.road_condition_sent = self.road_condition_sent, None
        sent, self.environment_sent = self.environment_sent, False
        if zone_ids is not None and self.offers("ROAD_CONDITION_APPLIED"):
            reply.road_condition_applied.CopyFrom(self._road_condition_reply(zone_ids))
        if sent and self.offers("ENVIRONMENT_APPLIED"):
            reply.environment_applied.CopyFrom(
                scenario_weather(self.world.applied_environment()))

    def _set_step_time(self, _req):
        return pb.Reply(ok=True)

    def _set_signals(self, req):
        if not self.offers("SIGNALS"):
            raise RequestError("traffic signals are not offered by this server")
        self.world.set_signals({s.opendrive_id: pb.COLOUR_NAME.get(s.colour, "OFF")
                                for s in req.states})
        return pb.Reply(ok=True)

    def _get_signals(self, req):
        if not self.offers("SIGNALS"):
            raise RequestError("traffic signals are not offered by this server")
        try:
            shown = self.world.get_signals(list(req.opendrive_ids))
        except ValueError as e:
            raise RequestError(str(e)) from e
        return pb.Reply(ok=True, signals=pb.SignalsReply(states=[
            pb.SignalState(opendrive_id=od, colour=pb.COLOUR[colour])
            for od, colour in sorted(shown.items())]))

    def _set_initial_speed(self, req):
        if not self.offers("INITIAL_SPEED"):
            raise RequestError("initial speeds are not offered by this server")
        if req.handle not in self.entities:
            raise RequestError(f"no entity with handle {req.handle}")
        name = self.entities[req.handle]
        motion = self.unstepped.get(req.handle)
        if motion is None:
            raise RequestError(f"{name!r}: an initial speed after its first Step is refused")
        if motion in ("KINEMATIC", "TRACKED"):
            raise RequestError(f"{name!r}: a {motion} body is posed, so its speed comes from "
                               "its poses; no initial speed")
        if not (math.isfinite(req.speed) and req.speed >= 0.0):
            raise RequestError(f"{name!r}: initial speed {req.speed} m/s; it must be finite "
                               "and >= 0")
        self.world.set_initial_speed(req.handle, req.speed)
        return pb.Reply(ok=True)

    def _set_goal(self, _req):
        raise RequestError("goals are not offered by this server (no AUTOPILOT)")

    def _set_weather(self, req):
        if not self.offers("WEATHER"):
            raise RequestError("weather is not offered by this server")
        w = req.weather
        if w.use_parameters and w.parameters:
            preset, parameters = "", dict(w.parameters)
        elif not w.use_parameters and w.preset.strip():
            preset, parameters = w.preset.strip(), {}
        else:
            raise RequestError("give a preset, or parameters by name")
        try:
            now = self.world.set_weather(preset, parameters)
        except ValueError as e:
            return pb.Reply(ok=True, weather=pb.WeatherReply(success=False, message=str(e)))
        return pb.Reply(ok=True, weather=self._weather_reply(now, "applied"))

    def _get_weather(self, _req):
        if not self.offers("WEATHER"):
            raise RequestError("weather is not offered by this server")
        return pb.Reply(ok=True, weather=self._weather_reply(self.world.get_weather(), ""))

    @staticmethod
    def _weather_reply(now, message):
        preset, parameters = now
        return pb.WeatherReply(success=True, message=message, weather=pb.Weather(
            preset=preset, parameters=dict(parameters), use_parameters=True))

    def _set_road_condition(self, req):
        if not self.offers("ROAD_CONDITION"):
            raise RequestError("road conditions are not offered by this server")
        rc = req.road_condition
        retired = [retired_property(n) for n in rc.properties if retired_property(n)]
        if retired:
            raise RequestError("RoadCondition refused:\n" + "\n".join(retired))
        try:
            self.world.set_road_condition(rc.friction_scale_factor, rc.wetness or "dry",
                                          dict(rc.properties))
        except ValueError as e:
            raise RequestError(f"RoadCondition refused:\n{e}") from e
        zone_ids = [n.split(".")[2] for n in rc.properties
                    if n.startswith("scensim.zone.") and len(n.split(".")) > 3]
        if self.offers("READBACK_AFTER_STEP"):
            self.road_condition_sent = zone_ids
        if self.offers("ROAD_CONDITION_APPLIED"):
            # 092 K9f-a, 086 C7: the road state the world applies now, read back
            return pb.Reply(ok=True, road_condition=self._road_condition_reply(zone_ids))
        return pb.Reply(ok=True)

    def _road_condition_reply(self, zone_ids):
        a = self.world.applied_road_condition(zone_ids)
        zones = {}
        for zid, z in (a.get("zones") or {}).items():
            za = pb.ZoneApplied(surface_class=z["surface_class"], wetness=z["wetness"],
                                mu=float(z["mu"]))
            if z.get("water_depth") is not None:
                za.water_depth = float(z["water_depth"])
            zones[zid] = za
        return pb.RoadConditionReply(
            friction_scale_factor=float(a["friction_scale_factor"]), wetness=a["wetness"],
            zones=zones)

    def _set_environment(self, req):
        if not self.offers("ENVIRONMENT"):
            raise RequestError("the weather and time of day are not offered by this server")
        problems = environment_range_problems(req.weather) if req.HasField("weather") else []
        if problems:
            raise RequestError("SetEnvironment refused, nothing changed:\n" + "\n".join(problems))
        weather = environment_weather(req.weather) if req.HasField("weather") else None
        time_of_day = None
        if req.HasField("time_of_day"):
            t = req.time_of_day
            time_of_day = {"hours": t.hours, "day_of_year": t.day_of_year,
                           "animation": t.animation, "date_time": t.date_time}
        try:
            self.world.set_environment(weather, time_of_day, self.sim_time)
        except ValueError as e:
            raise RequestError(f"Weather refused:\n{e}") from e
        if self.offers("READBACK_AFTER_STEP"):
            self.environment_sent = True
        if self.offers("ENVIRONMENT_APPLIED"):
            # 092 K9: what the world shows now, read back, for a knob profile's record
            return pb.Reply(ok=True, environment=pb.EnvironmentReply(
                applied=scenario_weather(self.world.applied_environment())))
        return pb.Reply(ok=True)

    def _shutdown(self, _req):
        self.stopped = True
        return pb.Reply(ok=True)


def _num(v):
    """A number as the Rust shell writes it (`{}` of an f64): 60, 1.6, NaN."""
    if math.isnan(v):
        return "NaN"
    return f"{v:.0f}" if float(v).is_integer() else repr(float(v))


def environment_range_problems(w):
    """Every field of ScenarioWeather `w` outside its range (scensim issue 161), one line each,
    naming the field, the value and the range with their unit; an angle also says what the
    number would be as degrees (092 K9e sent 60, meant as degrees, as an elevation in radians).
    The Rust shell's `environment_range_problems`, word for word."""
    out = []

    def angle(field, v, ok, text):
        if not ok:
            out.append(f"{field} {_num(v)} rad is outside {text} rad "
                       f"({_num(v)} deg would be {math.radians(v):.4f} rad)")

    if w.HasField("sun"):
        a, e = w.sun.azimuth, w.sun.elevation
        angle("sun_azimuth", a, 0.0 <= a < 2 * math.pi, "[0, 6.2832)")
        angle("sun_elevation", e, -math.pi / 2 <= e <= math.pi / 2, "[-1.5708, 1.5708]")
    if w.HasField("cloud_cover") and w.cloud_cover.oktas > 9:
        out.append(f"cloud_cover {w.cloud_cover.oktas} oktas is outside 0 to 9 oktas "
                   "(9: the sky cannot be seen)")
    if w.HasField("fog") and not w.fog.visual_range > 0.0:
        out.append(f"fog_visual_range {_num(w.fog.visual_range)} m is outside its range, "
                   "above 0 m")
    if w.HasField("precipitation") and not w.precipitation.intensity >= 0.0:
        out.append(f"precipitation_intensity {_num(w.precipitation.intensity)} mm/h is outside "
                   "its range, 0 or more mm/h")
    return out


def serve(socket, server, should_stop=lambda: False, poll_ms=100):
    """Answer requests on a bound REP `socket` until `Shutdown` or `should_stop()`. The pass
    between requests is under the deadline too: `should_stop` may call into the simulator
    (Isaac's `app.is_running()`)."""
    import zmq

    def stopping():
        with server.watchdog.step(f"the loop between requests (after frame {server.frame})"):
            return should_stop()

    while not server.stopped and not stopping():
        if not socket.poll(poll_ms, zmq.POLLIN):
            continue
        socket.send(server.handle(socket.recv()))


def bind(port, host="127.0.0.1"):
    """The REP socket on `tcp://host:port`, bound now: a server binds before its simulator's
    long start, so a second one on the port fails at once (`Address already in use`) and
    exits 1, which its supervisor does not respawn. Requests that arrive before `run` wait
    in the socket's queue."""
    import zmq

    socket = zmq.Context.instance().socket(zmq.REP)
    try:
        socket.bind(f"tcp://{host}:{port}")
    except zmq.ZMQError as e:
        # A supervisor respawns on any exit: wait, so a refusal is not a loop.
        log.error("cannot bind tcp://%s:%d: %s (another world server holds it); exiting in "
                  "%d s", host, port, e, REFUSED_PAUSE)
        time.sleep(REFUSED_PAUSE)
        raise SystemExit(1) from e
    return socket


def run(world, socket, should_stop=lambda: False, step_deadline=STEP_DEADLINE,
        load_deadline=LOAD_DEADLINE, watchdog=None):
    """Serve `world` on the bound `socket` until `Shutdown`, SIGINT/SIGTERM or
    `should_stop()`. A request (or the loop between them) that runs past its deadline,
    `load_deadline` for a LoadWorld and `step_deadline` for the rest, ends the process: a
    watchdog child process dumps its stacks and kills it, for its supervisor to start a fresh
    one (`watchdog`, roadmap 077 G70). `None` or 0: no deadline. `watchdog`: a
    `StepWatchdog` the caller started earlier (to watch its own start, csb issue 042),
    used instead of a new one with `step_deadline`; it is closed when the server ends."""
    server = WorldServer(world, watchdog=watchdog or StepWatchdog(step_deadline))
    server.load_deadline = load_deadline
    stop = {"now": False}

    def on_signal(signum, _frame):
        log.info("signal %d: stopping", signum)
        stop["now"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    import zmq

    step_deadline = getattr(server.watchdog, "deadline", step_deadline)
    log.info("%s world server ready on %s (%s, pid %d, capabilities %s, deadlines: %s)",
             world.simulator_name, socket.getsockopt_string(zmq.LAST_ENDPOINT),
             world.simulator_version, os.getpid(),
             ", ".join(sorted(world.capabilities)) or "core only",
             f"{step_deadline:g} s, world load {load_deadline:g} s" if step_deadline else "off")
    try:
        serve(socket, server, should_stop=lambda: stop["now"] or should_stop())
    finally:
        log.info("world server stopping after %d step(s), sim time %.3f s",
                 server.frame, server.sim_time)
        server.watchdog.close()
        socket.close(linger=0)
        world.close()
    return server
