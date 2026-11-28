"""The world server's protocol side on a fake world (roadmaps 037, 046); no simulator needed.

    python3 -m unittest discover -s src/scensim_world_server/test     # protobuf only
"""
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scensim_world_server import lifetime  # noqa: E402
from scensim_world_server import protocol as pb  # noqa: E402
from scensim_world_server.server import Drive, WorldServer  # noqa: E402
from scensim_world_server.watchdog import StepWatchdog  # noqa: E402
from scensim_world_server.world import World  # noqa: E402


class FakeWorld(World):
    """Moves bodies exactly to their poses; the ego touches whatever is within 1 m."""

    simulator_name = "fake"
    capabilities = ("SIGNALS", "TRACKED_MOTION", "CONTACTS")
    max_physics_dt = 1 / 60

    def __init__(self):
        self.bodies = {}
        self.loads = []
        self.steps = []
        self.command = None   # what an EGO ran this step (TakenCommand fields), or None
        self.resets = []

    @property
    def simulator_version(self):
        return "fake-0"

    def load(self, world, ground_paths, signals=(), request=None):
        self.loads.append((world, ground_paths))
        self.signals = list(signals)
        self.colours = {}
        self.bodies.clear()

    def set_signals(self, colours):
        self.colours.update(colours)

    def get_signals(self, opendrive_ids):
        unknown = [od for od in opendrive_ids if od not in self.colours]
        if unknown:
            raise ValueError(f"no lamp for OpenDRIVE id(s) {', '.join(unknown)}")
        return {od: self.colours[od] for od in (opendrive_ids or self.colours)}

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="", motion="KINEMATIC"):
        self.bodies[handle] = {"name": name, "kind": kind, "pose": pose, "dims": dims,
                               "model": model, "motion": motion, "twist": None}
        if kind == "EGO":  # the car it built, wheels in the entity frame (roadmap 051)
            return {"model": "fake:car", "wheels": (3.0, 0.0, 1.6, 0.35), "spec_hash": "abc"}

    def despawn(self, handle):
        del self.bodies[handle]

    def reset(self, handles):
        self.resets.append(sorted(handles))
        super().reset(handles)

    def step(self, drives, substeps, dt, render, sim_time):
        self.steps.append((dict(drives), substeps, dt, render, sim_time))
        for h, d in drives.items():
            b = self.bodies[h]
            if d.kinematic:
                b["motion"] = "KINEMATIC"
            if b["motion"] == "TRACKED":
                if d.teleport:
                    b["pose"] = d.pose
                if d.twist is not None:
                    (x, y, z), q = b["pose"]
                    v = d.twist[0]
                    t = substeps * dt
                    b["pose"] = ((x + v[0] * t, y + v[1] * t, z + v[2] * t), q)
                    b["twist"] = d.twist
            elif d.pose is not None:
                b["pose"] = d.pose
        states = [(h, b["pose"], b["twist"] if b["motion"] == "TRACKED" else None)
                  + ((self.command,) if b["kind"] == "EGO" else ())
                  for h, b in self.bodies.items()]
        contacts = []
        for h, b in self.bodies.items():
            if b["kind"] != "EGO":
                continue
            for o, other in self.bodies.items():
                if o != h and abs(other["pose"][0][0] - b["pose"][0][0]) < 1.0:
                    contacts.append((h, o, f"/World/entities/e{o}", 2.5))
        return states, contacts


BRIDGE = pb.ClientId(host="testhost", pid=100, start_time=1)
OTHER = pb.ClientId(host="testhost", pid=200, start_time=2)


def call(server, client=BRIDGE, **request):
    return pb.Reply.FromString(server.handle(
        pb.Request(client=client, **request).SerializeToString()))


def hello(server, client=BRIDGE, probe=False):
    return call(server, client=client,
                hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION, probe=probe))


def spawn(server, name, kind, x, motion="KINEMATIC"):
    return call(server, spawn=pb.Spawn(
        name=name, kind=pb.KIND[kind], box_center=pb.Vec3(x=1.5, z=0.9),
        box_dimensions=pb.Vec3(x=4.5, y=2.1, z=1.8), motion=pb.MOTION[motion],
        pose=pb.make_pose((x, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))))


def step(server, *poses, substeps=3, dt=1 / 60, render=False):
    """An Apply with `poses`, then a Step: one SSv2 frame."""
    r = call(server, apply=pb.Apply(poses=list(poses)))
    assert r.ok, r.error
    return call(server, step=pb.Step(substeps=substeps, dt=dt, render=render))


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.world = FakeWorld()
        self.clock = Clock()
        self.gone = set()
        self.server = WorldServer(self.world, lease_seconds=15, clock=self.clock,
                                  is_gone=lambda c: c.pid in self.gone)

    def test_hello_first_and_only_in_our_version(self):
        r = call(self.server, load_world=pb.LoadWorld())
        self.assertFalse(r.ok)
        self.assertIn("before hello", r.error)
        r = call(self.server, hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION + 1))
        self.assertFalse(r.ok)
        self.assertIn(f"speaks {pb.PROTOCOL_VERSION}", r.error)
        r = hello(self.server)
        self.assertTrue(r.ok)
        h = r.hello
        self.assertEqual((h.simulator_name, h.simulator_version), ("fake", "fake-0"))
        self.assertEqual(h.protocol_version, pb.PROTOCOL_VERSION)
        self.assertEqual({pb.CAPABILITY_NAME[c] for c in h.capabilities},
                         {"SIGNALS", "TRACKED_MOTION", "CONTACTS"})
        self.assertAlmostEqual(h.max_physics_dt, 1 / 60)
        self.assertEqual(h.owner.pid, BRIDGE.pid)
        self.assertTrue(h.ready)
        self.assertEqual(len(h.instance), 32)

    def test_spawn_step_despawn(self):
        hello(self.server)
        r = call(self.server, load_world=pb.LoadWorld())
        self.assertTrue(r.ok)
        self.assertEqual(r.load_world.sim_time_ns, pb.BASE_NS)
        self.assertEqual(r.load_world.episode_id, 1)
        r = spawn(self.server, "ego", "EGO", 0.0, motion="EXTERNAL")
        ego = r.spawn.handle
        self.assertEqual(r.spawn.body.model, "fake:car")
        self.assertAlmostEqual(r.spawn.body.box_extent.x, 2.25)
        w = r.spawn.body.wheels
        self.assertEqual((w.front_x, w.rear_x, w.track, w.radius), (3.0, 0.0, 1.6, 0.35))
        self.assertEqual(r.spawn.body.spec_hash, "abc")
        r = spawn(self.server, "npc", "VEHICLE", -10.0)
        self.assertEqual(r.spawn.body.model, "fake:box")
        self.assertFalse(r.spawn.body.HasField("wheels"))
        npc = r.spawn.handle
        self.assertEqual((ego, npc), (1, 2))
        self.assertEqual(self.world.bodies[ego]["dims"], (4.5, 2.1, 1.8))
        r = step(self.server, pb.EntityPose(handle=npc, pose=pb.make_pose((-0.5, 0, 0), (1, 0, 0, 0))),
                 pb.EntityPose(handle=ego), render=True)
        self.assertTrue(r.ok, r.error)
        self.assertAlmostEqual(r.step.sim_time, 1000.05)
        self.assertEqual(r.step.frame, 1)
        # An EXTERNAL body is listed with no drive: the step does not touch it.
        self.assertEqual(set(self.world.steps[-1][0]), {npc})
        states = {s.handle: pb.pose_tuple(s.pose) for s in r.step.states}
        self.assertEqual(states[npc][0], (-0.5, 0.0, 0.0))
        # no command run yet: the ego's state has none (an NPC never does)
        self.assertFalse(any(s.HasField("command") for s in r.step.states))
        self.world.command = {"stamp_ns": 1000_000_000_000, "acceleration": 1.0,
                              "velocity": 3.0, "tire_angle": 0.1, "gear": 2, "arrived": 2}
        r = step(self.server)
        c = {s.handle: s for s in r.step.states}
        self.assertTrue(c[ego].HasField("command") and not c[npc].HasField("command"))
        self.assertEqual((c[ego].command.stamp_ns, c[ego].command.arrived), (1000_000_000_000, 2))
        self.assertAlmostEqual(c[ego].command.tire_angle, 0.1)
        self.world.command = None
        self.assertEqual(len(r.step.contacts), 1)
        c = r.step.contacts[0]
        self.assertEqual((c.handle, c.other, c.impulse), (ego, npc, 2.5))
        # Drives are used once: the next step has none.
        step(self.server)
        self.assertEqual(self.world.steps[-1][0], {})
        self.assertTrue(call(self.server, despawn=pb.Despawn(handle=npc)).ok)
        r = call(self.server, despawn=pb.Despawn(handle=npc))
        self.assertFalse(r.ok)
        self.assertIn("no entity with handle 2", r.error)

    def test_a_tracked_body_takes_twists_teleports_and_the_fallback(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        npc = spawn(self.server, "npc", "VEHICLE", 0.0, motion="TRACKED").spawn.handle
        self.assertEqual(self.world.bodies[npc]["motion"], "TRACKED")
        twist = pb.make_twist((8.0, 0.0, 0.0), (0.0, 0.0, 0.1))
        r = step(self.server, pb.EntityPose(handle=npc, twist=twist))
        self.assertTrue(r.ok, r.error)
        drive = self.world.steps[-1][0][npc]
        self.assertEqual(drive, Drive(pose=None, twist=((8.0, 0.0, 0.0), (0.0, 0.0, 0.1))))
        state = r.step.states[0]
        self.assertAlmostEqual(state.pose.position.x, 0.4)
        self.assertEqual(pb.twist_tuple(state.twist), ((8.0, 0.0, 0.0), (0.0, 0.0, 0.1)))
        r = step(self.server, pb.EntityPose(
            handle=npc, teleport=True, twist=twist,
            pose=pb.make_pose((10.0, 1.0, 0.0), (1.0, 0.0, 0.0, 0.0))))
        self.assertAlmostEqual(r.step.states[0].pose.position.x, 10.4)
        r = step(self.server, pb.EntityPose(
            handle=npc, kinematic=True, pose=pb.make_pose((20.0, 1.0, 0.0), (1.0, 0.0, 0.0, 0.0))))
        self.assertEqual(self.world.bodies[npc]["motion"], "KINEMATIC")
        self.assertAlmostEqual(r.step.states[0].pose.position.x, 20.0)
        self.assertFalse(r.step.states[0].HasField("twist"))

    def test_what_a_server_does_not_offer_is_refused(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        r = spawn(self.server, "ap", "VEHICLE", 0.0, motion="AUTOPILOT")
        self.assertFalse(r.ok)
        self.assertIn("AUTOPILOT bodies are not offered", r.error)
        r = spawn(self.server, "walker", "PEDESTRIAN", 0.0, motion="COMMANDED")
        self.assertFalse(r.ok)
        self.assertIn("COMMANDED bodies are not offered", r.error)
        r = spawn(self.server, "bg_av_1", "VEHICLE", 0.0, motion="EXTERNAL")
        self.assertFalse(r.ok)
        self.assertIn("other than the ego", r.error)
        for request in ({"set_goal": pb.SetGoal(handle=1)},
                        {"set_weather": pb.SetWeather()}, {"get_weather": pb.GetWeather()},
                        {"set_road_condition": pb.SetRoadCondition()},
                        {"set_environment": pb.SetEnvironment()},
                        {"set_initial_speed": pb.SetInitialSpeed(handle=1, speed=5.0)}):
            r = call(self.server, **request)
            self.assertFalse(r.ok)
            self.assertIn("not offered", r.error)
        self.world.capabilities = ()
        bare = WorldServer(self.world, clock=self.clock)
        hello(bare)
        for request in ({"set_signals": pb.SetSignals()}, {"get_signals": pb.GetSignals()}):
            r = call(bare, **request)
            self.assertFalse(r.ok)
            self.assertIn("signals are not offered", r.error)
        r = spawn(bare, "npc", "VEHICLE", 0.0, motion="TRACKED")
        self.assertFalse(r.ok)
        # No SIGNALS: the map's signals are not handed to the world.
        call(bare, load_world=pb.LoadWorld(signals=[pb.Signal(way_id=1, opendrive_id="1")]))
        self.assertEqual(self.world.signals, [])

    def test_the_road_condition_reaches_the_world_and_its_refusal_the_bridge(self):
        conditions = []

        def set_road_condition(friction_scale, wetness, properties):
            if "scensim.zone.nowhere.class" in properties:
                raise ValueError("scensim.zone.nowhere.class: no zone 'nowhere'")
            conditions.append((friction_scale, wetness, properties))

        self.world.capabilities = ("ROAD_CONDITION",)
        self.world.set_road_condition = set_road_condition
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        rc = pb.RoadCondition(friction_scale_factor=0.5, wetness="lowFlooded")
        rc.properties["scensim.zone.ford.water_depth"] = "0.12"
        r = call(server, set_road_condition=pb.SetRoadCondition(road_condition=rc))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(conditions, [(0.5, "lowFlooded", {"scensim.zone.ford.water_depth": "0.12"})])
        # No wetness is dry.
        call(server, set_road_condition=pb.SetRoadCondition(
            road_condition=pb.RoadCondition(friction_scale_factor=1.0)))
        self.assertEqual(conditions[-1], (1.0, "dry", {}))
        rc = pb.RoadCondition(friction_scale_factor=1.0)
        rc.properties["scensim.zone.nowhere.class"] = "ice"
        r = call(server, set_road_condition=pb.SetRoadCondition(road_condition=rc))
        self.assertFalse(r.ok)
        self.assertIn("RoadCondition refused:\nscensim.zone.nowhere.class: no zone", r.error)
        # the retired prefix (roadmap 080) is refused by name before the world sees it
        n = len(conditions)
        rc = pb.RoadCondition(friction_scale_factor=1.0)
        rc.properties["swb.zone.ford.water_depth"] = "0.12"
        r = call(server, set_road_condition=pb.SetRoadCondition(road_condition=rc))
        self.assertFalse(r.ok)
        self.assertIn("swb.zone.ford.water_depth: renamed scensim.zone.ford.water_depth", r.error)
        self.assertEqual(len(conditions), n)

    def test_the_weather_reaches_the_world_part_by_part(self):
        calls = []

        def set_environment(weather, time_of_day, sim_time):
            if weather and weather.get("precipitation", ("",))[0] == "snow":
                raise ValueError("snow: none here")
            calls.append((weather, time_of_day))

        self.world.capabilities = ("ENVIRONMENT",)
        self.world.set_environment = set_environment
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        w = pb.ScenarioWeather()
        w.fog.visual_range = 50.0
        w.precipitation.type = pb.PRECIPITATION["RAIN"]
        w.precipitation.intensity = 4.0
        r = call(server, set_environment=pb.SetEnvironment(
            weather=w, time_of_day=pb.TimeOfDay(hours=7.5, day_of_year=100, animation=True)))
        self.assertTrue(r.ok, r.error)
        weather, time_of_day = calls[-1]
        self.assertEqual(weather, {"fog_visual_range": 50.0, "precipitation": ("rain", 4.0)})
        self.assertEqual((time_of_day["hours"], time_of_day["animation"]), (7.5, True))
        call(server, set_environment=pb.SetEnvironment(time_of_day=pb.TimeOfDay(hours=1.0)))
        self.assertIsNone(calls[-1][0])
        w = pb.ScenarioWeather()
        w.precipitation.type = pb.PRECIPITATION["SNOW"]
        r = call(server, set_environment=pb.SetEnvironment(weather=w))
        self.assertFalse(r.ok)
        self.assertIn("Weather refused:\nsnow", r.error)

    def test_a_world_with_the_read_back_answers_what_it_shows(self):
        """092 K9: ENVIRONMENT_APPLIED answers SetEnvironment with the world's own weather; a
        world without it answers ok alone, as before."""
        shown = {}

        def set_environment(weather, time_of_day, sim_time):
            shown.update({k: v for k, v in (weather or {}).items()})
            if "fog_visual_range" in shown:          # this world shows fog in 10 m steps
                shown["fog_visual_range"] = round(shown["fog_visual_range"], -1)

        self.world.set_environment = set_environment
        self.world.applied_environment = lambda: dict(shown)
        w = pb.ScenarioWeather()
        w.fog.visual_range = 123.0
        w.sun.azimuth, w.sun.elevation = 1.0, 0.2
        self.world.capabilities = ("ENVIRONMENT",)
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        r = call(server, set_environment=pb.SetEnvironment(weather=w))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.WhichOneof("reply"), None)
        self.world.capabilities = ("ENVIRONMENT", "ENVIRONMENT_APPLIED")
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        r = call(server, set_environment=pb.SetEnvironment(weather=w))
        self.assertTrue(r.ok, r.error)
        a = r.environment.applied
        self.assertEqual(a.fog.visual_range, 120.0)
        self.assertEqual((a.sun.azimuth, a.sun.elevation), (1.0, 0.2))
        self.assertFalse(a.HasField("precipitation"))

    def test_a_world_with_the_road_read_back_answers_what_it_applies(self):
        """092 K9f-a, 086 C7: ROAD_CONDITION_APPLIED answers SetRoadCondition with the world's
        own road state, and the zones the condition named; a world without it answers ok
        alone, as before."""
        applied = {}

        def set_road_condition(friction_scale, wetness, properties):
            applied.update(scale=round(friction_scale * 20) / 20, wetness=wetness)  # 0.05 steps

        def applied_road_condition(zone_ids):
            return {"friction_scale_factor": applied["scale"], "wetness": applied["wetness"],
                    "zones": {z: {"surface_class": "gravel", "wetness": "moist",
                                  "water_depth": 0.2 if z == "ford" else None, "mu": 0.5}
                              for z in zone_ids}}
        self.world.set_road_condition = set_road_condition
        self.world.applied_road_condition = applied_road_condition
        rc = pb.RoadCondition(friction_scale_factor=0.83, wetness="moist")
        self.world.capabilities = ("ROAD_CONDITION",)
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        r = call(server, set_road_condition=pb.SetRoadCondition(road_condition=rc))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.WhichOneof("reply"), None)
        self.world.capabilities = ("ROAD_CONDITION", "ROAD_CONDITION_APPLIED", "ROAD_ZONES")
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        rc.properties["scensim.zone.ford.class"] = "gravel"
        rc.properties["scensim.zone.patch.wetness"] = "moist"
        r = call(server, set_road_condition=pb.SetRoadCondition(road_condition=rc))
        self.assertTrue(r.ok, r.error)
        a = r.road_condition
        self.assertAlmostEqual(a.friction_scale_factor, 0.85)
        self.assertEqual(a.wetness, "moist")
        self.assertEqual(sorted(a.zones), ["ford", "patch"])
        self.assertTrue(a.zones["ford"].HasField("water_depth"))
        self.assertAlmostEqual(a.zones["ford"].water_depth, 0.2)
        self.assertFalse(a.zones["patch"].HasField("water_depth"))

    def test_issue_161_out_of_range_values_are_refused_by_name_with_the_unit(self):
        """Issue 161: a value outside its field's range refuses the whole request, by field,
        value and range with their unit, word for word as the Rust shell; nothing is applied."""
        calls = []
        self.world.set_environment = lambda w, t, now: calls.append(w)
        self.world.capabilities = ("ENVIRONMENT",)
        server = WorldServer(self.world, clock=self.clock)
        hello(server)

        def weather(**parts):
            w = pb.ScenarioWeather()
            w.fog.visual_range = 500.0
            for k, v in parts.items():
                if k == "sun":
                    w.sun.azimuth, w.sun.elevation = v
                elif k == "fog":
                    w.fog.visual_range = v
                elif k == "oktas":
                    w.cloud_cover.oktas = v
                elif k == "rain":
                    w.precipitation.type = pb.PRECIPITATION["RAIN"]
                    w.precipitation.intensity = v
            return w

        for w, says in [
            (weather(sun=(0.0, 1.6)), "sun_elevation 1.6 rad is outside [-1.5708, 1.5708] rad"),
            (weather(sun=(-0.1, 0.0)), "sun_azimuth -0.1 rad is outside [0, 6.2832) rad"),
            (weather(sun=(float("nan"), 0.0)), "sun_azimuth NaN rad"),
            (weather(fog=0.0), "fog_visual_range 0 m is outside its range, above 0 m"),
            (weather(oktas=10), "cloud_cover 10 oktas is outside 0 to 9 oktas"),
            (weather(rain=-1.0),
             "precipitation_intensity -1 mm/h is outside its range, 0 or more mm/h"),
        ]:
            r = call(server, set_environment=pb.SetEnvironment(weather=w))
            self.assertFalse(r.ok)
            self.assertIn(says, r.error)
            self.assertIn("nothing changed", r.error)
        self.assertEqual(calls, [])
        for w in [weather(sun=(0.0, math.pi / 2)), weather(sun=(0.0, -math.pi / 2)),
                  weather(fog=0.1), weather(oktas=9), weather(rain=0.0)]:
            r = call(server, set_environment=pb.SetEnvironment(weather=w))
            self.assertTrue(r.ok, r.error)

    def test_issue_161_k9e_60_sent_as_radians_is_refused_saying_the_units(self):
        """Issue 161, the K9e case (133): 60 meant as degrees, sent as an elevation in radians,
        is refused, saying rad and what 60 deg would have been; 60 deg in radians is taken."""
        self.world.set_environment = lambda w, t, now: None
        self.world.capabilities = ("ENVIRONMENT",)
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        w = pb.ScenarioWeather()
        w.sun.elevation = 60.0
        r = call(server, set_environment=pb.SetEnvironment(weather=w))
        self.assertFalse(r.ok)
        self.assertIn("sun_elevation 60 rad is outside [-1.5708, 1.5708] rad "
                      "(60 deg would be 1.0472 rad)", r.error)
        w.sun.elevation = math.radians(60.0)
        self.assertTrue(call(server, set_environment=pb.SetEnvironment(weather=w)).ok)

    def test_issue_161_the_step_after_a_send_carries_the_read_back_after_its_step(self):
        """Issue 161: with READBACK_AFTER_STEP the first step after a SetEnvironment or a
        SetRoadCondition carries the read-back taken after the world's step; the next does not;
        without the bit none does."""
        shown, at_step = {}, {}
        self.world.set_environment = lambda w, t, now: at_step.update(w or {})
        self.world.applied_environment = lambda: dict(shown)
        road = {"friction_scale_factor": 1.0, "wetness": "dry", "zones": {}}
        self.world.set_road_condition = lambda f, wet, props: at_step.update(road=(f, wet))
        self.world.applied_road_condition = lambda zone_ids: dict(road)
        step_of = self.world.step

        def lagging_step(*a):
            # this world shows what it was sent only after its step (CARLA 0.9.16's weather)
            out = step_of(*a)
            r = at_step.pop("road", None)
            if r:
                road.update(friction_scale_factor=r[0], wetness=r[1])
            shown.update(at_step)
            at_step.clear()
            return out
        self.world.step = lagging_step
        self.world.capabilities = ("ENVIRONMENT", "ENVIRONMENT_APPLIED", "ROAD_CONDITION",
                                   "ROAD_CONDITION_APPLIED", "READBACK_AFTER_STEP")
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        call(server, load_world=pb.LoadWorld(world="/maps/x.usd"))
        w = pb.ScenarioWeather()
        w.fog.visual_range = 120.0
        r = call(server, set_environment=pb.SetEnvironment(weather=w))
        self.assertFalse(r.environment.applied.HasField("fog"))   # before the step
        rc = pb.RoadCondition(friction_scale_factor=0.8, wetness="moist")
        r = call(server, set_road_condition=pb.SetRoadCondition(road_condition=rc))
        self.assertAlmostEqual(r.road_condition.friction_scale_factor, 1.0)
        r = step(server)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.step.environment_applied.fog.visual_range, 120.0)
        self.assertAlmostEqual(r.step.road_condition_applied.friction_scale_factor, 0.8)
        self.assertEqual(r.step.road_condition_applied.wetness, "moist")
        r = step(server)
        self.assertFalse(r.step.HasField("environment_applied"))
        self.assertFalse(r.step.HasField("road_condition_applied"))
        self.world.capabilities = ("ENVIRONMENT", "ENVIRONMENT_APPLIED")
        server = WorldServer(self.world, clock=self.clock)
        hello(server)
        call(server, load_world=pb.LoadWorld(world="/maps/x.usd"))
        self.assertTrue(call(server, set_environment=pb.SetEnvironment(weather=w)).ok)
        self.assertFalse(step(server).step.HasField("environment_applied"))

    def test_signals_come_with_the_world_and_take_colours(self):
        hello(self.server)
        r = call(self.server, load_world=pb.LoadWorld(world="/maps/x.usd", signals=[
            pb.Signal(way_id=43763, regulatory_element_ids=[43856], opendrive_id="377",
                      position=pb.Vec3(x=102.72, y=-52.85, z=0.1)),
            pb.Signal(way_id=43766, opendrive_id="376")]))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(self.world.signals[0], {
            "way_id": 43763, "regulatory_element_ids": [43856], "opendrive_id": "377",
            "position": (102.72, -52.85, 0.1)})
        self.assertIsNone(self.world.signals[1]["position"])
        r = call(self.server, set_signals=pb.SetSignals(states=[
            pb.SignalState(opendrive_id="377", colour=pb.COLOUR["RED"]),
            pb.SignalState(opendrive_id="376", colour=pb.COLOUR["OFF"])]))
        self.assertTrue(r.ok, r.error)
        self.assertEqual(self.world.colours, {"377": "RED", "376": "OFF"})
        r = call(self.server, get_signals=pb.GetSignals(opendrive_ids=["377"]))
        self.assertTrue(r.ok, r.error)
        self.assertEqual([(s.opendrive_id, s.colour) for s in r.signals.states],
                         [("377", pb.COLOUR["RED"])])
        r = call(self.server, get_signals=pb.GetSignals())
        self.assertEqual(len(r.signals.states), 2)
        r = call(self.server, get_signals=pb.GetSignals(opendrive_ids=["999"]))
        self.assertFalse(r.ok)
        self.assertIn("999", r.error)

    def test_bad_requests_are_refused_and_the_server_goes_on(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        spawn(self.server, "npc", "VEHICLE", 0.0)
        r = spawn(self.server, "npc", "VEHICLE", 5.0)
        self.assertFalse(r.ok)
        self.assertIn("exists", r.error)
        r = call(self.server, spawn=pb.Spawn(name="flat", kind=pb.KIND["MISC_OBJECT"]))
        self.assertFalse(r.ok)
        self.assertIn("not a box", r.error)
        r = call(self.server, apply=pb.Apply(poses=[pb.EntityPose(handle=99)]))
        self.assertFalse(r.ok)
        self.assertIn("unknown handle", r.error)
        r = call(self.server, step=pb.Step(substeps=0, dt=0.05))
        self.assertFalse(r.ok)
        r = call(self.server, step=pb.Step(poses=[pb.EntityPose(handle=1)], substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("by Apply", r.error)
        self.assertFalse(pb.Reply.FromString(self.server.handle(b"\xff\xff")).ok)
        self.assertTrue(call(self.server, step=pb.Step(substeps=1, dt=0.05)).ok)

    def test_a_world_failure_is_an_error_reply(self):
        hello(self.server)

        def broken(*_args):
            raise RuntimeError("PhysX is gone")

        self.world.step = broken
        r = call(self.server, step=pb.Step(substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("PhysX is gone", r.error)

    def test_a_load_forgets_every_entity_and_the_clock_goes_on(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        spawn(self.server, "npc", "VEHICLE", 0.0)
        call(self.server, step=pb.Step(substeps=2, dt=0.025))
        r = call(self.server, load_world=pb.LoadWorld(world="/maps/x.usd",
                                                      ground_paths=["/World/env/road"]))
        self.assertEqual(r.load_world.sim_time_ns, pb.BASE_NS + 50_000_000)
        # Another world: another episode (roadmap 051); the same world again: the same one.
        self.assertEqual(r.load_world.episode_id, 2)
        self.assertEqual(self.world.loads[-1], ("/maps/x.usd", ["/World/env/road"]))
        r = call(self.server, step=pb.Step(substeps=2, dt=0.025))
        self.assertEqual(len(r.step.states), 0)
        self.assertAlmostEqual(r.step.sim_time, 1000.1)
        self.assertEqual((r.step.sim_time_ns, r.step.episode_id), (pb.BASE_NS + 100_000_000, 2))
        r = call(self.server, load_world=pb.LoadWorld(world="/maps/x.usd",
                                                      ground_paths=["/World/env/road"]))
        self.assertEqual(r.load_world.episode_id, 2)
        # Handles stay unique over the server's lifetime.
        self.assertEqual(spawn(self.server, "npc", "VEHICLE", 0.0).spawn.handle, 2)

    def test_steps_get_the_clock_at_their_start_and_are_timed(self):
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        npc = spawn(self.server, "npc", "VEHICLE", 0.0).spawn.handle
        for _ in range(3):
            r = call(self.server, step=pb.Step(substeps=2, dt=0.025))
        self.assertGreaterEqual(r.step.tick_seconds, 0.0)
        self.assertEqual([s[4] for s in self.world.steps], [0.0, 0.05, 0.1])
        self.assertEqual(len(self.server.step_ms), 3)
        # Despawning the last entity ends the scenario: the step times are summarized.
        call(self.server, despawn=pb.Despawn(handle=npc))
        self.assertEqual(self.server.step_ms, [])

    def test_shutdown_stops_the_loop(self):
        hello(self.server)
        self.assertTrue(call(self.server, shutdown=pb.Shutdown()).ok)
        self.assertTrue(self.server.stopped)


class SpeedWorld(FakeWorld):
    capabilities = ("INITIAL_SPEED", "AGENT_VEHICLES", "AUTOPILOT", "COMMANDS", "TRACKED_MOTION")

    def __init__(self):
        super().__init__()
        self.initial_speeds = []

    def set_initial_speed(self, handle, speed):
        self.initial_speeds.append((handle, speed))


def initial_speed(server, handle, speed):
    return call(server, set_initial_speed=pb.SetInitialSpeed(handle=handle, speed=speed))


class InitialSpeedTest(unittest.TestCase):
    """SetInitialSpeed (scensim issue 204), on the shell's side."""

    def setUp(self):
        self.world = SpeedWorld()
        self.server = WorldServer(self.world, clock=Clock())
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())

    def test_an_initial_speed_reaches_the_world_before_the_first_step(self):
        for i, motion in enumerate(("EXTERNAL", "AUTOPILOT", "COMMANDED")):
            r = spawn(self.server, f"e{i}", "VEHICLE", 10.0 * i, motion=motion)
            self.assertTrue(r.ok, r.error)
            r = initial_speed(self.server, r.spawn.handle, 7.5)
            self.assertTrue(r.ok, f"{motion}: {r.error}")
        self.assertEqual(self.world.initial_speeds, [(1, 7.5), (2, 7.5), (3, 7.5)])

    def test_an_initial_speed_after_the_entitys_first_step_is_refused(self):
        spawn(self.server, "ego", "EGO", 0.0, motion="EXTERNAL")
        self.assertTrue(step(self.server).ok)
        r = initial_speed(self.server, 1, 7.5)
        self.assertFalse(r.ok)
        self.assertIn("after its first Step", r.error)
        late = spawn(self.server, "late", "VEHICLE", 20.0, motion="EXTERNAL").spawn.handle
        r = initial_speed(self.server, late, 3.0)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(self.world.initial_speeds, [(late, 3.0)])

    def test_a_posed_body_takes_no_initial_speed(self):
        spawn(self.server, "k", "VEHICLE", 0.0)
        spawn(self.server, "t", "VEHICLE", 10.0, motion="TRACKED")
        for h in (1, 2):
            r = initial_speed(self.server, h, 5.0)
            self.assertFalse(r.ok)
            self.assertIn("posed", r.error)
        self.assertEqual(self.world.initial_speeds, [])

    def test_an_initial_speed_must_be_finite_non_negative_and_for_a_known_handle(self):
        spawn(self.server, "ego", "EGO", 0.0, motion="EXTERNAL")
        for v in (-1.0, float("nan"), float("inf")):
            r = initial_speed(self.server, 1, v)
            self.assertFalse(r.ok)
            self.assertIn("finite and >= 0", r.error)
        r = initial_speed(self.server, 9, 5.0)
        self.assertFalse(r.ok)
        self.assertIn("no entity with handle 9", r.error)
        self.assertEqual(self.world.initial_speeds, [])

    def test_a_load_world_forgets_which_entities_have_stepped(self):
        spawn(self.server, "ego", "EGO", 0.0, motion="EXTERNAL")
        step(self.server)
        call(self.server, load_world=pb.LoadWorld())
        h = spawn(self.server, "ego", "EGO", 0.0, motion="EXTERNAL").spawn.handle
        r = initial_speed(self.server, h, 4.0)
        self.assertTrue(r.ok, r.error)


class WeatherTest(unittest.TestCase):
    """`WEATHER` on the Python shell: a preset or parameters by the world's names (roadmap 051)."""

    class World(FakeWorld):
        capabilities = ("WEATHER",)
        PRESETS = {"Sunny": {"cloud": 0.0, "rain": 0.0}}

        def __init__(self):
            super().__init__()
            self.weather = ("", {"cloud": 50.0, "rain": 0.0})

        def set_weather(self, preset, parameters):
            if preset:
                if preset not in self.PRESETS:
                    raise ValueError(f"unknown preset {preset!r}; one of: Sunny")
                self.weather = (preset, dict(self.PRESETS[preset]))
            else:
                unknown = set(parameters) - set(self.weather[1])
                if unknown:
                    raise ValueError(f"unknown parameter(s) {sorted(unknown)}; these: cloud, rain")
                self.weather = ("", {**self.weather[1], **parameters})
            return self.weather

        def get_weather(self):
            return self.weather

    def setUp(self):
        self.server = WorldServer(self.World(), clock=Clock())
        hello(self.server)

    def test_a_preset_and_parameters_reach_the_world(self):
        r = call(self.server, set_weather=pb.SetWeather(weather=pb.Weather(preset="Sunny")))
        self.assertTrue(r.ok and r.weather.success)
        self.assertEqual(r.weather.weather.preset, "Sunny")
        r = call(self.server, set_weather=pb.SetWeather(weather=pb.Weather(
            parameters={"rain": 3.0}, use_parameters=True)))
        self.assertEqual(dict(r.weather.weather.parameters), {"cloud": 0.0, "rain": 3.0})
        r = call(self.server, get_weather=pb.GetWeather())
        self.assertEqual(dict(r.weather.weather.parameters)["rain"], 3.0)

    def test_an_unknown_name_fails_and_an_empty_request_is_refused(self):
        r = call(self.server, set_weather=pb.SetWeather(weather=pb.Weather(preset="Hail")))
        self.assertTrue(r.ok)
        self.assertFalse(r.weather.success)
        self.assertIn("unknown preset 'Hail'", r.weather.message)
        r = call(self.server, set_weather=pb.SetWeather(weather=pb.Weather()))
        self.assertFalse(r.ok)
        self.assertIn("give a preset", r.error)


class LeaseTest(unittest.TestCase):
    """One bridge drives a server at a time (roadmap 046)."""

    def setUp(self):
        self.world = FakeWorld()
        self.clock = Clock()
        self.gone = set()
        self.server = WorldServer(self.world, lease_seconds=15, clock=self.clock,
                                  is_gone=lambda c: c.pid in self.gone)
        hello(self.server)
        call(self.server, load_world=pb.LoadWorld())
        self.npc = spawn(self.server, "npc", "VEHICLE", 0.0).spawn.handle

    def test_a_second_bridge_is_refused_while_the_owner_lives(self):
        r = hello(self.server, client=OTHER)
        self.assertFalse(r.ok)
        self.assertIn("owned by testhost:100", r.error)
        r = call(self.server, client=OTHER, step=pb.Step(substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("refused: this world server is owned by testhost:100", r.error)
        # The owner is undisturbed.
        self.assertTrue(call(self.server, step=pb.Step(substeps=1, dt=0.05)).ok)
        self.assertIn(self.npc, self.world.bodies)

    def test_a_probe_changes_nothing(self):
        r = hello(self.server, client=OTHER, probe=True)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.hello.owner.pid, BRIDGE.pid)
        self.clock.now += 20
        # The probe did not refresh the owner's lease, nor take one.
        r = hello(self.server, client=OTHER, probe=True)
        self.assertEqual(r.hello.owner.pid, BRIDGE.pid)
        self.assertEqual(self.world.resets, [])

    def test_the_owners_hello_keeps_its_lease_and_its_world(self):
        for _ in range(4):
            self.clock.now += 10
            self.assertTrue(hello(self.server).ok)
        self.assertFalse(hello(self.server, client=OTHER).ok)
        self.assertIn(self.npc, self.world.bodies)
        self.assertEqual(self.world.resets, [])

    def test_a_dead_owner_is_replaced_at_once_and_its_entities_go(self):
        self.gone.add(BRIDGE.pid)
        r = hello(self.server, client=OTHER)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.hello.owner.pid, OTHER.pid)
        self.assertEqual(self.world.resets, [[self.npc]])
        self.assertEqual(self.world.bodies, {})
        # The old owner, should it wake up, is refused.
        r = call(self.server, step=pb.Step(substeps=1, dt=0.05))
        self.assertFalse(r.ok)
        self.assertIn("owned by testhost:200", r.error)

    def test_a_silent_owner_lapses(self):
        self.clock.now += 14
        self.assertFalse(hello(self.server, client=OTHER).ok)
        self.clock.now += 2
        r = hello(self.server, client=OTHER)
        self.assertTrue(r.ok, r.error)
        self.assertEqual(r.hello.owner.pid, OTHER.pid)

    def test_release_leaves_no_owner_and_no_entities(self):
        self.assertTrue(call(self.server, release=pb.Release()).ok)
        self.assertEqual(self.world.bodies, {})
        r = hello(self.server, client=OTHER, probe=True)
        self.assertFalse(r.hello.HasField("owner"))
        self.assertTrue(hello(self.server, client=OTHER).ok)


class LifetimeTest(unittest.TestCase):
    def test_this_process_is_not_gone_and_a_reused_pid_is(self):
        me = lifetime.client_id(pb)
        self.assertFalse(lifetime.is_gone(me))
        reused = pb.ClientId(host=me.host, pid=me.pid, start_time=me.start_time + 1)
        self.assertTrue(lifetime.is_gone(reused))
        self.assertTrue(lifetime.is_gone(pb.ClientId(host=me.host, pid=2**22 + 7, start_time=1)))
        # Another host is never provably gone.
        self.assertFalse(lifetime.is_gone(pb.ClientId(host="elsewhere", pid=1, start_time=1)))


class RecordingDog:
    """A watchdog that records what it was armed with."""

    def __init__(self):
        self.armed = []

    def step(self, label, deadline=None):
        dog = self

        class _Armed:
            def __enter__(self):
                dog.armed.append((label, deadline))

            def __exit__(self, *_):
                return False
        return _Armed()


# A world server in a process of its own whose `{where}` blocks: `{block}` is how (a sleep
# lets go of the GIL, a catastrophic regex holds it).
BLOCKED_SERVER = """
import logging, re, sys, time
sys.path.insert(0, {root!r})
sys.path.insert(0, {test!r})
logging.basicConfig(level=logging.INFO)
import test_server as t
from scensim_world_server import protocol as pb
from scensim_world_server.server import WorldServer
from scensim_world_server.watchdog import StepWatchdog

class W(t.FakeWorld):
    n = 0
    def load(self, *a, **k):
        if {where!r} == "load":
            {block}
        return super().load(*a, **k)
    def step(self, *a):
        W.n += 1
        if {where!r} == "step" and W.n == 2:
            {block}
        return super().step(*a)

server = WorldServer(W(), watchdog=StepWatchdog({deadline}))
server.load_deadline = {load_deadline}
assert t.hello(server).ok
assert t.step(server).ok
print("first step done", flush=True)
t.call(server, load_world=pb.LoadWorld(world="town.usd"))
t.step(server)
print("returned", flush=True)
"""


class RequestDeadlineTest(unittest.TestCase):
    """G70: a request (or the loop between requests) past its deadline ends the world server,
    from a watchdog child process, whatever the blocked thread holds."""

    def test_every_request_is_armed_and_a_load_with_its_own_deadline(self):
        dog = RecordingDog()
        server = WorldServer(FakeWorld(), watchdog=dog)
        server.load_deadline = 900.0
        hello(server)
        step(server)
        call(server, load_world=pb.LoadWorld(world="town.usd"))
        kinds = [label.split(" ")[0] for label, _ in dog.armed]
        self.assertEqual(kinds, ["hello", "apply", "step", "load_world"])
        self.assertEqual(dog.armed[2][0], "step 1 (3 x 0.0166667 s at sim time 0.000 s)")
        self.assertEqual(dog.armed[3], ("load_world town.usd", 900.0))
        self.assertTrue(all(d is None for _, d in dog.armed[:3]))  # the watchdog's own

    def test_the_child_expires_a_late_block_and_ignores_finished_ones(self):
        from scensim_world_server import watchdog
        r, w = os.pipe()
        expired = []

        def feed():
            os.write(w, b"begin 0.3 hello request (frame 0)\nend\n")
            os.write(w, b"begin 0.3 step 2 (3 x 0.0166667 s at sim time 0.050 s)\n")
            time.sleep(1.0)
            os.close(w)
        t = threading.Thread(target=feed)
        t.start()
        with os.fdopen(r, "rb", buffering=0) as stream:
            t0 = time.monotonic()
            watchdog.watch(1234, stream, expire=lambda *a: expired.append(a))
            took = time.monotonic() - t0
        t.join()
        self.assertEqual(len(expired), 1)
        parent, label, elapsed, deadline = expired[0]
        self.assertEqual((parent, label, deadline),
                         (1234, "step 2 (3 x 0.0166667 s at sim time 0.050 s)", 0.3))
        self.assertGreaterEqual(elapsed, 0.3)
        self.assertLess(took, 0.9)  # at the deadline, not at the end of the stream

    def test_the_child_returns_when_the_world_server_closes_the_pipe(self):
        from scensim_world_server import watchdog
        r, w = os.pipe()
        os.write(w, b"begin 30 step 1 (x)\n")
        os.close(w)
        with os.fdopen(r, "rb", buffering=0) as stream:
            t0 = time.monotonic()
            watchdog.watch(1234, stream, expire=lambda *a: self.fail("expired"))
        self.assertLess(time.monotonic() - t0, 2)

    def test_off_means_no_child(self):
        dog = StepWatchdog(None)
        self.assertIsNone(dog._proc)
        server = WorldServer(FakeWorld(), watchdog=dog)
        hello(server)
        self.assertTrue(step(server).ok)

    def blocked(self, where, block, deadline=0.5, load_deadline=0.5):
        here = Path(__file__).resolve()
        code = BLOCKED_SERVER.format(root=str(here.parents[1]), test=str(here.parent),
                                     where=where, block=block, deadline=deadline,
                                     load_deadline=load_deadline)
        return subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                              timeout=60)

    def test_a_blocked_load_is_killed_with_its_name_and_the_stacks(self):
        r = self.blocked("load", "time.sleep(30)")
        self.assertEqual(r.returncode, -9, r.stderr[-2000:])
        self.assertIn("first step done", r.stdout)
        self.assertNotIn("returned", r.stdout)
        self.assertRegex(r.stderr, r"load_world town\.usd has run 0\.\d s, past its deadline "
                         r"of 0\.5 s: ending world server \d+ for a fresh one")
        self.assertIn("in load", r.stderr)  # faulthandler's dump of the blocked thread

    def test_a_step_that_holds_the_gil_is_killed_all_the_same(self):
        r = self.blocked("step", "re.match(r'(a+)+$', 'a' * 40 + 'b')")
        self.assertEqual(r.returncode, -9, r.stderr[-2000:])
        self.assertRegex(r.stderr, r"step 2 \(3 x 0\.0166667 s at sim time 0\.050 s\) has run "
                         r"0\.\d s, past its deadline of 0\.5 s")

    def test_the_child_ends_the_server_at_a_fatal_line_of_a_watched_log(self):
        """csb issue 042 (011's exit on a Kit FATAL, from outside the world server): a
        watched log that does not exist yet, then ordinary lines, then a fatal one."""
        from scensim_world_server import watchdog
        d = tempfile.mkdtemp()
        path = os.path.join(d, "kit.log")
        r, w = os.pipe()
        fatal = []

        def feed():
            os.write(w, f"log\t] [Fatal] [\t{path}\n".encode())
            time.sleep(0.3)
            with open(path, "a") as f:
                f.write("[1ms] [Warning] [omni.physx.plugin] a warning\n")
                f.flush()
                time.sleep(0.6)
                f.write("[2ms] [Fatal] [omni.rtx] Out of resource descriptors!\n")
            time.sleep(3.0)
            os.close(w)
        t = threading.Thread(target=feed)
        t.start()
        with os.fdopen(r, "rb", buffering=0) as stream:
            t0 = time.monotonic()
            watchdog.watch(1234, stream, expire=lambda *a: self.fail("expired"),
                           fatal=lambda *a: fatal.append(a))
            took = time.monotonic() - t0
        t.join()
        shutil.rmtree(d)
        self.assertEqual(fatal, [(1234, path, "[2ms] [Fatal] [omni.rtx] Out of resource "
                                              "descriptors!")])
        self.assertLess(took, 2.5)  # at the line, not at the end of the stream

    def test_a_watched_log_without_a_fatal_line_ends_nothing(self):
        from scensim_world_server import watchdog
        d = tempfile.mkdtemp()
        path = os.path.join(d, "kit.log")
        with open(path, "w") as f:
            f.write("[1ms] [Error] [carb] not fatal\n[2ms] [Fatal] [x] no newline yet")
        r, w = os.pipe()
        os.write(w, f"log\t] [Fatal] [\t{path}\n".encode())

        def close():
            time.sleep(1.2)
            os.close(w)
        t = threading.Thread(target=close)
        t.start()
        with os.fdopen(r, "rb", buffering=0) as stream:
            watchdog.watch(1234, stream, expire=lambda *a: self.fail("expired"),
                           fatal=lambda *a: self.fail(f"fatal {a}"))
        t.join()
        shutil.rmtree(d)

    def test_a_fatal_in_the_log_kills_a_server_that_holds_the_gil(self):
        """042: the world server's main thread holds the GIL; a fatal line in its log still
        ends it, because the child, not the world server, reads the log."""
        d = tempfile.mkdtemp()
        path = os.path.join(d, "kit.log")
        here = Path(__file__).resolve()
        code = FATAL_SERVER.format(root=str(here.parents[1]), path=path)
        r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                           timeout=60)
        shutil.rmtree(d)
        self.assertEqual(r.returncode, -9, r.stderr[-2000:])
        self.assertIn("holding the GIL", r.stdout)
        self.assertRegex(r.stderr, r"fatal in .*kit\.log: \[3ms\] \[Fatal\] \[omni\.rtx\] gone: "
                                   r"ending world server \d+ for a fresh one")

    def test_a_server_that_ends_takes_its_watchdog_with_it(self):
        dog = StepWatchdog(5.0)
        child = dog._proc
        self.assertIsNone(child.poll())
        dog.close()
        self.assertIsNotNone(child.poll())


# A world server whose log gets a fatal line (written by another process, as Kit's C logger
# writes its file without the GIL) while its main thread holds the GIL (csb issue 042).
FATAL_SERVER = """
import re, subprocess, sys, time
sys.path.insert(0, {root!r})
from scensim_world_server.watchdog import StepWatchdog
dog = StepWatchdog(60.0)
dog.watch_log({path!r}, "] [Fatal] [")
# Kit's C logger writes its file without the GIL: here a process of its own does
subprocess.Popen(["sh", "-c", "sleep 0.5; echo '[3ms] [Fatal] [omni.rtx] gone' >> " + {path!r}])
with dog.step("Kit's start and the preload", deadline=60.0):
    print("holding the GIL", flush=True)
    re.match(r'(a+)+$', 'a' * 40 + 'b')
print("returned", flush=True)
"""


# A world server that stops on SIGTERM as a launcher stops its group: `{stop}` is how long
# its stop takes, and `{armed}` whether the stop runs under a deadline of 1 s.
STOPPING_SERVER = """
import logging, signal, sys, time
sys.path.insert(0, {root!r})
logging.basicConfig(level=logging.INFO)
from scensim_world_server.watchdog import StepWatchdog
dog = StepWatchdog(60.0)
print("child", dog._proc.pid, flush=True)
stopping = []
signal.signal(signal.SIGTERM, lambda *_: stopping.append(1))
while not stopping:
    with dog.step("idle"):
        time.sleep(0.05)
if {armed}:
    with dog.step("the stop", deadline=1.0):
        time.sleep({stop})
else:
    with dog.step("the stop"):  # writes to the child after the group's SIGTERM
        time.sleep({stop})
dog.close()
print("stopped", flush=True)
"""


class StopTest(unittest.TestCase):
    """G70 follow-up: the group's stop signal is the world server's; its watchdog child
    outlives it, so no "the watchdog process is gone" at a stop, and keeps its deadline."""

    def stop(self, stop, armed):
        root = str(Path(__file__).resolve().parents[1])
        code = STOPPING_SERVER.format(root=root, stop=stop, armed=armed)
        p = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, start_new_session=True)
        child = int(p.stdout.readline().split()[1])
        time.sleep(0.5)
        os.killpg(p.pid, signal.SIGTERM)  # as play_launch or ros2 launch stops a group
        time.sleep(0.2)
        alive_after_term = os.path.exists(f"/proc/{child}")
        out, err = p.communicate(timeout=30)
        time.sleep(0.3)
        return p.returncode, out, err, alive_after_term, os.path.exists(f"/proc/{child}")

    def test_the_child_outlives_the_stop_and_ends_with_the_parent(self):
        rc, out, err, alive_after_term, alive_at_end = self.stop(1.0, armed=False)
        self.assertEqual(rc, 0, err[-2000:])
        self.assertIn("stopped", out)
        self.assertTrue(alive_after_term)   # the SIGTERM did not end it
        self.assertFalse(alive_at_end)      # the closed pipe did
        self.assertNotIn("watchdog process is gone", err)

    def test_a_stop_past_its_deadline_is_killed_all_the_same(self):
        rc, out, err, _, alive_at_end = self.stop(30.0, armed=True)
        self.assertEqual(rc, -9, err[-2000:])
        self.assertRegex(err, r"the stop has run 1\.\d s, past its deadline of 1 s")
        self.assertFalse(alive_at_end)


class DescriptorTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("protoc"), "no protoc")
    def test_the_descriptor_set_is_the_protos(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "scensim_world.desc"
            subprocess.run(["protoc", f"--proto_path={root / 'proto'}",
                            f"--descriptor_set_out={out}", str(root / "proto/scensim_world.proto")],
                           check=True)
            self.assertEqual(out.read_bytes(), pb.DESCRIPTOR_SET.read_bytes(),
                             "scensim_world.desc is stale: run "
                             "src/scensim_world_server/scripts/gen_world_proto.sh")


if __name__ == "__main__":
    unittest.main()
