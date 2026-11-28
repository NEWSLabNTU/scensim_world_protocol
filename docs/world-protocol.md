# The world protocol

How the scenario bridge (scensim's `RemoteBackend`) drives a simulator's
**world server**. The bridge knows no simulator: everything it asks of one goes over this
protocol, and a simulator becomes a backend by serving it. The definition is
[`proto/scensim_world.proto`](../proto/scensim_world.proto) (package `scensim_world`); this page says
what the messages mean together. How to write a server, and the conformance check that
says one is done: [world-server.md](world-server.md).

## Transport

- ZMQ, TCP. The server binds a `REP` socket; the bridge connects a `REQ` socket (relaxed and
  correlated, so after a timeout it can send again and a late reply to the old request is
  dropped). The servers of this project bind `tcp://127.0.0.1:<port>`.
- One `Request` in, one `Reply` out, strictly alternating. Nothing is pushed: the server only
  ever answers.
- Every `Request` carries one request in its `oneof` and the sender's `ClientId`
  (`client`, field 100), on `Hello` too.
- Every `Reply` has `ok`. `ok: false` is a refusal: `error` says why, in words the bridge
  passes on to SSv2. `fatal: true` means the server cannot go on (a hung simulator call, its
  simulator gone) and exits after this reply; the bridge ends as after a request that timed
  out. A refusal is not fatal unless it says so: the server keeps serving.
- An undecodable request, or one with no request set, is refused (`ok: false`), not dropped.

`PROTOCOL_VERSION` is **4**. It is defined in three places that must agree:
`scensim_core::world_client::PROTOCOL_VERSION` (used by the bridge, the Rust shell and
`scensim_world_check`) and `scensim_world_server/protocol.py`'s `PROTOCOL_VERSION`. Bump it on any
change a peer of the other version would misread. A field added for a capability the bridge
only uses when the server declares it needs no bump.

## Frames and units

- The world frame is right-handed and Z-up, in metres and radians (ROS's). A simulator with
  another convention (CARLA: left-handed) converts inside its server.
- The bridge subtracts the map origin (`<map dir>/<simulator>/world.yaml`) before it sends
  a pose. The server therefore works in world coordinates. It sees map coordinates only when
  it reads map files itself, and `LoadWorld.map_origin` tells it where the world origin sits
  in the map frame.
- Poses (`Pose`: `position`, `orientation` as a quaternion x, y, z, w) are at the **SSv2
  entity origin**. For a vehicle that is the rear axle centre on the ground. The declared box
  sits around that origin (`Spawn.box_center` is the box centre from it).
- Orientation is ROS REP-103 (ENU: x forward, y left, z up, right-handed), in both
  directions and in every message: roll about x, pitch about y, yaw about z, so a positive
  pitch puts the nose **down** and a positive roll the right side down; a car facing up a
  10 deg slope has pitch -10 deg. A server whose simulator keeps another convention
  converts at its own edge (CARLA/UE: pitch positive nose up, so roll stays, pitch and yaw
  negate; `scensim_carla_world`); Isaac's stage is ENU and takes the quaternion as it is. The
  conformance check tests it ("a pose's orientation reads back as posed" and, on a map
  slope, "the ego settles with the road's pitch sign").
- `Pose.rpy`, when set, is roll, pitch and yaw in radians, authoritative over the quaternion.
  A server that holds orientation as Euler angles sets it so a reader does not round-trip
  through the quaternion.
- A `Twist` is world frame at the entity origin unless the field says otherwise
  (`EntityPose.ssv2_twist` and `ReadBack.twist` are in the entity's own frame, x forward).

## Ownership: one bridge at a time

A server is driven by one client at a time, its **owner**.

- A `Hello` with `probe: true` is a health check. It is answered, and nothing changes: no
  ownership taken, no lease refreshed. `scensim_world_check hello` and the bridge's idle checks
  are probes.
- A `Hello` with `probe: false`:
  - with no owner, makes the sender the owner, and every entity the server still has is
    removed (the world stays loaded);
  - from the owner, refreshes its lease;
  - from anyone else, takes the server over only when the owner is **provably gone** or has
    been **silent longer than the lease** (15 s; `LEASE` in the Rust shell, `LEASE_SECONDS`
    in the Python shell). Otherwise it is refused with `owned by <host>:<pid> ...`.
- "Provably gone": the owner's `host` is this host, and its `pid` either does not run or
  runs with a start time other than `ClientId.start_time` (`/proc/<pid>/stat` field 22,
  clock ticks since boot; 0 means unknown and is never taken as a mismatch). A client on
  another host is never provably gone; only its lease lapses.
- Every request other than `Hello` must come from the owner. With no owner it is refused
  (`<request> before hello: say hello first`); from another client, refused
  (`<request> from <client> refused: this world server is owned by <owner>`). Each request
  from the owner refreshes its lease.
- `Release`: the owner leaves. The server undoes what it did for it (entities destroyed, the
  simulator's settings restored) and has no owner after. The world stays loaded.
- `HelloReply.owner` is the owner after this `Hello` (unset: none). A client knows it owns
  the server when that equals its own `ClientId`, field for field.

## Capabilities

The **core** is what every server does: `Hello`, `LoadWorld`, `Spawn` of `KINEMATIC` bodies
and of the ego as `EXTERNAL`, `Apply`, `Step`, `Despawn`, `SetStepTime`, `Release`,
`Shutdown`. Everything else is declared in `HelloReply.capabilities`. The bridge never sends a
request or a drive a server did not declare. It refuses SSv2's request instead, naming the
missing capability. A server refuses a request for a capability it did not declare (both
shells do this for you).

| capability | the server does | requests and fields |
|---|---|---|
| `SIGNALS` | holds traffic lamps at the colours it is told | `LoadWorld.signals`, `SetSignals` |
| `AUTOPILOT` | its own traffic drives a vehicle toward a goal | `Motion.AUTOPILOT`, `SetGoal` |
| `AGENT_VEHICLES` | spawns `EXTERNAL` vehicles other than the ego, driven by another client through the simulator | `Spawn` with `motion: EXTERNAL`, `kind` not `EGO` |
| `TRACKED_MOTION` | moves `TRACKED` bodies by the velocity the bridge computes | `EntityPose.twist`, `teleport`, `kinematic` |
| `TRACKS_TARGETS` | tracks `TRACKED` bodies itself from SSv2's pose and twist, and reports each one's tracking | `EntityPose.pose`, `ssv2_twist`; `ApplyReply.tracking` |
| `READS_BACK` | reads back what it moves (the ego, `EXTERNAL`, `AUTOPILOT` bodies) as SSv2 wants them | `ApplyReply.read_back` |
| `WEATHER` | a weather by preset or by named parameters | `SetWeather`, `GetWeather` |
| `CONTACTS` | reports contacts of reporting bodies | `StepReply.contacts`, `ApplyReply.contacts` |
| `ROAD_CONDITION` | OpenSCENARIO's road condition, world-wide and per zone | `SetRoadCondition` |
| `ENVIRONMENT` | OpenSCENARIO's weather and time of day | `SetEnvironment` |
| `ENVIRONMENT_APPLIED` | reads the weather back after each `SetEnvironment` | `EnvironmentReply` |
| `ROAD_ZONES` | applies `scensim.zone.<id>.<field>` to its map's surface record and lists every named zone in the reply; needs `ROAD_CONDITION_APPLIED` | `RoadConditionReply.zones` |
| `ROAD_CONDITION_APPLIED` | reads the road state back after each `SetRoadCondition`; needs `ROAD_CONDITION` | `RoadConditionReply` |
| `READBACK_AFTER_STEP` | reads the weather or road state back again after the first step that follows a send, the simulator's tick done (scensim issue 161); needs `ENVIRONMENT_APPLIED` or `ROAD_CONDITION_APPLIED` | `StepReply.environment_applied`, `StepReply.road_condition_applied` |
| `COMMANDS` | moves `COMMANDED` bodies by a behaviour model's kinematic commands, which reach it with its tick gate's acknowledgements, and reads each back with the frame its command answered; needs `READS_BACK` | `Motion.COMMANDED`; `ReadBack.cmd_frame`, `EntityState.cmd_frame`, `StepReply.command_counts` |
| `INITIAL_SPEED` | starts a body it moves (`EXTERNAL`, `AUTOPILOT`, `COMMANDED`) at the speed `SetInitialSpeed` gives, along its spawn heading, from its start (the first step after its spawn once SSv2's NPC logic has started), a vehicle's wheels spinning to match (scensim issue 204) | `SetInitialSpeed` |

A `Spawn` with `motion: TRACKED` needs `TRACKED_MOTION` or `TRACKS_TARGETS`; `AUTOPILOT`
needs `AUTOPILOT`; `EXTERNAL` for a kind other than `EGO` needs `AGENT_VEHICLES`; `COMMANDED`
needs `COMMANDS`. The bridge spawns a `COMMANDED` body only on a server declaring `COMMANDS`
and `READS_BACK`, so the motion needed no protocol version: an older server never sees it.

## The requests

### Hello -> HelloReply

`Hello.protocol_version` must equal the server's, or the request is refused with both
versions in the message. The reply:

| field | |
|---|---|
| `protocol_version` | the server's (the client refuses a reply in another version) |
| `simulator_version` | for the log |
| `simulator_name` | `carla`, `isaac`, ...: the map directory's `<name>/world.yaml` is this server's |
| `capabilities` | as above |
| `owner` | the owner after this `Hello`, unset when none |
| `instance` | unique per server process. A bridge that sees it change knows the server restarted and every entity it had is gone |
| `max_physics_dt` | the longest physics step the server takes, s. The bridge splits each SSv2 step into `n = ceil(step / max_physics_dt)` substeps of `step / n`. 0: the server takes SSv2's step whole |
| `ready`, `not_ready_reason` | whether the server can load a world now (CARLA: connected to its server) |
| `boot_id` | the server host's boot id (`/proc/sys/kernel/random/boot_id`): `StepReply`'s `CLOCK_MONOTONIC` times compare only with times taken under the same one (scensim issue 135); empty where the server does not say |

### LoadWorld -> LoadWorldReply

SSv2's `Initialize`. It replaces the world: every entity is gone after it. A server that is
not `ready` refuses it.

| field | |
|---|---|
| `world` | the world as the server's `world.yaml` names it (a CARLA town, a USD stage). Empty: the server's default for the map |
| `ground_paths` | the environment a body rests on; contacts with it are not reported |
| `signals` | the map's traffic signals (only to a `SIGNALS` server); every lamp green until `SetSignals` |
| `map_dir`, `lanelet2_map_path`, `map_origin` | SSv2's map, for a server that reads map files |
| `step_time`, `realtime_factor` | SSv2's `Initialize` |

The reply carries the time the first step starts from (`sim_time_ns`) and the episode the
load began (`episode_id`); see "Time and episodes".

### Spawn -> SpawnReply

One entity: a body at `pose` with a box collider, moving as `motion` says.

| field | |
|---|---|
| `name` | unique among the server's entities: a second of one name is refused |
| `kind` | `EGO`, `VEHICLE`, `PEDESTRIAN`, `MISC_OBJECT` |
| `subtype` | SSv2's (`car`, `truck`, ...), for the look; may be empty |
| `box_center`, `box_dimensions`, `box_declared` | the declared box (centre from the entity origin; length, width, height). The Python shell refuses dimensions that are not all positive |
| `pose` | world frame, entity origin |
| `model`, `asset_key` | the look: SSv2's asset key after and before the bridge's asset map. A CARLA blueprint id (`vehicle.tesla.model3`) is what both servers of this project know. Empty or unknown: the server's default for the kind |
| `motion` | `KINEMATIC` (posed every step, pushed by nothing), `TRACKED`, `EXTERNAL` (the simulator's physics moves it and something else drives it: read back, never posed), `AUTOPILOT`, `COMMANDED` (the server integrates a behaviour model's commands: read back, never posed) |
| `role_name` | the name the simulator knows the vehicle by, for whoever attaches to it (the ego's role name) |
| `axles`, `performance` | as SSv2 declared them |

`SpawnReply.handle` is nonzero and unique for the server's lifetime (both shells assign it).
`SpawnReply.body` is the body the server chose:

| field | |
|---|---|
| `sim_id` | the simulator's id for it (CARLA's actor id); 0 when there is none |
| `model` | what the body is: a blueprint id, `isaac:box`, ... |
| `box_location`, `box_extent`, `has_box` | the model's own box: centre and **half** extents in the body's frame |
| `origin_offset`, `origin_yaw` | from the entity origin to the body's origin (entity frame), and the body's turn from the entity's heading |
| `declared_center`, `declared_dimensions` | the declared box as the server took it |
| `wheels` | a vehicle's wheels as the server knows them, entity frame: `front_x`, `rear_x` (axle x from the entity origin; 0 when the rear axle is the origin, as SSv2's is), `track`, `radius`. Unset when unknown |
| `spec_hash` | the vehicle spec (sim_vehicle_spec's `spec_hash`) the server built the body from: the rig or the physics it applied. Empty when none |

What the bridge does with the last two, for the ego (roadmap 051):

- **The spawn check**: it compares the body with what SSv2 declared (the vehicle catalog's
  car) and refuses the ego when a field is more than 2 cm off, naming each field with both
  values. The box is compared when the server reported one, the wheels when both sides know
  them, and the box's y by size only.
- **The start check**: it compares `spec_hash` with the vehicle agent's and with the
  installed catalog entry's (`Property name="scensim.spec_hash"`) and refuses the ego when two
  differ, naming who loaded which. A part that reports none is left out.

### Despawn

Removes one entity. An unknown handle is refused.

### Apply -> ApplyReply

SSv2's `UpdateEntityStatus`: every entity's drive for the coming step, as `EntityPose`s.
Handles the server does not know are refused (the whole request). The drive of each motion:

- `KINEMATIC`: goes to `pose`.
- `TRACKED` under `TRACKED_MOTION`: takes `twist` before the substeps, its pose ignored
  unless `teleport` (then first put at `pose`); with `kinematic` it becomes kinematic from
  this step on (the bridge's fallback for a blocked body) and goes to `pose`.
- `TRACKED` under `TRACKS_TARGETS`: given SSv2's `pose` and `ssv2_twist`; the server
  decides how to follow.
- `EXTERNAL`, `AUTOPILOT` and `COMMANDED`: listed with nothing set, so the server knows SSv2 still has them
  this frame.

The reply: `read_back` (`READS_BACK`: pose, twist, acceleration and jerk of each body the
server moves; a `COMMANDED` body's also carries `cmd_frame`), `tracking` (`TRACKS_TARGETS`: one `TrackingSample` per tracked body),
`failure` (what failed, for SSv2's reply; empty when nothing did) and `contacts` learned
since the last reply (a simulator that reports them asynchronously, belonging to the last
step). The drives take effect at the next `Step`.

### Step -> StepReply

SSv2's `UpdateFrame`: advance the world by `substeps` physics steps of `dt` seconds. Both
shells refuse a `Step` that carries `poses` (drives come by `Apply` since protocol 4); the
Python shell also refuses `substeps < 1` or `dt <= 0`. `render`: whether to render this step. `want_states`: the reply
must carry every entity's state; a server that has them anyway may always send them.

| reply field | |
|---|---|
| `sim_time_ns` | simulation time after the step, ns, `nanos(elapsed) + BASE_NS`: required (a server that leaves it 0 fails the conformance check) |
| `sim_time` | the same time in seconds, `sim_time_ns / 1e9` (`BASE_NS` included), for the log and the bridge's frame interval; use `sim_time_ns` for time |
| `frame` | the server's frame counter after the step |
| `episode_id` | the episode the step is in (`LoadWorldReply`'s); 0 from a server that does not say |
| `states` | every entity's `EntityState`: pose after the step, world frame, at the **body's** origin (`Body.origin_offset` from the entity origin); `twist`, the body's velocity, where the server has it; `cmd_frame`, a `COMMANDED` body's, as its read-back; `applied_control`, a physics vehicle's actuator control (throttle, brake, steer, flags, gear; on CARLA under the automatic transmission the gear engaged, not the one sent) as the simulator held it in the step, raw, where the server has it (CARLA: the ego and every tracked vehicle, from the step's snapshot; Isaac leaves it unset) |
| `contacts` | `CONTACTS`: a reporting body (the ego, a tracked body) touching another at any substep, once per pair and reporting side. `other` is the other entity's handle, 0 for the environment or a body the bridge did not spawn; `impulse` the total normal impulse over the step, N s |
| `tick_seconds` | the time the server spent stepping the simulator, s, for the bridge's frame budget |
| `command_counts` | `COMMANDS`: since the world's last load, the commands naming an entity that is not a `COMMANDED` body (each entity and frame once), the frames the tick gate ticked on without an acknowledgement, the datagrams dropped whole, and the commands dropped as stale (answering a frame at or before a floor: the frame the entity's body was spawned or destroyed at, or the world's last step before a reset; each entity and frame once); unset elsewhere |
| `tick_start_mono_ns`, `tick_return_mono_ns` | `CLOCK_MONOTONIC`, ns, on the server just before its call to the simulator's tick that produced the reply's frame and when it returned (scensim issue 135); the start precedes the simulator's own by up to the call's send latency; comparable only within the boot `HelloReply.boot_id` names; 0 where the server does not say (Isaac) |

### SetStepTime

SSv2's `UpdateStepTime`. Core: a server may ignore it (the Python shell does), since every
`Step` names its `dt`.

### SetSignals (`SIGNALS`)

Colours (`OFF`, `RED`, `AMBER`, `GREEN`) by OpenDRIVE signal id, from the next step on.
Lamps not named keep theirs.

### SetGoal (`AUTOPILOT`)

SSv2's `UpdateEntityGoal` for an `AUTOPILOT` body: waypoints (the last is the goal),
`clear`, and an optional target speed.

### SetInitialSpeed (`INITIAL_SPEED`)

An entity's start speed, m/s along its spawn heading, finite and >= 0 (scensim issue 204).
OpenSCENARIO gives it by an Init `SpeedAction`, which SSv2 runs after the spawn, so it cannot
ride in `Spawn`.

**When it applies.** At the entity's start: the first `Step` after its spawn once SSv2's NPC
logic has started (`Apply.npc_logic_started`). SSv2 steps the world before that: the
interpreter's first update runs before the Init actions, and the frames go on while it waits
for an engagement. Through those steps the body stays at rest at its spawn pose. On the start
step's reply it is at the speed; with no command it coasts on from there. For an entity
spawned after the start, the start is its first `Step`.

**What is refused.** One sent after the entity's start, and one for a body the bridge poses
(`KINEMATIC`, `TRACKED`: their speed comes from their poses). Sent again before the start, the
last one holds.

The additive request needed no protocol version: the bridge sends it only to a server declaring
`INITIAL_SPEED`, and refuses a nonzero start speed on another one, naming issue 204.

### SetWeather, GetWeather -> WeatherReply (`WEATHER`)

A generic weather: the names belong to the server. The CARLA world server takes CARLA's
presets and the fields of CARLA's `WeatherParameters`; another server defines its own.

- `Weather`: `preset` (a name), or `parameters` (name to value) with `use_parameters: true`.
- `SetWeather` with a preset applies it. With parameters it changes **only the parameters
  named**; the others keep their current values.
- `GetWeather` returns the weather in effect: `WeatherReply.success: true`, and in
  `weather` every parameter the server has, by its own name (this is how a client learns
  the names), and the preset it is, or empty when the parameters match none.
- `WeatherReply.success: false` with a `message` is a weather the server could not apply;
  `ok: false` is a request it refused.
- A world load resets the weather to the new world's own. The CARLA world server holds the
  weather last asked for and applies it again after every load.

The bridge exposes this as ROS services (`/scenario_world_bridge/set_weather`,
`get_weather`) and passes requests through.

### SetRoadCondition (`ROAD_CONDITION`)

SSv2's `UpdateEnvironment` road condition: the whole road state from now on, replacing the
last one. `friction_scale_factor` scales every surface's friction; `wetness` is one of
OpenSCENARIO's levels (empty: dry); `properties` holds only `scensim.` properties, where
`scensim.zone.<id>.<field>` sets a field of a zone of the map's `surfaces.yaml`. A server refuses
a property it does not know or cannot apply, listing every problem (a zone it does not have
by its id), and then changes nothing. `LoadWorld` and `Release` go back to the default.

A server with `ROAD_CONDITION_APPLIED` answers with `RoadConditionReply { friction_scale_factor,
wetness, zones }`: the road state it applies now, read back from its own model, for the
world-wide part; with `ROAD_ZONES` also every zone the condition named (`ZoneApplied`: its class,
wetness, water depth when it has one, and mu, the class's mu times the wetness factor times the
scale), empty otherwise (scensim roadmap 092 K9f-a, 086 C7). A zone the condition did not name is
as the map has it and is not listed. CARLA 0.9.16 offers the read-back without zones (086 C7:
its friction trigger reaches every car of a blueprint, and a per-wheel physics apply steps the
car); Isaac offers both.

### SetEnvironment (`ENVIRONMENT`)

SSv2's `UpdateEnvironment` weather and time of day. Every part (`cloud_cover`, `sun`, `fog`,
`precipitation`, `wind`, `time_of_day`) is optional: a part not set is left as it was. A
server refuses, listing every problem, a part it cannot show, and then changes nothing.
`LoadWorld` and `Release` go back to the world's own sky.

Every server refuses a value outside its field's range the same way, by the shell before the
world sees it (scensim issue 161): the sun's azimuth [0, 2 pi) rad and elevation [-pi/2, pi/2]
rad, cloud cover 0 to 9 oktas, a fog's visual range above 0 m, precipitation 0 or more mm/h. The
message names the field, the value and the range with their unit, and for an angle what the
number would be as degrees: `sun_elevation 60 rad is outside [-1.5708, 1.5708] rad (60 deg would
be 1.0472 rad)`.

A server with `ENVIRONMENT_APPLIED` answers with `EnvironmentReply { applied }`: the weather it
shows now, read back from its simulator in `ScenarioWeather`'s units, for every part the
scenario has set (a part never given is unset). A knob profile's record reads it (scensim
roadmap 092 K9): the bridge sends a profile's value as `SetEnvironment` when it moves and
writes the read-back beside it in the gap trace. CARLA reads back fog, cloud cover, rain and the
sun (not wind: CARLA keeps no direction for it); Isaac reads back fog, rain and the sun from its
renderer and the sun light (not clouds or wind, of which it keeps no state of its own).

### The read-back after the step (`READBACK_AFTER_STEP`)

A read-back taken right after the send may predate it: CARLA 0.9.16's client sends the weather
without waiting, and a read sent next can be served first (scensim issue 161, 171). A server with
`READBACK_AFTER_STEP` reads it back again after the first step that follows a `SetEnvironment`
or a `SetRoadCondition`, once the simulator's tick has returned, and sends it in that step's
reply: `StepReply.environment_applied` (`EnvironmentReply.applied`'s form) and
`StepReply.road_condition_applied` (`RoadConditionReply`'s form). Every other step leaves them
unset. A server declaring the capability fills the field on that step without fail; an owner
treats an empty one as a protocol error. The shells do this for every server that declares it.

### Release, Shutdown

`Release`: see "Ownership". `Shutdown`: the server stops serving after this reply.

## Time and episodes

Simulation time is one integer, the same in every process that holds the frame (roadmap
051):

```text
sim_time_ns = nanos(elapsed_seconds) + BASE_NS
BASE_NS     = 1_000_000_000_000          (1000 s)
nanos(s)    = round(s * 1e9), half away from zero
```

`elapsed_seconds` is the simulator's time into its episode. `BASE_NS` keeps ROS time far from
0, which ROS reads as "no clock yet", and from the small values for which `now() - duration`
goes negative. The rounding is Rust's `f64::round`, not Python's `round()` (half to even,
1 ns apart on about 1.4 % of CARLA frames). Implementations: `scensim_core::sim_time` (`BASE_NS`,
`nanos`, `sim_ns`) and `scensim_world_server.protocol` (`BASE_NS`, `nanos`, `sim_ns`).
Nothing about time is shared, stored or sent beyond the replies: any process computes it from
the frame it holds.

An **episode** is one world: from its load until the next load of another world, or the
server's death.

- `LoadWorldReply.episode_id` is nonzero, and differs from the last one whenever the world
  was loaded anew. The CARLA world server's is CARLA's world id. The Python shell counts
  loads of a different world (`LoadWorld.world` changed): the same world loaded again is the
  same episode.
- `StepReply.episode_id` repeats the episode the step is in.
- `sim_time_ns` is monotonic **within** an episode: every step's is greater than the last,
  and the same world loaded again keeps it going on. Across episodes it may start again. A
  server may keep it running across episodes (the Isaac world server and the Python shell
  do), but nothing may depend on that.
- `LoadWorldReply.sim_time_ns` is the time the first step starts from, and is at least
  `BASE_NS`.
- When the episode changes, the bridge logs it. Every vehicle stack of the old episode ends
  (its vehicle bridge exits before publishing a frame of the new one) and is restarted on the
  new one by whatever supervises it.

## SSv2's message types

`proto/` also holds SSv2's `simulation_interface` protos (`simulation_api_schema`,
`geometry_msgs`, `traffic_simulator_msgs`, `builtin_interfaces`, `autoware_control_msgs`,
`autoware_vehicle_msgs`, `rosgraph_msgs`, `std_msgs`). They are not part of the world
protocol on the wire. `scensim_core`'s `SimBackend` seam and its shared world policy are typed in
them. They must be byte-identical to the SSv2 fork's (`scenario_simulator_v2`,
`simulation/simulation_interface/proto/`): a field added to one copy and not the other
decodes as silence, not as an error. scensim's `just test` checks this
(`scripts/test_ssv2_protos.py`), since that repository holds both.
