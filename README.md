# scensim_world_protocol

The world protocol of [scensim](https://github.com/NEWSLabNTU/scensim):
how the scenario bridge drives a simulator's **world server**, whatever the simulator. The
bridge knows no simulator; a simulator becomes a backend by serving this protocol and passing
`scensim_world_check conformance`.

| Path | What |
|---|---|
| `proto/scensim_world.proto` | The protocol (package `scensim_world`, version 4): `Hello` and the owner lease, capabilities, `LoadWorld`, `Spawn`, `Apply`, `Step`, signals, goals, weather, road condition, environment, `Release` |
| `proto/*.proto` (the rest) | SSv2's `simulation_interface` message types, which `scensim_core`'s shared world code is typed in. Byte-identical to the SSv2 fork's; scensim's `just test` checks it |
| `scensim_core/` | Rust: the world client (`world_client`), the server shell (`world_server`: the `World` trait, `Shell`, `serve`, the lease), simulation time (`sim_time`: `BASE_NS`, `nanos`, `sim_ns`), the `SimBackend` seam, and the world policy every Rust world server shares (entities, NPC tracking, gap trace, contacts, weather requests, maps) |
| `scensim_world_server/` | Python: the server shell (`World` base class, `WorldServer`, `bind`, `run`, the lease, the clock and the episode, the request deadlines), on a descriptor set of the proto (`scensim_world.desc`). A request, or the loop between requests, that runs past its deadline (`run(step_deadline=180, load_deadline=900)`, s) is ended by a watchdog child process: its stacks dumped (py-spy when installed, faulthandler), then killed for its supervisor to start a fresh one (`watchdog.py`). A world server can start the watchdog itself before its simulator and pass it to `run(watchdog=)`, to put its own start under a deadline, and can have the child watch a log file for fatal lines (`StepWatchdog.watch_log`, Kit's log on Isaac), so that nothing of the kind runs in the world server's process (csb issue 042) | It also carries the ground's surface class table every world server shares (`surface_classes.yaml`, `classes.SURFACE_CLASSES`; installed to `share/scensim_world_server/config/` too; scensim roadmap 086 C2).
| `scensim_world_check/` | `scensim_world_check hello / conformance / rtt <endpoint>`: probe, conformance-check or time any world server |

Docs:

- [docs/world-protocol.md](docs/world-protocol.md): the protocol, request by request; the
  owner lease; capabilities; time and episodes; the spawn reply's `Body` (wheels and
  `spec_hash`); the generic weather.
- [docs/world-server.md](docs/world-server.md): writing a world server on the Python or the
  Rust shell (with a complete Python example), and what the conformance check checks.

## Time, in one paragraph

Simulation time is `sim_time_ns = nanos(elapsed) + BASE_NS`, with `BASE_NS` = 1000 s and
`nanos` rounding half away from zero (`scensim_core::sim_time`, `scensim_world_server.protocol`). An
episode is one world: `LoadWorldReply.episode_id` and `StepReply.episode_id` name it, and the
time is monotonic within an episode only. An episode change ends every vehicle stack, which
comes back on the new one (roadmap 051).

## Build and test

No ROS is needed. The Rust crates need `protoc` (prost-build) and libzmq; the Python shell
needs `pyzmq` and `protobuf`.

```bash
just build       # cargo, profile dev-release: scensim_core, scensim_world_check
just test        # cargo nextest, and the Python shell's unit tests
just check       # cargo fmt --check, clippy -D warnings
just proto-desc  # regenerate scensim_world_server/scensim_world.desc after editing the proto
```

`scensim_world_check` is also an ament_cargo package, so a colcon workspace that holds this
repository installs it.

The CARLA and Isaac Sim world servers are
[scensim_carla_world](https://github.com/NEWSLabNTU/scensim_carla_world) (Rust shell) and
[scensim_isaac_world](https://github.com/NEWSLabNTU/scensim_isaac_world) (Python shell).
