//! The Rust shell on a fake world: the lease, handles, capabilities, the fatal end.

use super::*;

#[derive(Default)]
struct Fake {
    spawned: Vec<u32>,
    resets: Vec<Vec<u32>>,
    released: Vec<Vec<u32>>,
    fail_step_fatally: bool,
    caps: Vec<wire::Capability>,
    fog: Option<f64>,
    road: Option<(f64, String)>,
    /// A world that shows what it was sent only after its next tick (CARLA 0.9.16's weather,
    /// scensim issue 161): the fog sent, waiting for the tick.
    lag: bool,
    fog_at_tick: Option<f64>,
    steps: u32,
    initial_speeds: Vec<(u32, f64)>,
}

impl World for Fake {
    fn simulator_name(&self) -> &str {
        "fake"
    }
    fn simulator_version(&self) -> String {
        "fake-0".into()
    }
    fn capabilities(&self) -> Vec<wire::Capability> {
        self.caps.clone()
    }
    fn load_world(&mut self, _r: &wire::LoadWorld) -> WorldResult<wire::LoadWorldReply> {
        self.spawned.clear();
        Ok(wire::LoadWorldReply {
            sim_time_ns: 7,
            episode_id: 1,
        })
    }
    fn spawn(&mut self, handle: u32, _r: &wire::Spawn) -> WorldResult<wire::Body> {
        self.spawned.push(handle);
        Ok(wire::Body::default())
    }
    fn despawn(&mut self, handle: u32) -> WorldResult<()> {
        self.spawned.retain(|h| *h != handle);
        Ok(())
    }
    fn apply(&mut self, _r: &wire::Apply) -> WorldResult<wire::ApplyReply> {
        Ok(wire::ApplyReply::default())
    }
    fn step(&mut self, _r: &wire::Step) -> WorldResult<wire::StepReply> {
        if self.fail_step_fatally {
            return Err(Refusal::fatal("the tick never returned"));
        }
        self.steps += 1;
        if let Some(f) = self.fog_at_tick.take() {
            self.fog = Some(f);
        }
        Ok(wire::StepReply {
            frame: 1,
            ..Default::default()
        })
    }
    fn reset(&mut self, handles: &[u32]) -> WorldResult<()> {
        self.resets.push(handles.to_vec());
        self.spawned.retain(|h| !handles.contains(h));
        Ok(())
    }
    fn release(&mut self, handles: &[u32]) -> WorldResult<()> {
        self.released.push(handles.to_vec());
        Ok(())
    }
    fn set_environment(&mut self, r: &wire::SetEnvironment) -> WorldResult<()> {
        if let Some(f) = r.weather.as_ref().and_then(|w| w.fog) {
            // a world that shows fog only to the nearest 10 m
            let shown = (f.visual_range / 10.0).round() * 10.0;
            if self.lag {
                self.fog_at_tick = Some(shown);
            } else {
                self.fog = Some(shown);
            }
        }
        Ok(())
    }
    fn set_road_condition(&mut self, r: &wire::RoadCondition) -> WorldResult<()> {
        // a world that holds the scale to the nearest 0.05
        self.road = Some((
            (r.friction_scale_factor / 0.05).round() * 0.05,
            r.wetness.clone(),
        ));
        Ok(())
    }
    fn applied_road_condition(
        &mut self,
        _r: &wire::RoadCondition,
    ) -> WorldResult<wire::RoadConditionReply> {
        let (f, w) = self.road.clone().unwrap_or((1.0, String::new()));
        Ok(wire::RoadConditionReply {
            friction_scale_factor: f,
            wetness: w,
            zones: Default::default(),
        })
    }
    fn set_initial_speed(&mut self, handle: u32, speed: f64) -> WorldResult<()> {
        self.initial_speeds.push((handle, speed));
        Ok(())
    }
    fn applied_environment(&mut self) -> WorldResult<wire::ScenarioWeather> {
        Ok(wire::ScenarioWeather {
            fog: self
                .fog
                .map(|v| wire::scenario_weather::Fog { visual_range: v }),
            ..Default::default()
        })
    }
}

fn id(pid: u32) -> wire::ClientId {
    wire::ClientId {
        host: "testhost".into(),
        pid,
        start_time: u64::from(pid),
    }
}

/// Clients whose pid is 999 are gone.
fn gone(c: &wire::ClientId) -> bool {
    c.pid == 999
}

fn call(shell: &mut Shell<Fake>, client: u32, request: wire::request::Request) -> wire::Reply {
    let bytes = wire::Request {
        request: Some(request),
        client: Some(id(client)),
    }
    .encode_to_vec();
    wire::Reply::decode(shell.handle(&bytes).as_slice()).unwrap()
}

fn hello(shell: &mut Shell<Fake>, client: u32, probe: bool) -> wire::Reply {
    call(
        shell,
        client,
        wire::request::Request::Hello(wire::Hello {
            protocol_version: PROTOCOL_VERSION,
            probe,
        }),
    )
}

fn spawn(shell: &mut Shell<Fake>, client: u32, name: &str, motion: wire::Motion) -> wire::Reply {
    call(
        shell,
        client,
        wire::request::Request::Spawn(wire::Spawn {
            name: name.into(),
            kind: wire::Kind::Vehicle as i32,
            motion: motion as i32,
            ..Default::default()
        }),
    )
}

fn shell(caps: Vec<wire::Capability>) -> Shell<Fake> {
    Shell::new(Fake {
        caps,
        ..Default::default()
    })
    .with_lease(Duration::from_millis(200), gone)
}

#[test]
fn hello_first_and_in_our_version() {
    let mut s = shell(vec![]);
    let r = call(
        &mut s,
        1,
        wire::request::Request::LoadWorld(Default::default()),
    );
    assert!(!r.ok);
    assert!(r.error.contains("before hello"), "{}", r.error);
    let r = call(
        &mut s,
        1,
        wire::request::Request::Hello(wire::Hello {
            protocol_version: PROTOCOL_VERSION + 1,
            probe: false,
        }),
    );
    assert!(!r.ok);
    let r = hello(&mut s, 1, false);
    assert!(r.ok, "{}", r.error);
    let Some(wire::reply::Reply::Hello(h)) = r.reply else {
        panic!("no hello reply");
    };
    assert_eq!(h.simulator_name, "fake");
    assert_eq!(h.owner.unwrap().pid, 1);
    assert!(h.ready);
}

#[test]
fn handles_are_the_shells_and_unique() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    let r = spawn(&mut s, 1, "a", wire::Motion::Kinematic);
    let Some(wire::reply::Reply::Spawn(a)) = r.reply else {
        panic!("{}", r.error);
    };
    assert_eq!(a.handle, 1);
    assert!(!spawn(&mut s, 1, "a", wire::Motion::Kinematic).ok);
    let r = call(
        &mut s,
        1,
        wire::request::Request::Despawn(wire::Despawn { handle: 9 }),
    );
    assert!(r.error.contains("no entity with handle 9"), "{}", r.error);
    let r = call(
        &mut s,
        1,
        wire::request::Request::Apply(wire::Apply {
            poses: vec![wire::EntityPose {
                handle: 9,
                ..Default::default()
            }],
            ..Default::default()
        }),
    );
    assert!(r.error.contains("unknown handle"), "{}", r.error);
    call(
        &mut s,
        1,
        wire::request::Request::LoadWorld(Default::default()),
    );
    let Some(wire::reply::Reply::Spawn(b)) = spawn(&mut s, 1, "a", wire::Motion::Kinematic).reply
    else {
        panic!("respawn after load");
    };
    assert_eq!(b.handle, 2);
}

#[test]
fn what_is_not_offered_is_refused() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    for (motion, needle) in [
        (wire::Motion::Tracked, "TRACKED bodies"),
        (wire::Motion::Autopilot, "AUTOPILOT bodies"),
        (wire::Motion::External, "other than the ego"),
        (wire::Motion::Commanded, "COMMANDED bodies"),
    ] {
        let r = spawn(&mut s, 1, "x", motion);
        assert!(!r.ok && r.error.contains(needle), "{motion:?}: {}", r.error);
    }
    for q in [
        wire::request::Request::SetSignals(Default::default()),
        wire::request::Request::GetSignals(Default::default()),
        wire::request::Request::SetGoal(Default::default()),
        wire::request::Request::SetWeather(Default::default()),
        wire::request::Request::GetWeather(Default::default()),
        wire::request::Request::SetRoadCondition(Default::default()),
        wire::request::Request::SetEnvironment(Default::default()),
        wire::request::Request::SetInitialSpeed(Default::default()),
    ] {
        let r = call(&mut s, 1, q);
        assert!(!r.ok && r.error.contains("not offered"), "{}", r.error);
    }
    let mut s = shell(vec![
        wire::Capability::TracksTargets,
        wire::Capability::Autopilot,
        wire::Capability::Commands,
    ]);
    hello(&mut s, 1, false);
    assert!(spawn(&mut s, 1, "t", wire::Motion::Tracked).ok);
    assert!(spawn(&mut s, 1, "a", wire::Motion::Autopilot).ok);
    assert!(spawn(&mut s, 1, "m", wire::Motion::Commanded).ok);
}

#[test]
fn one_owner_at_a_time() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    spawn(&mut s, 1, "npc", wire::Motion::Kinematic);
    // Another bridge: refused, and refused for every other request.
    let r = hello(&mut s, 2, false);
    assert!(
        !r.ok && r.error.contains("owned by testhost:1"),
        "{}",
        r.error
    );
    let r = call(&mut s, 2, wire::request::Request::Step(Default::default()));
    assert!(r.error.contains("owned by testhost:1"), "{}", r.error);
    // A probe is answered and changes nothing.
    let r = hello(&mut s, 2, true);
    assert!(r.ok);
    assert!(s.world.resets.is_empty());
    // The owner's own hello keeps its world.
    assert!(hello(&mut s, 1, false).ok);
    assert_eq!(s.world.spawned, vec![1]);
}

#[test]
fn a_dead_owner_is_replaced_at_once_and_a_silent_one_after_its_lease() {
    let mut s = shell(vec![]);
    hello(&mut s, 999, false);
    spawn(&mut s, 999, "npc", wire::Motion::Kinematic);
    let r = hello(&mut s, 2, false);
    assert!(r.ok, "{}", r.error);
    assert_eq!(s.world.resets, vec![vec![1]]);
    assert_eq!(s.owner().unwrap().pid, 2);
    // Pid 2 now owns it; pid 3 waits out the lease.
    assert!(!hello(&mut s, 3, false).ok);
    std::thread::sleep(Duration::from_millis(250));
    assert!(hello(&mut s, 3, false).ok);
    assert_eq!(s.owner().unwrap().pid, 3);
}

#[test]
fn release_leaves_no_owner() {
    let mut s = shell(vec![]);
    hello(&mut s, 1, false);
    spawn(&mut s, 1, "npc", wire::Motion::Kinematic);
    assert!(
        call(
            &mut s,
            1,
            wire::request::Request::Release(Default::default())
        )
        .ok
    );
    assert_eq!(s.world.released, vec![vec![1]]);
    assert!(s.owner().is_none());
    assert!(hello(&mut s, 2, false).ok);
    assert!(s.world.resets.is_empty(), "nothing left to reset");
}

#[test]
fn a_fatal_call_ends_the_server() {
    let mut s = shell(vec![]);
    s.world.fail_step_fatally = true;
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, wire::request::Request::Step(Default::default()));
    assert!(!r.ok && r.fatal);
    assert_eq!(s.fatal(), Some("the tick never returned"));
    // Everything after is refused as fatal too.
    let r = call(&mut s, 1, wire::request::Request::Apply(Default::default()));
    assert!(r.fatal);
}

#[test]
fn a_retired_road_condition_property_is_refused_naming_its_successor() {
    // roadmap 080: `swb.*` became `scensim.*`; the old name is refused, never aliased
    let mut s = shell(vec![wire::Capability::RoadCondition]);
    hello(&mut s, 1, false);
    let mut rc = wire::RoadCondition {
        friction_scale_factor: 1.0,
        ..Default::default()
    };
    rc.properties
        .insert("swb.zone.ford.class".into(), "ice".into());
    let r = call(
        &mut s,
        1,
        wire::request::Request::SetRoadCondition(wire::SetRoadCondition {
            road_condition: Some(rc),
        }),
    );
    assert!(!r.ok, "a retired property was taken");
    assert!(
        r.error
            .contains("swb.zone.ford.class: renamed scensim.zone.ford.class"),
        "{}",
        r.error
    );
}

#[test]
fn set_environment_answers_what_the_world_shows_when_it_offers_the_read_back() {
    let fog = |v: f64| {
        wire::request::Request::SetEnvironment(wire::SetEnvironment {
            weather: Some(wire::ScenarioWeather {
                fog: Some(wire::scenario_weather::Fog { visual_range: v }),
                ..Default::default()
            }),
            time_of_day: None,
        })
    };
    // ENVIRONMENT alone: applied, no read-back (as before 092 K9)
    let mut s = shell(vec![wire::Capability::Environment]);
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, fog(123.0));
    assert!(r.ok && r.reply.is_none(), "{r:?}");
    // with ENVIRONMENT_APPLIED: the world's own value, not the one asked for
    let mut s = shell(vec![
        wire::Capability::Environment,
        wire::Capability::EnvironmentApplied,
    ]);
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, fog(123.0));
    match r.reply {
        Some(wire::reply::Reply::Environment(e)) => {
            assert_eq!(e.applied.unwrap().fog.unwrap().visual_range, 120.0)
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn set_road_condition_answers_what_the_world_applies_when_it_offers_the_read_back() {
    let rc = |f: f64| {
        wire::request::Request::SetRoadCondition(wire::SetRoadCondition {
            road_condition: Some(wire::RoadCondition {
                friction_scale_factor: f,
                wetness: "moist".into(),
                ..Default::default()
            }),
        })
    };
    // ROAD_CONDITION alone: applied, no read-back (as before 092 K9f-a and 086 C7)
    let mut s = shell(vec![wire::Capability::RoadCondition]);
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, rc(0.83));
    assert!(r.ok && r.reply.is_none(), "{r:?}");
    // with ROAD_CONDITION_APPLIED: the world's own value, not the one asked for
    let mut s = shell(vec![
        wire::Capability::RoadCondition,
        wire::Capability::RoadConditionApplied,
    ]);
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, rc(0.83));
    match r.reply {
        Some(wire::reply::Reply::RoadCondition(a)) => {
            assert!((a.friction_scale_factor - 0.85).abs() < 1e-9, "{a:?}");
            assert_eq!(a.wetness, "moist");
            assert!(a.zones.is_empty());
        }
        other => panic!("{other:?}"),
    }
}

/// Issue 135 step 3: the boot id a Hello carries is this host's, a UUID.
#[test]
fn boot_id_is_this_hosts_uuid() {
    let id = super::boot_id();
    assert_eq!(id.len(), 36, "{id}");
    assert_eq!(id.matches('-').count(), 4, "{id}");
    assert_eq!(id, super::boot_id());
}

fn environment(w: wire::ScenarioWeather) -> wire::request::Request {
    wire::request::Request::SetEnvironment(wire::SetEnvironment {
        weather: Some(w),
        time_of_day: None,
    })
}

fn sun(azimuth: f64, elevation: f64) -> wire::ScenarioWeather {
    wire::ScenarioWeather {
        sun: Some(wire::scenario_weather::Sun {
            azimuth,
            elevation,
            illuminance: 0.0,
        }),
        ..Default::default()
    }
}

fn fog_of(v: f64) -> wire::ScenarioWeather {
    wire::ScenarioWeather {
        fog: Some(wire::scenario_weather::Fog { visual_range: v }),
        ..Default::default()
    }
}

fn step(shell: &mut Shell<Fake>) -> wire::StepReply {
    match call(
        shell,
        1,
        wire::request::Request::Step(wire::Step::default()),
    )
    .reply
    {
        Some(wire::reply::Reply::Step(r)) => r,
        other => panic!("{other:?}"),
    }
}

/// Issue 161: a value outside its field's range refuses the whole request, by field, value and
/// range with their unit, and the world is not called.
#[test]
fn issue_161_out_of_range_environment_values_are_refused_by_name_with_the_unit() {
    use std::f64::consts::{FRAC_PI_2, PI};
    let cases: Vec<(wire::ScenarioWeather, &str)> = vec![
        (
            sun(0.0, 1.6),
            "sun_elevation 1.6 rad is outside [-1.5708, 1.5708] rad",
        ),
        (sun(0.0, -1.6), "sun_elevation -1.6 rad"),
        (
            sun(-0.1, 0.0),
            "sun_azimuth -0.1 rad is outside [0, 6.2832) rad",
        ),
        (sun(2.0 * PI, 0.0), "sun_azimuth 6.28"),
        (sun(f64::NAN, 0.0), "sun_azimuth NaN rad"),
        (
            fog_of(0.0),
            "fog_visual_range 0 m is outside its range, above 0 m",
        ),
        (fog_of(f64::NAN), "fog_visual_range NaN m"),
        (
            wire::ScenarioWeather {
                cloud_cover: Some(wire::scenario_weather::CloudCover { oktas: 10 }),
                ..Default::default()
            },
            "cloud_cover 10 oktas is outside 0 to 9 oktas",
        ),
        (
            wire::ScenarioWeather {
                precipitation: Some(wire::scenario_weather::Precipitation {
                    r#type: wire::scenario_weather::PrecipitationType::Rain as i32,
                    intensity: -1.0,
                }),
                ..Default::default()
            },
            "precipitation_intensity -1 mm/h is outside its range, 0 or more mm/h",
        ),
    ];
    for (w, says) in cases {
        let mut s = shell(vec![
            wire::Capability::Environment,
            wire::Capability::EnvironmentApplied,
        ]);
        hello(&mut s, 1, false);
        // a fog that is in range rides along and is not applied either
        let mut w = w;
        if w.fog.is_none() {
            w.fog = Some(wire::scenario_weather::Fog {
                visual_range: 500.0,
            });
        }
        let r = call(&mut s, 1, environment(w));
        assert!(!r.ok, "{r:?}");
        assert!(r.error.contains(says), "{} / {says}", r.error);
        assert!(r.error.contains("nothing changed"), "{}", r.error);
        assert_eq!(s.world.fog, None, "applied though refused");
    }
    // the bounds themselves are in range
    for w in [
        sun(0.0, FRAC_PI_2),
        sun(0.0, -FRAC_PI_2),
        sun(0.0, 0.0),
        fog_of(0.1),
        wire::ScenarioWeather {
            cloud_cover: Some(wire::scenario_weather::CloudCover { oktas: 9 }),
            precipitation: Some(wire::scenario_weather::Precipitation {
                r#type: wire::scenario_weather::PrecipitationType::Dry as i32,
                intensity: 0.0,
            }),
            ..Default::default()
        },
    ] {
        let mut s = shell(vec![wire::Capability::Environment]);
        hello(&mut s, 1, false);
        let r = call(&mut s, 1, environment(w));
        assert!(r.ok, "{r:?}");
    }
}

/// Issue 161, the K9e case (133): 60 meant as degrees, sent as an elevation in radians, is
/// refused, the message saying rad and what 60 deg would have been.
#[test]
fn issue_161_k9e_60_sent_as_radians_is_refused_saying_the_units() {
    let mut s = shell(vec![wire::Capability::Environment]);
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, environment(sun(0.0, 60.0)));
    assert!(!r.ok);
    assert!(
        r.error.contains(
            "sun_elevation 60 rad is outside [-1.5708, 1.5708] rad (60 deg would be 1.0472 rad)"
        ),
        "{}",
        r.error
    );
    let r = call(&mut s, 1, environment(sun(0.0, 60f64.to_radians())));
    assert!(r.ok, "{r:?}");
}

/// Issue 161: with READBACK_AFTER_STEP the first step after a send carries the read-back taken
/// after the tick; the send's own reply may predate it (a world that shows it only after a tick).
#[test]
fn issue_161_the_step_after_a_send_carries_the_read_back_after_its_tick() {
    let mut s = shell(vec![
        wire::Capability::Environment,
        wire::Capability::EnvironmentApplied,
        wire::Capability::ReadbackAfterStep,
    ]);
    s.world.lag = true;
    hello(&mut s, 1, false);
    let r = call(&mut s, 1, environment(fog_of(123.0)));
    match r.reply {
        // before the tick: the world still shows no fog
        Some(wire::reply::Reply::Environment(e)) => assert_eq!(e.applied.unwrap().fog, None),
        other => panic!("{other:?}"),
    }
    let r = step(&mut s);
    assert_eq!(
        r.environment_applied.unwrap().fog.unwrap().visual_range,
        120.0
    );
    assert!(r.road_condition_applied.is_none());
    // a step with no send carries none
    assert!(step(&mut s).environment_applied.is_none());
    // without the bit, no step carries one
    let mut s = shell(vec![
        wire::Capability::Environment,
        wire::Capability::EnvironmentApplied,
    ]);
    hello(&mut s, 1, false);
    assert!(call(&mut s, 1, environment(fog_of(123.0))).ok);
    assert!(step(&mut s).environment_applied.is_none());
}

/// Issue 161: the road condition's read-back after the step, the same way.
#[test]
fn issue_161_the_step_after_a_road_condition_carries_its_read_back() {
    let mut s = shell(vec![
        wire::Capability::RoadCondition,
        wire::Capability::RoadConditionApplied,
        wire::Capability::ReadbackAfterStep,
    ]);
    hello(&mut s, 1, false);
    let r = call(
        &mut s,
        1,
        wire::request::Request::SetRoadCondition(wire::SetRoadCondition {
            road_condition: Some(wire::RoadCondition {
                friction_scale_factor: 0.83,
                wetness: "moist".into(),
                ..Default::default()
            }),
        }),
    );
    assert!(r.ok, "{r:?}");
    let r = step(&mut s);
    let a = r.road_condition_applied.unwrap();
    assert!((a.friction_scale_factor - 0.85).abs() < 1e-9, "{a:?}");
    assert!(r.environment_applied.is_none());
    assert!(step(&mut s).road_condition_applied.is_none());
}

fn initial_speed(shell: &mut Shell<Fake>, handle: u32, speed: f64) -> wire::Reply {
    call(
        shell,
        1,
        wire::request::Request::SetInitialSpeed(wire::SetInitialSpeed { handle, speed }),
    )
}

fn initial_speed_shell() -> Shell<Fake> {
    let mut s = shell(vec![
        wire::Capability::InitialSpeed,
        wire::Capability::AgentVehicles,
        wire::Capability::Autopilot,
        wire::Capability::Commands,
        wire::Capability::TracksTargets,
    ]);
    hello(&mut s, 1, false);
    s
}

#[test]
fn an_initial_speed_reaches_the_world_before_the_first_step() {
    let mut s = initial_speed_shell();
    for (i, motion) in [
        wire::Motion::External,
        wire::Motion::Autopilot,
        wire::Motion::Commanded,
    ]
    .into_iter()
    .enumerate()
    {
        assert!(spawn(&mut s, 1, &format!("e{i}"), motion).ok);
        let r = initial_speed(&mut s, i as u32 + 1, 7.5);
        assert!(r.ok, "{motion:?}: {}", r.error);
    }
    assert_eq!(s.world.initial_speeds, vec![(1, 7.5), (2, 7.5), (3, 7.5)]);
}

#[test]
fn an_initial_speed_after_the_entitys_first_step_is_refused() {
    let mut s = initial_speed_shell();
    spawn(&mut s, 1, "ego", wire::Motion::External);
    assert!(call(&mut s, 1, wire::request::Request::Step(Default::default())).ok);
    let r = initial_speed(&mut s, 1, 7.5);
    assert!(
        !r.ok && r.error.contains("after its first Step"),
        "{}",
        r.error
    );
    // An entity spawned later still takes one before the next Step.
    spawn(&mut s, 1, "late", wire::Motion::External);
    let r = initial_speed(&mut s, 2, 3.0);
    assert!(r.ok, "{}", r.error);
    assert_eq!(s.world.initial_speeds, vec![(2, 3.0)]);
}

#[test]
fn a_posed_body_takes_no_initial_speed() {
    let mut s = initial_speed_shell();
    spawn(&mut s, 1, "k", wire::Motion::Kinematic);
    spawn(&mut s, 1, "t", wire::Motion::Tracked);
    for h in [1, 2] {
        let r = initial_speed(&mut s, h, 5.0);
        assert!(!r.ok && r.error.contains("posed"), "{}", r.error);
    }
    assert!(s.world.initial_speeds.is_empty());
}

#[test]
fn an_initial_speed_must_be_finite_non_negative_and_for_a_known_handle() {
    let mut s = initial_speed_shell();
    spawn(&mut s, 1, "ego", wire::Motion::External);
    for v in [-1.0, f64::NAN, f64::INFINITY] {
        let r = initial_speed(&mut s, 1, v);
        assert!(
            !r.ok && r.error.contains("finite and >= 0"),
            "{v}: {}",
            r.error
        );
    }
    let r = initial_speed(&mut s, 9, 5.0);
    assert!(r.error.contains("no entity with handle 9"), "{}", r.error);
    assert!(s.world.initial_speeds.is_empty());
}

#[test]
fn a_load_world_forgets_which_entities_have_stepped() {
    let mut s = initial_speed_shell();
    spawn(&mut s, 1, "ego", wire::Motion::External);
    call(&mut s, 1, wire::request::Request::Step(Default::default()));
    call(
        &mut s,
        1,
        wire::request::Request::LoadWorld(Default::default()),
    );
    spawn(&mut s, 1, "ego", wire::Motion::External);
    let r = initial_speed(&mut s, 2, 4.0);
    assert!(r.ok, "{}", r.error);
}
