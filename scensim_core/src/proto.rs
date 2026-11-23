//! Generated protobuf types from SSv2's `simulation_interface` protocol.
//!
//! All packages are siblings under this module so cross-package references (e.g.
//! `traffic_simulator_msgs` using `super::autoware_vehicle_msgs`) resolve correctly.
//!
//! `dead_code` is allowed throughout: these modules are a transcription of the wire
//! protocol, and a message type this bridge happens not to construct is a normal state, not
//! an unused-code smell. Denying it would make `just check` fail for messages SSv2 defines
//! and we simply do not send.

#[allow(clippy::all, dead_code)]
pub mod simulation_api_schema {
    include!(concat!(env!("OUT_DIR"), "/simulation_api_schema.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod geometry_msgs {
    include!(concat!(env!("OUT_DIR"), "/geometry_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod traffic_simulator_msgs {
    include!(concat!(env!("OUT_DIR"), "/traffic_simulator_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod builtin_interfaces {
    include!(concat!(env!("OUT_DIR"), "/builtin_interfaces.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod autoware_control_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_control_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod autoware_vehicle_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_vehicle_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod rosgraph_msgs {
    include!(concat!(env!("OUT_DIR"), "/rosgraph_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod std_msgs {
    include!(concat!(env!("OUT_DIR"), "/std_msgs.rs"));
}

// SSv2's protos above are the fork's files byte for byte; the scenario bridge, which holds
// the fork, checks that (scensim's `just test`).

/// scensim's world protocol with the Isaac Sim world server (`proto/scensim_world.proto`, roadmap
/// 037): scensim's own, not SSv2's.
#[allow(clippy::all, dead_code)]
pub mod scensim_world {
    include!(concat!(env!("OUT_DIR"), "/scensim_world.rs"));
}

#[cfg(test)]
mod tests {
    use super::scensim_world::{AppliedControl, EntityState};
    use prost::Message;

    /// Issue 135: `applied_control` round-trips, and a peer that never sets it (an older server,
    /// Isaac) reads as unset, not as a zero control.
    #[test]
    fn applied_control_round_trips_and_is_unset_when_absent() {
        let c = AppliedControl {
            throttle: 0.25,
            brake: 0.5,
            steer: -0.75,
            hand_brake: true,
            reverse: true,
            manual_gear_shift: true,
            gear: -1,
        };
        let with = EntityState {
            handle: 3,
            applied_control: Some(c),
            ..Default::default()
        };
        let back = EntityState::decode(with.encode_to_vec().as_slice()).unwrap();
        assert_eq!(back.applied_control, Some(c));
        let without = EntityState {
            handle: 3,
            cmd_frame: 7,
            ..Default::default()
        };
        let back = EntityState::decode(without.encode_to_vec().as_slice()).unwrap();
        assert_eq!(back.applied_control, None);
        assert_eq!(back.cmd_frame, 7);
    }
}
