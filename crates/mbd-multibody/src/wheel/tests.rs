use glam::{DQuat, DVec3};
use mbd_core::ComponentId;

use super::*;
use crate::contact::{SurfaceMedium, SurfaceSample};

#[test]
fn driven_mars_rover_wheel_propels_forward_and_brakes_oppose_motion() {
    let spec = WheelSpec {
        component: ComponentId(0),
        hub_offset_local_m: DVec3::ZERO,
        wheel_radius_m: 0.26,
        spring_n_m: 12_000.0,
        damper_n_s_m: 400.0,
        forward_axis_local: DVec3::X,
        axle_axis_local: DVec3::Y,
        peak_mu: 0.75,
        cornering_stiffness_n_rad: 2_500.0,
        rolling_resistance_coeff: 0.04,
        longitudinal_stiffness_n: 4_000.0,
    };
    let ground = SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    };
    let mut state = WheelState::default();
    let cmd = WheelCommand {
        steer_angle_rad: 0.0,
        drive_torque_nm: 60.0,
        brake_torque_nm: 0.0,
    };
    let mut out = evaluate_wheel(
        &spec,
        &mut state,
        &cmd,
        DVec3::new(0.0, 0.0, 0.22),
        DVec3::new(0.2, 0.0, 0.0),
        DQuat::IDENTITY,
        &ground,
        0.4,
        0.02,
    )
    .unwrap();
    for _ in 0..10 {
        out = evaluate_wheel(
            &spec,
            &mut state,
            &cmd,
            DVec3::new(0.0, 0.0, 0.22),
            DVec3::new(0.2, 0.0, 0.0),
            DQuat::IDENTITY,
            &ground,
            0.4,
            0.02,
        )
        .unwrap();
    }
    assert!(out.contact_load_n > 400.0);
    assert!(out.wrench.force_local_n.x > 100.0);

    // Full brake at forward roll speed produces decelerating (-x) friction.
    state.spin_rate_rad_s = 0.0;
    let brake = WheelCommand {
        steer_angle_rad: 0.0,
        drive_torque_nm: 0.0,
        brake_torque_nm: 200.0,
    };
    let brk = evaluate_wheel(
        &spec,
        &mut state,
        &brake,
        DVec3::new(0.0, 0.0, 0.22),
        DVec3::new(15.0, 0.0, 0.0),
        DQuat::IDENTITY,
        &ground,
        0.4,
        0.02,
    )
    .unwrap();
    assert!(brk.wrench.force_local_n.x < -200.0);
}
