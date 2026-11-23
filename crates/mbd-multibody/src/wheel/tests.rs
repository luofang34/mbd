use glam::{DMat3, DVec3};
use mbd_core::ComponentId;

use super::*;
use crate::contact::{SurfaceMedium, SurfaceSample};

const SPIN_INERTIA_KG_M2: f64 = 0.4;
const DT_S: f64 = 0.02;

/// One explicit-Euler step of a wheel at a prescribed hub motion: evaluate,
/// then advance the spin.
fn step_wheel(
    spec: &WheelSpec,
    state: &mut WheelState,
    cmd: &WheelCommand,
    hub_world_pos: DVec3,
    hub_world_vel: DVec3,
    surface: &SurfaceSample,
) -> WheelEvaluation {
    let hub = PointKinematics {
        position_world_m: hub_world_pos,
        velocity_world_m_s: hub_world_vel,
        angular_vel_world_rad_s: DVec3::ZERO,
        rotation_world_from_link: DMat3::IDENTITY,
    };
    let out = spec
        .evaluate(state, cmd, &hub, surface, SPIN_INERTIA_KG_M2)
        .unwrap();
    state.spin_rate_rad_s += out.spin_accel_rad_s2 * DT_S;
    out
}

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
        tire: TireModelSpec::DEFAULT,
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
    let mut out = step_wheel(
        &spec,
        &mut state,
        &cmd,
        DVec3::new(0.0, 0.0, 0.22),
        DVec3::new(0.2, 0.0, 0.0),
        &ground,
    );
    for _ in 0..10 {
        out = step_wheel(
            &spec,
            &mut state,
            &cmd,
            DVec3::new(0.0, 0.0, 0.22),
            DVec3::new(0.2, 0.0, 0.0),
            &ground,
        );
    }
    assert!(out.contact_load_n > 400.0);
    assert!(out.force_world_n.x > 100.0);

    // Full brake at forward roll speed produces decelerating (-x) friction.
    state.spin_rate_rad_s = 0.0;
    let brake = WheelCommand {
        steer_angle_rad: 0.0,
        drive_torque_nm: 0.0,
        brake_torque_nm: 200.0,
    };
    let brk = step_wheel(
        &spec,
        &mut state,
        &brake,
        DVec3::new(0.0, 0.0, 0.22),
        DVec3::new(15.0, 0.0, 0.0),
        &ground,
    );
    assert!(brk.force_world_n.x < -200.0);
}
