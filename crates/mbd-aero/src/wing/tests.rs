use glam::DVec3;
use mbd_core::AmbientGas;

use super::*;

fn swept_wing(sweep_rad: f64, ac_pos: DVec3) -> WingSurfaceAero {
    WingSurfaceAero::new(WingSurfaceParams {
        area_m2: 120.0,
        mean_chord_m: 4.0,
        span_m: 35.0,
        aspect_ratio: 10.2,
        sweep_quarter_chord_rad: sweep_rad,
        sweep_control_channel: Some(3),
        cl_alpha_incompressible: 5.0,
        cl0: 0.15,
        cl_max: 1.4,
        alpha_stall_rad: 0.28,
        cd0: 0.018,
        oswald_efficiency: 0.85,
        wave_drag_factor: 1.5,
        critical_mach: 0.72,
        cm0: -0.04,
        ac_position_body_m: ac_pos,
        chord_direction_body: DVec3::X,
        lift_normal_body: -DVec3::Z,
        flap_channel: Some(4),
        flap_dcl: 0.8,
        flap_dcd: 0.05,
        aileron_channel: Some(0),
        aileron_dcl: 0.4,
    })
    .unwrap()
}

#[test]
fn positive_alpha_lifts_upward_and_orthogonally_to_flow() {
    let wing = swept_wing(0.2, DVec3::ZERO);
    let air = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    let alpha = 0.08_f64;
    let v = DVec3::new(alpha.cos(), 0.0, alpha.sin()) * 80.0;
    let w = wing
        .evaluate(&LinkFlow {
            velocity_air_link_m_s: v,
            angular_rate_link_rad_s: DVec3::ZERO,
            ambient: air,
            controls: &[],
        })
        .unwrap();
    assert!(w.force_link_n.z < 0.0, "lift must point -Z up");
    assert!(w.force_link_n.dot(v) < 0.0, "drag opposes flight path");
}

#[test]
fn variable_sweep_reduces_transonic_wave_drag() {
    let wing = swept_wing(0.35, DVec3::ZERO);
    let air = AmbientGas::earth_air(26_500.0, 223.0).unwrap();
    let speed = 0.90 * air.sound_speed_m_s;

    let forward_sweep = wing
        .evaluate(&LinkFlow {
            velocity_air_link_m_s: DVec3::X * speed,
            angular_rate_link_rad_s: DVec3::ZERO,
            ambient: air,
            controls: &[],
        })
        .unwrap();

    let mut controls = [0.0; 8];
    controls[3] = 0.75; // sweep back further (to ~1.1 rad ~ 63 deg)
    let swept_back = wing
        .evaluate(&LinkFlow {
            velocity_air_link_m_s: DVec3::X * speed,
            angular_rate_link_rad_s: DVec3::ZERO,
            ambient: air,
            controls: &controls,
        })
        .unwrap();

    assert!((-swept_back.force_link_n.x) < (-forward_sweep.force_link_n.x));
}

#[test]
fn wing_roll_rate_produces_restoring_roll_damping() {
    let right_wing = swept_wing(0.2, DVec3::new(0.0, 8.0, 0.0));
    let left_wing = swept_wing(0.2, DVec3::new(0.0, -8.0, 0.0));
    let air = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    let flow = LinkFlow {
        velocity_air_link_m_s: DVec3::X * 100.0,
        angular_rate_link_rad_s: DVec3::new(0.5, 0.0, 0.0), // rolling right wing down
        ambient: air,
        controls: &[],
    };
    let mr = right_wing
        .evaluate(&flow)
        .unwrap()
        .moment_about(DVec3::ZERO);
    let ml = left_wing.evaluate(&flow).unwrap().moment_about(DVec3::ZERO);
    assert!(
        (mr.x + ml.x) < 0.0,
        "roll rate +x must be damped (-x moment)"
    );
}
