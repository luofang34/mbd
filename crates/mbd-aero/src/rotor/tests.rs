use glam::DVec3;
use mbd_core::AmbientGas;

use super::*;

fn ingenuity_rotor(direction: f64, hub: DVec3) -> RotorBemAero {
    RotorBemAero::new(RotorBemParams {
        radius_m: 0.605,
        hub_radius_m: 0.08,
        blade_count: 2,
        chord_m: 0.075,
        root_pitch_rad: 19f64.to_radians(),
        tip_pitch_rad: 9f64.to_radians(),
        cl_alpha: 5.5,
        cl_max: 1.25,
        cd0_high_re: 0.018,
        low_re_drag_factor: 4.5,
        critical_tip_mach: 0.72,
        coaxial_interference_factor: 1.15,
        rotation_direction: direction,
        hub_position_body_m: hub,
        shaft_axis_body: -DVec3::Z, // upward lift
        radial_stations: 10,
        omega_channel: 0,
        collective_channel: Some(1),
        cyclic_lat_channel: None,
        cyclic_lon_channel: None,
        numerics: RotorBemNumerics::DEFAULT,
    })
    .unwrap()
}

#[test]
fn ingenuity_coaxial_pair_hovers_on_mars_co2_with_torque_cancellation() {
    let mars = AmbientGas::mars_air(610.0, 215.0).unwrap();
    let upper = ingenuity_rotor(1.0, DVec3::new(0.0, 0.0, -0.3));
    let lower = ingenuity_rotor(-1.0, DVec3::new(0.0, 0.0, -0.2));
    let hover = LinkFlow {
        velocity_air_link_m_s: DVec3::ZERO,
        angular_rate_link_rad_s: DVec3::ZERO,
        ambient: mars,
        controls: &[],
    };
    let state = RotorState {
        omega_rad_s: 265.7,
        collective_rad: 0.01,
        ..RotorState::default()
    };
    let ru = upper.evaluate_state(&hover, &state).unwrap();
    let rl = lower.evaluate_state(&hover, &state).unwrap();
    let total_thrust_n = -(ru.aero.force_link_n.z + rl.aero.force_link_n.z);
    let net_yaw_torque = ru.aero.moment_ref_link_nm.z + rl.aero.moment_ref_link_nm.z;
    let total_power_w = ru.shaft_power_w + rl.shaft_power_w;

    // 1.8 kg Mars weight ~ 6.70 N; expect hover-capable thrust around 5..9 N and ~150..500 W.
    assert!(
        (5.0..10.0).contains(&total_thrust_n),
        "thrust {total_thrust_n}"
    );
    assert!(
        (100.0..500.0).contains(&total_power_w),
        "power {total_power_w}"
    );
    assert!(net_yaw_torque.abs() < 1e-12, "coaxial reaction cancels");
    assert!((0.65..0.75).contains(&ru.tip_mach));
    assert!(ru.mean_reynolds < 25_000.0, "Mars low Reynolds number");
}

#[test]
fn earth_aircraft_propeller_generates_forward_thrust_and_p_factor_damping() {
    let prop = RotorBemAero::new(RotorBemParams {
        radius_m: 0.95,
        hub_radius_m: 0.14,
        blade_count: 2,
        chord_m: 0.12,
        root_pitch_rad: 26f64.to_radians(),
        tip_pitch_rad: 14f64.to_radians(),
        cl_alpha: 5.7,
        cl_max: 1.3,
        cd0_high_re: 0.012,
        low_re_drag_factor: 1.0,
        critical_tip_mach: 0.85,
        coaxial_interference_factor: 1.0,
        rotation_direction: 1.0,
        hub_position_body_m: DVec3::new(1.5, 0.0, 0.0),
        shaft_axis_body: DVec3::X,
        radial_stations: 10,
        omega_channel: 0,
        collective_channel: None,
        cyclic_lat_channel: None,
        cyclic_lon_channel: None,
        numerics: RotorBemNumerics::DEFAULT,
    })
    .unwrap();
    let sea_level = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    let static_run = prop
        .evaluate(&LinkFlow {
            velocity_air_link_m_s: DVec3::ZERO,
            angular_rate_link_rad_s: DVec3::ZERO,
            ambient: sea_level,
            // 2400 rpm
            controls: &[251.3],
        })
        .unwrap();
    assert!(static_run.aero.force_link_n.x > 1_000.0);
    assert!(
        static_run.aero.moment_ref_link_nm.x < 0.0,
        "clockwise prop gives -x reaction roll"
    );
}
