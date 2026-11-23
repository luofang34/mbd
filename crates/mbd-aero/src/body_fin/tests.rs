use glam::DVec3;
use mbd_core::AmbientGas;

use super::*;

fn raketa_like() -> BodyAndFinAero {
    BodyAndFinAero::new(BodyAndFinParams {
        ref_area_m2: 0.785,
        ref_length_m: 1.0,
        reference_point_body_m: DVec3::new(-4.0, 0.0, 0.0),
        nose_radius_m: 0.3,
        cd0_subsonic: 0.35,
        cd0_supersonic: 0.55,
        transonic_drag_peak: 0.45,
        cn_alpha_body: 2.2,
        pitch_damping_coeff: -2.0,
        roll_damping_coeff: -0.5,
        fin_area_m2: 0.25,
        fin_arm_m: 4.0,
        fin_cl_alpha: 3.0,
        fin_stall_rad: 0.35,
        pitch_control_channel: Some(0),
        yaw_control_channel: Some(1),
        roll_control_channel: Some(2),
    })
    .unwrap()
}

#[test]
fn transonic_drag_rise_peaks_near_mach_one() {
    let rocket = raketa_like();
    let sea_level = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    let cd_at_m = |mach: f64| {
        let speed = mach * sea_level.sound_speed_m_s;
        let w = rocket
            .evaluate(&AeroFlowState {
                velocity_air_body_m_s: DVec3::X * speed,
                angular_rate_body_rad_s: DVec3::ZERO,
                ambient: sea_level,
                control_surfaces_rad: [0.0; 8],
            })
            .unwrap();
        let q = 0.5 * sea_level.density_kg_m3 * speed * speed;
        -w.force_body_n.x / (q * rocket.params().ref_area_m2)
    };
    assert!((cd_at_m(0.5) - 0.35).abs() < 0.02);
    assert!(cd_at_m(1.05) > cd_at_m(0.5) + 0.35);
    assert!(cd_at_m(1.05) > cd_at_m(2.5));
    assert!((cd_at_m(2.5) - 0.55).abs() < 0.02);
}

#[test]
fn positive_pitch_fin_command_produces_nose_up_moment_and_stability() {
    let rocket = raketa_like();
    let air = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    let mut controls = [0.0; 8];
    controls[0] = 0.1;
    let w_cmd = rocket
        .evaluate(&AeroFlowState {
            velocity_air_body_m_s: DVec3::X * 200.0,
            angular_rate_body_rad_s: DVec3::ZERO,
            ambient: air,
            control_surfaces_rad: controls,
        })
        .unwrap();
    assert!(w_cmd.moment_ref_body_nm.y > 0.0);

    // Nose-up AoA (positive +z velocity component) yields restorative (-z) lift and nose-down moment about CG ahead of CP.
    let w_aoa = rocket
        .evaluate(&AeroFlowState {
            velocity_air_body_m_s: DVec3::new(200.0, 0.0, 20.0),
            angular_rate_body_rad_s: DVec3::ZERO,
            ambient: air,
            control_surfaces_rad: [0.0; 8],
        })
        .unwrap();
    assert!(w_aoa.force_body_n.z < 0.0);
    assert!(w_aoa.moment_about_cg(DVec3::ZERO).y < 0.0);
}
