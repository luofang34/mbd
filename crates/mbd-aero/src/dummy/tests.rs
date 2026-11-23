use glam::DVec3;
use mbd_core::AmbientGas;

use super::*;

#[test]
fn human_dummy_reaches_realistic_subsonic_terminal_velocity() {
    let dummy = HumanDummyAero::new(HumanDummyParams {
        frontal_area_m2: 0.65,
        reference_length_m: 1.75,
        drag_coeff: 1.15,
        damping_coeff: -0.4,
        reference_point_body_m: DVec3::ZERO,
    })
    .unwrap();
    let sea_level = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    // ~95 kg suited cosmonaut => weight ~ 932 N; terminal speed ~ 45 m/s.
    let w = dummy
        .evaluate(&LinkFlow {
            velocity_air_link_m_s: DVec3::new(0.0, 0.0, 45.0),
            angular_rate_link_rad_s: DVec3::X * 0.5,
            ambient: sea_level,
            controls: &[],
        })
        .unwrap();
    assert!((800.0..1_100.0).contains(&(-w.force_link_n.z)));
    assert!(w.moment_ref_link_nm.x < 0.0);
}
