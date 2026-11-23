use glam::DVec3;
use mbd_core::AmbientGas;

use super::*;

#[test]
fn stowed_chute_adds_zero_drag_and_supersonic_mars_chute_scales_correctly() {
    let dgb = ParachuteCanopyAero::new(ParachuteParams {
        nominal_area_m2: 363.0, // ~21.5 m Mars Perseverance DGB chute
        cd_subsonic: 0.65,
        cd_supersonic: 0.52,
        cn_alpha: 0.4,
        riser_attach_body_m: DVec3::new(-1.5, 0.0, 0.0),
        canopy_axis_body: -DVec3::X,
        inflation_time_constant_s: 0.8,
        wake_velocity_ratio: 0.85,
        rotational_damping_coeff: 0.5,
        reference_dynamic_pressure_pa: 200.0,
        deploy_channel: Some(0),
        reefing_channel: None,
        deployed_at_start: false,
        inflation_at_start: 0.0,
    })
    .unwrap();

    let mars = AmbientGas::mars_air(250.0, 195.0).unwrap();
    let flow = LinkFlow {
        velocity_air_link_m_s: DVec3::X * (1.8 * mars.sound_speed_m_s),
        angular_rate_link_rad_s: DVec3::ZERO,
        ambient: mars,
        controls: &[],
    };

    let stowed = ParachuteState::default();
    assert_eq!(
        dgb.evaluate_state(&flow, &stowed).unwrap().force_link_n,
        DVec3::ZERO
    );
    assert_eq!(dgb.inflation_rate(&stowed, 400.0), 0.0);

    let mut st = ParachuteState {
        deployed: true,
        inflation_fraction: 0.1,
        reefing_area_fraction: 0.5,
    };
    // Explicit Euler on the filling law over 4 s.
    for _ in 0..4_000 {
        st.inflation_fraction += dgb.inflation_rate(&st, 400.0) * 1e-3;
    }
    assert!(st.inflation_fraction > 0.99);

    let reefed = dgb.evaluate_state(&flow, &st).unwrap();
    st.reefing_area_fraction = 1.0;
    let full = dgb.evaluate_state(&flow, &st).unwrap();
    assert!((full.force_link_n.x / reefed.force_link_n.x - 2.0).abs() < 1e-9);
    assert!(full.force_link_n.x < -1_000.0);
}
