use glam::DVec3;
use mbd_core::{AmbientGas, GasSpecies};

use super::*;

fn solar_panel(normal: DVec3, double_sided: bool) -> RarefiedPanelAero {
    RarefiedPanelAero::new(RarefiedPanelParams {
        area_m2: 10.0,
        normal_body: normal,
        center_body_m: DVec3::new(0.0, 4.0, 0.0),
        double_sided,
        surface_temperature_k: 310.0,
        energy_accommodation: 0.9,
        tangential_accommodation: 0.9,
    })
    .unwrap()
}

#[test]
fn hyperthermal_ram_panel_matches_classical_sentman_drag_coeff() {
    let panel = solar_panel(DVec3::X, true);
    let vleo = AmbientGas::from_partial_number_densities(
        900.0,
        &[(GasSpecies::AtomicO, 2.0e15), (GasSpecies::N2, 4.0e14)],
    )
    .unwrap();
    let v = 7_750.0;
    let w = panel
        .evaluate(&AeroFlowState {
            velocity_air_body_m_s: DVec3::X * v,
            angular_rate_body_rad_s: DVec3::ZERO,
            ambient: vleo,
            control_surfaces_rad: [0.0; 8],
        })
        .unwrap();
    let q = 0.5 * vleo.density_kg_m3 * v * v;
    let cd = -w.force_body_n.x / (q * 10.0);
    assert!((2.05..2.45).contains(&cd), "Sentman CD = {cd}");
}

#[test]
fn specular_plate_doubles_normal_drag_and_edge_on_produces_shear_drag() {
    let diffuse = solar_panel(DVec3::X, true);
    let specular = RarefiedPanelAero::new(RarefiedPanelParams {
        energy_accommodation: 0.0,
        tangential_accommodation: 0.0,
        ..*diffuse.params()
    })
    .unwrap();
    let vleo =
        AmbientGas::from_partial_number_densities(900.0, &[(GasSpecies::AtomicO, 1e15)]).unwrap();
    let ram = AeroFlowState {
        velocity_air_body_m_s: DVec3::X * 7_800.0,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient: vleo,
        control_surfaces_rad: [0.0; 8],
    };
    let f_diff = -diffuse.evaluate(&ram).unwrap().force_body_n.x;
    let f_spec = -specular.evaluate(&ram).unwrap().force_body_n.x;
    assert!(f_spec > 1.5 * f_diff);

    // Edge-on double-sided plate feels finite shear drag from diffuse thermal spread.
    let edge = solar_panel(DVec3::Z, true);
    let w_edge = edge.evaluate(&ram).unwrap();
    assert!(w_edge.force_body_n.x < 0.0);
}

#[test]
fn atomic_oxygen_lowers_speed_ratio_relative_to_heavy_n2_at_same_density() {
    let edge = solar_panel(DVec3::Z, true);
    let light_o =
        AmbientGas::from_partial_number_densities(900.0, &[(GasSpecies::AtomicO, 2e15)]).unwrap();
    let heavy_n2 =
        AmbientGas::from_partial_number_densities(900.0, &[(GasSpecies::N2, 1.142e15)]).unwrap();
    let ram = |gas| AeroFlowState {
        velocity_air_body_m_s: DVec3::X * 7_500.0,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient: gas,
        control_surfaces_rad: [0.0; 8],
    };
    let q_o = 0.5 * light_o.density_kg_m3 * 7_500.0_f64.powi(2) * 10.0;
    let q_n2 = 0.5 * heavy_n2.density_kg_m3 * 7_500.0_f64.powi(2) * 10.0;
    let cd_o = -edge.evaluate(&ram(light_o)).unwrap().force_body_n.x / q_o;
    let cd_n2 = -edge.evaluate(&ram(heavy_n2)).unwrap().force_body_n.x / q_n2;
    assert!(
        cd_o > cd_n2,
        "lower molar mass O yields lower s and higher edge shear Cd ({cd_o} vs {cd_n2})"
    );
}
