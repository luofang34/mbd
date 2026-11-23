use glam::{DMat3, DVec3};
use mbd_core::{
    AmbientGas, ComponentId, ComponentSpec, ComponentState, ComponentTree, GasComposition,
    GasSpecies, JointSpec, Placement, PowerRailState, ThermalLoad,
};

use super::{
    AtomicOxygenErosionSpec, AtomicOxygenErosionState, ThermalLinkSpec, ThermalNetwork,
    ThermalNodeSpec, ThermalStepEnv, TpsAblationSpec, TpsAblationState, apply_tps_ablation_to_tree,
    step_atomic_oxygen_erosion, step_tps_ablation,
};

fn make_capsule_tree() -> (ComponentTree, ComponentId, ComponentId) {
    let cm = ComponentId(0);
    let tree = ComponentTree::new([
        (
            ComponentSpec {
                name: "cm".into(),
                placement: Placement::ROOT,
                dry_mass_kg: 2400.0,
                dry_inertia_body: DMat3::IDENTITY * 100.0,
                thermal_capacity_j_k: 200_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState::at_temperature(293.15),
        ),
        (
            ComponentSpec {
                name: "sm_radiator".into(),
                placement: Placement::child_of(cm, DVec3::NEG_X * 2.0),
                dry_mass_kg: 1500.0,
                dry_inertia_body: DMat3::IDENTITY * 100.0,
                thermal_capacity_j_k: 150_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    (tree, cm, ComponentId(1))
}

#[test]
fn thermal_coolant_loop_stops_upon_sm_separation_and_detects_overtemp() {
    let (mut tree, cm, sm) = make_capsule_tree();
    let mut net = ThermalNetwork::new();
    net.add_node(
        ThermalNodeSpec {
            component: cm,
            heat_capacity_j_k: 10_000.0,
            radiative_area_m2: 0.1,
            emissivity: 0.1,
            solar_absorptivity: 0.2,
            solar_area_m2: 0.5,
            convective_area_m2: 0.0,
            min_operating_temp_k: 250.0,
            max_operating_temp_k: 330.0,
        },
        300.0,
    )
    .unwrap();
    net.add_node(
        ThermalNodeSpec {
            component: sm,
            heat_capacity_j_k: 50_000.0,
            radiative_area_m2: 12.0,
            emissivity: 0.9,
            solar_absorptivity: 0.1,
            solar_area_m2: 1.0,
            convective_area_m2: 0.0,
            min_operating_temp_k: 200.0,
            max_operating_temp_k: 360.0,
        },
        270.0,
    )
    .unwrap();
    net.add_link(ThermalLinkSpec {
        node_a: cm,
        node_b: sm,
        conductance_w_k: 120.0,
        pumped_loop: true,
        pump_power_w: 80.0,
        requires_attachment: true,
        priority: 1,
    })
    .unwrap();

    let loads = [ThermalLoad {
        component: cm,
        heat_rate_w: 1500.0,
    }];
    let env = ThermalStepEnv {
        dt_s: 20.0,
        ambient: AmbientGas::VACUUM,
        airspeed_m_s: 0.0,
        radiative_sink_temp_k: 4.0,
        solar_irradiance_w_m2: 1361.0,
        sunlit: false,
    };

    // With SM attached, coolant loop rejects CM heat to radiator -> CM stays bounded
    let out1 = net
        .step(&tree, &env, &loads, |_| PowerRailState::nominal(28.0))
        .unwrap();
    assert!(!out1.electrical_loads.is_empty());
    let t_attached = net.temperature_k(cm).unwrap();
    assert!(t_attached < 325.0);

    // Separate SM radiator -> CM heats up and flags overtemp
    let mut sm_state = *tree.state(sm).unwrap();
    sm_state.attached = false;
    tree.set_state(sm, sm_state).unwrap();

    let sep_env = ThermalStepEnv { dt_s: 400.0, ..env };
    let out2 = net
        .step(&tree, &sep_env, &loads, |_| PowerRailState::nominal(28.0))
        .unwrap();
    assert!(out2.overtemp_nodes.contains(&cm));
    net.sync_to_tree(&mut tree).unwrap();
    assert!((tree.state(cm).unwrap().temperature_k - net.temperature_k(cm).unwrap()).abs() < 1e-12);
}

#[test]
fn tps_ablates_on_earth_and_mars_entry_with_blackout_and_ao_erodes_kapton() {
    let tps = TpsAblationSpec {
        component: ComponentId(0),
        nose_radius_m: 1.5,
        shield_area_m2: 4.0,
        char_surface_temp_k: 1800.0,
        emissivity: 0.85,
        heat_of_ablation_j_kg: 1.5e7,
        backface_soak_fraction: 0.02,
        initial_ablator_mass_kg: 120.0,
        radiative_heating_coeff: 1.0e-7,
    };
    let mut tps_state = TpsAblationState::fresh(&tps);
    let earth_entry_gas =
        AmbientGas::from_state_and_composition(10.0, 240.0, GasComposition::earth_dry_air())
            .unwrap();
    let entry_out = step_tps_ablation(&tps, &mut tps_state, 5.0, &earth_entry_gas, 7500.0).unwrap();
    assert!(entry_out.convective_heat_flux_w_m2 > 5.0e5);
    assert!(entry_out.ablated_mass_kg > 0.0);
    assert!(entry_out.comm_blackout);
    assert!(tps_state.cumulative_ablated_kg > 0.0);

    // Also verify Martian CO2 entry uses Mars Sutton-Graves coefficient without failing
    let mars_gas =
        AmbientGas::from_state_and_composition(10.0, 200.0, GasComposition::mars_air()).unwrap();
    let mars_out = step_tps_ablation(&tps, &mut tps_state, 1.0, &mars_gas, 5500.0).unwrap();
    assert!(mars_out.convective_heat_flux_w_m2 > 0.0);

    // VLEO Atomic Oxygen surface erosion
    let ao_spec = AtomicOxygenErosionSpec {
        component: ComponentId(0),
        area_m2: 2.0,
        erosion_yield_m3_per_atom: 3.0e-30,
        coating_density_kg_m3: 1420.0,
        initial_thickness_m: 50.0e-6,
    };
    let mut ao_state = AtomicOxygenErosionState::default();
    let vleo = AmbientGas::from_partial_number_densities(
        900.0,
        &[(GasSpecies::AtomicO, 5.0e15), (GasSpecies::N2, 1.0e15)],
    )
    .unwrap();
    let ao_out =
        step_atomic_oxygen_erosion(&ao_spec, &mut ao_state, 100_000.0, &vleo, 7800.0, 1.0).unwrap();
    assert!(ao_out.ao_flux_atoms_m2_s > 1.0e19);
    assert!(ao_out.mass_lost_step_kg > 0.0);
    assert!(ao_out.remaining_fraction < 1.0);
}

#[test]
fn apply_tps_ablation_updates_tree_dry_mass_delta() {
    let (mut tree, cm, _) = make_capsule_tree();
    let base_mass = tree.mass_properties().unwrap().total_mass_kg;
    let tps = TpsAblationSpec {
        component: cm,
        nose_radius_m: 1.5,
        shield_area_m2: 4.0,
        char_surface_temp_k: 1800.0,
        emissivity: 0.85,
        heat_of_ablation_j_kg: 1.5e7,
        backface_soak_fraction: 0.02,
        initial_ablator_mass_kg: 120.0,
        radiative_heating_coeff: 1.0e-7,
    };
    let mut tps_state = TpsAblationState::fresh(&tps);
    let gas = AmbientGas::from_state_and_composition(10.0, 240.0, GasComposition::earth_dry_air())
        .unwrap();
    let _ = step_tps_ablation(&tps, &mut tps_state, 5.0, &gas, 7500.0).unwrap();
    apply_tps_ablation_to_tree(&tps, &tps_state, &mut tree).unwrap();
    let new_mass = tree.mass_properties().unwrap().total_mass_kg;
    assert!((base_mass - new_mass - tps_state.cumulative_ablated_kg).abs() < 1e-12);
}
