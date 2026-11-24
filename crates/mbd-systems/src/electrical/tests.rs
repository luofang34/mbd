use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, ElectricalLoad, FaultState,
    JointSpec, Placement,
};

use super::{
    BatterySpec, BatteryState, ElectricalBus, ElectricalEnvInput, PowerGeneratorKind,
    PowerGeneratorSpec, PowerGeneratorState, SolarArraySpec, SolarArrayState,
};
use crate::fluid::{
    FeedRegime, FluidNetwork, FluidSpecies, TankGeometry, TankId, TankSpec, TankState,
};

fn make_spacecraft_tree() -> (ComponentTree, ComponentId, ComponentId) {
    let cm = ComponentId(0);
    let tree = ComponentTree::new([
        (
            ComponentSpec {
                name: "cm".into(),
                placement: Placement::ROOT,
                dry_mass_kg: 5000.0,
                dry_inertia_body: DMat3::IDENTITY * 100.0,
                thermal_capacity_j_k: 200_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState::at_temperature(293.15),
        ),
        (
            ComponentSpec {
                name: "sm".into(),
                placement: Placement::child_of(cm, DVec3::NEG_X * 3.0),
                dry_mass_kg: 4000.0,
                dry_inertia_body: DMat3::IDENTITY * 100.0,
                thermal_capacity_j_k: 200_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    (tree, cm, ComponentId(1))
}

fn setup_solar_battery_bus(cm: ComponentId, sm: ComponentId) -> ElectricalBus {
    let mut bus = ElectricalBus::new(28.0).unwrap();
    bus.add_solar_array(
        SolarArraySpec {
            component: sm,
            area_m2: 10.0,
            normal_local: DVec3::Z,
            efficiency: 0.25,
            packing_factor: 0.9,
            annual_degradation: 0.02,
            nominal_voltage_v: 28.0,
        },
        SolarArrayState {
            deployed_fraction: 1.0,
            age_years: 0.0,
            fault: FaultState::Nominal,
        },
    )
    .unwrap();
    bus.add_battery(
        BatterySpec {
            component: cm,
            capacity_wh: 2000.0,
            nominal_voltage_v: 28.0,
            max_charge_w: 1000.0,
            max_discharge_w: 600.0,
            charge_efficiency: 0.95,
            discharge_efficiency: 0.95,
            min_soc: 0.1,
            internal_resistance_ohm: 0.02,
        },
        BatteryState {
            energy_wh: 1000.0,
            fault: FaultState::Nominal,
        },
    )
    .unwrap();
    bus
}

#[test]
fn electrical_bus_sheds_noncritical_loads_during_eclipse_and_survives_sm_staging() {
    let (mut tree, cm, sm) = make_spacecraft_tree();
    let mut bus = setup_solar_battery_bus(cm, sm);

    let loads = [
        ElectricalLoad {
            component: cm,
            power_w: 400.0,
            priority: 0,
        },
        ElectricalLoad {
            component: sm,
            power_w: 500.0,
            priority: 4,
        },
    ];

    let sun_out = bus
        .step(
            &tree,
            &loads,
            &ElectricalEnvInput {
                dt_s: 360.0,
                default_sun_dir_local: DVec3::Z,
                solar_irradiance_w_m2: 1361.0,
                sunlit: true,
                default_shaft_power_w: 0.0,
            },
            None,
            |_| DVec3::Z,
        )
        .unwrap();
    assert_eq!(sun_out.total_shed_w, 0.0);
    assert!(sun_out.total_generated_w > 2500.0);

    let ecl_out = bus
        .step(
            &tree,
            &loads,
            &ElectricalEnvInput {
                dt_s: 36.0,
                default_sun_dir_local: DVec3::Z,
                solar_irradiance_w_m2: 1361.0,
                sunlit: false,
                default_shaft_power_w: 0.0,
            },
            None,
            |_| DVec3::Z,
        )
        .unwrap();
    assert!((ecl_out.granted_powers_w[0] - 400.0).abs() < 1e-6);
    assert!((ecl_out.granted_powers_w[1] - 200.0).abs() < 1e-6);
    assert!((ecl_out.total_shed_w - 300.0).abs() < 1e-6);

    let mut sm_state = *tree.state(sm).unwrap();
    sm_state.attached = false;
    tree.set_state(sm, sm_state).unwrap();

    let sep_out = bus
        .step(
            &tree,
            &loads,
            &ElectricalEnvInput {
                dt_s: 10.0,
                default_sun_dir_local: DVec3::Z,
                solar_irradiance_w_m2: 1361.0,
                sunlit: true,
                default_shaft_power_w: 0.0,
            },
            None,
            |_| DVec3::Z,
        )
        .unwrap();
    assert_eq!(sep_out.total_generated_w, 0.0);
    assert!((sep_out.granted_powers_w[0] - 400.0).abs() < 1e-6);
    assert_eq!(sep_out.granted_powers_w[1], 0.0);
}

#[test]
fn rtg_provides_steady_power_and_waste_heat() {
    let (tree, cm, _) = make_spacecraft_tree();
    let mut bus = ElectricalBus::new(28.0).unwrap();
    bus.add_generator(
        PowerGeneratorSpec {
            component: cm,
            kind: PowerGeneratorKind::Rtg {
                bol_electrical_w: 110.0,
                bol_thermal_w: 2000.0,
                half_life_years: 87.7,
            },
            nominal_voltage_v: 28.0,
        },
        PowerGeneratorState::default(),
    )
    .unwrap();
    let out = bus
        .step(
            &tree,
            &[],
            &ElectricalEnvInput {
                dt_s: 1.0,
                default_sun_dir_local: DVec3::Z,
                solar_irradiance_w_m2: 590.0,
                sunlit: false,
                default_shaft_power_w: 0.0,
            },
            None,
            |_| DVec3::Z,
        )
        .unwrap();
    assert!((out.total_generated_w - 110.0).abs() < 1e-3);
    assert!((out.thermal_loads[0].heat_rate_w - 1890.0).abs() < 1.0);
}

#[test]
fn fuel_cell_drops_output_when_o2_tank_depletes() {
    let (tree, _cm, sm) = make_spacecraft_tree();
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(1),
            component: sm,
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Sphere { radius_m: 0.5 },
            species: FluidSpecies::LiquidHydrogen,
            fluid_density_kg_m3: 70.85,
            capacity_mass_kg: 10.0,
            minimum_unusable_kg: 0.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(5.0, 200_000.0, 20.3),
    )
    .unwrap();
    net.add_tank(
        TankSpec {
            id: TankId(2),
            component: sm,
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Sphere { radius_m: 0.5 },
            species: FluidSpecies::LiquidOxygen,
            fluid_density_kg_m3: 1141.0,
            capacity_mass_kg: 20.0,
            minimum_unusable_kg: 0.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(0.4, 200_000.0, 90.2),
    )
    .unwrap();

    let mut bus = ElectricalBus::new(28.0).unwrap();
    bus.add_generator(
        PowerGeneratorSpec {
            component: sm,
            kind: PowerGeneratorKind::FuelCell {
                max_power_w: 1800.0,
                waste_heat_ratio: 0.6,
                specific_reactant_kg_wh: 0.45,
                h2_tank: Some(TankId(1)),
                o2_tank: Some(TankId(2)),
            },
            nominal_voltage_v: 28.0,
        },
        PowerGeneratorState::default(),
    )
    .unwrap();
    let env = ElectricalEnvInput {
        dt_s: 3600.0,
        default_sun_dir_local: DVec3::Z,
        solar_irradiance_w_m2: 0.0,
        sunlit: false,
        default_shaft_power_w: 0.0,
    };
    let out1 = bus
        .step(&tree, &[], &env, Some(&mut net), |_| DVec3::Z)
        .unwrap();
    assert!(out1.total_generated_w > 0.0 && out1.total_generated_w < 1800.0);
    let out2 = bus
        .step(&tree, &[], &env, Some(&mut net), |_| DVec3::Z)
        .unwrap();
    assert_eq!(out2.total_generated_w, 0.0);
}

#[test]
fn remap_and_merge_handle_default_unpowered_and_powered_buses() {
    let default_bus = ElectricalBus::default();
    let remapped = default_bus
        .remap_subset(&[(ComponentId(0), ComponentId(0))])
        .unwrap();
    assert_eq!(remapped.nominal_bus_voltage_v(), 0.0);

    let (_, cm, sm) = make_spacecraft_tree();
    let powered = setup_solar_battery_bus(cm, sm);
    let merged = default_bus
        .merge_remapped(
            &[(ComponentId(0), ComponentId(0))],
            &powered,
            &[(cm, ComponentId(1)), (sm, ComponentId(2))],
            &[],
        )
        .unwrap();
    assert_eq!(merged.nominal_bus_voltage_v(), 28.0);
    assert_eq!(merged.batteries().len(), 1);
    assert_eq!(merged.solar_arrays().len(), 1);
}
