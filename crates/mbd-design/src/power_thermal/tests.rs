use glam::{DMat3, DVec3};
use mbd_core::{
    AmbientGas, ComponentId, ComponentSpec, ComponentState, ComponentTree, ElectricalLoad,
    FaultState, JointSpec, Placement, ThermalLoad,
};
use mbd_systems::{
    BatterySpec, BatteryState, ElectricalBus, SolarArraySpec, SolarArrayState, ThermalNetwork,
    ThermalNodeSpec, ThermalStepEnv,
};

use super::{OrbitalPowerScenario, analyze_orbital_power_balance, analyze_thermal_envelope};

fn make_sat_tree() -> (ComponentTree, ComponentId) {
    let sat = ComponentId(0);
    let tree = ComponentTree::new([(
        ComponentSpec {
            name: "servicer".into(),
            placement: Placement::ROOT,
            dry_mass_kg: 1200.0,
            dry_inertia_body: DMat3::IDENTITY * 200.0,
            thermal_capacity_j_k: 400_000.0,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(293.15),
    )])
    .unwrap();
    (tree, sat)
}

#[test]
fn orbital_power_balance_distinguishes_positive_vs_deficit_sizing() {
    let (tree, sat) = make_sat_tree();
    let mut bus = ElectricalBus::new(28.0).unwrap();
    bus.add_solar_array(
        SolarArraySpec {
            component: sat,
            area_m2: 6.0,
            normal_local: DVec3::Z,
            efficiency: 0.25,
            packing_factor: 0.9,
            annual_degradation: 0.0,
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
            component: sat,
            capacity_wh: 2000.0,
            nominal_voltage_v: 28.0,
            max_charge_w: 1500.0,
            max_discharge_w: 1500.0,
            charge_efficiency: 0.95,
            discharge_efficiency: 0.95,
            min_soc: 0.2,
            internal_resistance_ohm: 0.01,
        },
        BatteryState {
            energy_wh: 1200.0,
            fault: FaultState::Nominal,
        },
    )
    .unwrap();

    let scenario = OrbitalPowerScenario {
        sunlit_duration_s: 3600.0,
        eclipse_duration_s: 1800.0,
        step_dt_s: 60.0,
        solar_irradiance_w_m2: 1361.0,
        sun_dir_local: [0.0, 0.0, 1.0],
        shaft_power_w: 0.0,
    };

    let nominal = [ElectricalLoad {
        component: sat,
        power_w: 600.0,
        priority: 0,
    }];
    let pos = analyze_orbital_power_balance(&bus, &tree, &nominal, &scenario).unwrap();
    assert!(pos.energy_positive);
    assert_eq!(pos.total_shed_wh, 0.0);

    let heavy = [ElectricalLoad {
        component: sat,
        power_w: 2200.0,
        priority: 0,
    }];
    let deficit = analyze_orbital_power_balance(&bus, &tree, &heavy, &scenario).unwrap();
    assert!(!deficit.energy_positive);
    assert!(deficit.net_energy_delta_wh < 0.0);
}

#[test]
fn thermal_envelope_tracks_node_bounds_and_flags_overtemp() {
    let (tree, sat) = make_sat_tree();
    let mut net = ThermalNetwork::new();
    net.add_node(
        ThermalNodeSpec {
            component: sat,
            heat_capacity_j_k: 20_000.0,
            radiative_area_m2: 2.0,
            emissivity: 0.85,
            solar_absorptivity: 0.2,
            solar_area_m2: 1.0,
            convective_area_m2: 0.0,
            min_operating_temp_k: 240.0,
            max_operating_temp_k: 340.0,
        },
        290.0,
    )
    .unwrap();

    let env = ThermalStepEnv {
        dt_s: 30.0,
        ambient: AmbientGas::VACUUM,
        airspeed_m_s: 0.0,
        radiative_sink_temp_k: 4.0,
        solar_irradiance_w_m2: 1361.0,
        sunlit: true,
    };
    let benign = analyze_thermal_envelope(
        &net,
        &tree,
        &env,
        &[ThermalLoad {
            component: sat,
            heat_rate_w: 200.0,
        }],
        10,
    )
    .unwrap();
    assert!(!benign.any_overtemp);

    let hot = analyze_thermal_envelope(
        &net,
        &tree,
        &env,
        &[ThermalLoad {
            component: sat,
            heat_rate_w: 5000.0,
        }],
        30,
    )
    .unwrap();
    assert!(hot.any_overtemp);
    assert!(hot.nodes[0].max_temp_k > 340.0);
}
