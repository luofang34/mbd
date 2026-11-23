use glam::DVec3;
use mbd_core::{AmbientGas, ComponentId, FaultState, GasComposition, GasSpecies, PowerRailState};

use super::{
    AbepStepInput, AbepThrusterSpec, AbepThrusterState, AirbreathingEngineKind,
    AirbreathingEngineSpec, AirbreathingEngineState, AirbreathingStepInput, ElectricThrusterSpec,
    ElectricThrusterState, ElectricThrusterStepInput, LiquidRocketSpec, LiquidRocketState,
    LiquidRocketStepInput, PlumeImpingementSpec, PlumeSurfaceTarget, SolidMotorSpec,
    SolidMotorState, evaluate_plume_impingement, step_abep_thruster, step_airbreathing_engine,
    step_electric_thruster, step_liquid_rocket, step_solid_motor,
};
use crate::fluid::{
    FeedRegime, FluidNetwork, FluidSpecies, TankGeometry, TankId, TankSpec, TankState,
    TankStepInput, step_tank,
};

fn make_tank_network(species: FluidSpecies, mass_kg: f64) -> FluidNetwork {
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(1),
            component: ComponentId(0),
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Sphere { radius_m: 1.5 },
            species,
            fluid_density_kg_m3: species.nominal_density_kg_m3(),
            capacity_mass_kg: mass_kg.max(100.0) * 2.0,
            minimum_unusable_kg: 1.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(mass_kg, 250_000.0, 290.0),
    )
    .unwrap();
    net
}

#[test]
fn gemini_stuck_on_rcs_fires_without_command_until_manifold_isolated() {
    let mut net = make_tank_network(FluidSpecies::Hydrazine, 200.0);
    let spec = LiquidRocketSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::new(0.0, 1.2, 0.0),
        thrust_axis_local: DVec3::NEG_Z,
        gimbal_max_rad: 0.0,
        max_vacuum_thrust_n: 111.0,
        isp_vac_s: 290.0,
        isp_sl_s: 220.0,
        fuel_tank: Some(TankId(1)),
        oxidizer_tank: None,
        mixture_ratio_o_f: 0.0,
        min_throttle: 0.0,
        startup_tau_s: 0.0,
        shutdown_tau_s: 0.0,
        nominal_heat_to_site_w: 500.0,
    };
    let mut state = LiquidRocketState {
        throttle_cmd: 0.0,
        fault: FaultState::ShortCircuit,
        ..LiquidRocketState::default()
    };

    let out_stuck = step_liquid_rocket(
        &spec,
        &mut state,
        &mut net,
        LiquidRocketStepInput {
            dt_s: 1.0,
            ambient: AmbientGas::VACUUM,
            manifold_isolated: false,
        },
    )
    .unwrap();
    assert!((out_stuck.thrust_n - 111.0).abs() < 1e-6);
    assert!(out_stuck.fuel_consumed_kg > 0.03);

    let out_isolated = step_liquid_rocket(
        &spec,
        &mut state,
        &mut net,
        LiquidRocketStepInput {
            dt_s: 1.0,
            ambient: AmbientGas::VACUUM,
            manifold_isolated: true,
        },
    )
    .unwrap();
    assert_eq!(out_isolated.thrust_n, 0.0);
    assert_eq!(out_isolated.fuel_consumed_kg, 0.0);
}

#[test]
fn solid_motor_and_electric_thruster_respect_burnout_and_eclipse() {
    let s_spec = SolidMotorSpec {
        site: ComponentId(1),
        mount_offset_local_m: DVec3::NEG_X,
        grain_cg_local_m: DVec3::ZERO,
        thrust_axis_local: DVec3::X,
        propellant_mass_kg: 100.0,
        isp_vac_s: 260.0,
        nozzle_exit_area_m2: 0.1,
        burn_time_s: 4.0,
        tail_off_fraction: 0.2,
        grain_radius_m: 0.3,
        grain_length_m: 1.5,
    };
    let mut s_state = SolidMotorState::loaded(&s_spec);
    let s_out = step_solid_motor(&s_spec, &mut s_state, 1.0, &AmbientGas::VACUUM, true).unwrap();
    assert!(s_out.thrust_n > 50_000.0);
    assert!(s_out.burned_mass_kg > 20.0);

    let mut xe_net = make_tank_network(FluidSpecies::Xenon, 100.0);
    let e_spec = ElectricThrusterSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::NEG_X,
        thrust_axis_local: DVec3::X,
        propellant_tank: TankId(1),
        isp_s: 3000.0,
        total_efficiency: 0.65,
        max_power_w: 5000.0,
        priority: 3,
    };
    let e_state = ElectricThrusterState {
        power_throttle_cmd: 1.0,
        fault: FaultState::Nominal,
    };
    let e_sun = step_electric_thruster(
        &e_spec,
        &e_state,
        &mut xe_net,
        ElectricThrusterStepInput {
            dt_s: 1.0,
            power_rail: PowerRailState::nominal(120.0),
            allocated_power_w: 5000.0,
        },
    )
    .unwrap();
    assert!(e_sun.thrust_n > 0.2);

    let e_eclipse = step_electric_thruster(
        &e_spec,
        &e_state,
        &mut xe_net,
        ElectricThrusterStepInput {
            dt_s: 1.0,
            power_rail: PowerRailState {
                voltage_v: 0.0,
                ripple_v_rms: 0.0,
                enabled: false,
            },
            allocated_power_w: 0.0,
        },
    )
    .unwrap();
    assert_eq!(e_eclipse.thrust_n, 0.0);
    assert_eq!(e_eclipse.propellant_consumed_kg, 0.0);
}

#[test]
fn abep_collects_atomic_oxygen_and_n2() {
    let mut fluids = make_tank_network(FluidSpecies::AirCollected, 2.0);
    let abep = AbepThrusterSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::ZERO,
        thrust_axis_local: DVec3::X,
        intake_area_m2: 1.2,
        atomic_o_collection_efficiency: 0.45,
        n2_collection_efficiency: 0.30,
        isp_s: 4500.0,
        total_efficiency: 0.6,
        max_power_w: 8000.0,
        buffer_tank: Some(TankId(1)),
        priority: 1,
    };
    let vleo = AmbientGas::from_partial_number_densities(
        900.0,
        &[(GasSpecies::AtomicO, 4.0e15), (GasSpecies::N2, 1.0e15)],
    )
    .unwrap();
    let abep_out = step_abep_thruster(
        &abep,
        &AbepThrusterState {
            power_throttle_cmd: 1.0,
            fault: FaultState::Nominal,
        },
        &mut fluids,
        AbepStepInput {
            dt_s: 1.0,
            ambient: vleo,
            airspeed_m_s: 7800.0,
            intake_ram_alignment: 1.0,
            power_rail: PowerRailState::nominal(120.0),
            allocated_power_w: 8000.0,
        },
    )
    .unwrap();
    assert!(abep_out.captured_mass_kg > 0.0);
    assert!(abep_out.gross_thrust_n > 0.0);
    assert!(abep_out.intake_capture_drag_n > 0.0);
}

#[test]
fn airbreathing_burns_earth_air_and_flames_out_on_mars() {
    let mut jet_net = make_tank_network(FluidSpecies::JetA1, 5000.0);
    let jet = AirbreathingEngineSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::ZERO,
        thrust_axis_local: DVec3::X,
        kind: AirbreathingEngineKind::AfterburningTurbofan,
        sl_static_dry_thrust_n: 140_000.0,
        dry_tsfc_kg_n_s: 2.2e-5,
        reheat_thrust_multiplier: 1.25,
        reheat_tsfc_multiplier: 1.6,
        sl_max_shaft_power_w: 0.0,
        shaft_bsfc_kg_w_s: 0.0,
        max_operating_mach: 2.2,
        density_lapse_exponent: 0.75,
        min_oxidizer_mass_fraction: 0.10,
        fuel_tank: TankId(1),
        spool_tau_s: 0.0,
    };
    let mut jet_state = AirbreathingEngineState {
        throttle_cmd: 1.0,
        afterburner_cmd: 1.0,
        ..AirbreathingEngineState::default()
    };
    let earth_air =
        AmbientGas::from_state_and_composition(101_325.0, 288.15, GasComposition::earth_dry_air())
            .unwrap();
    let out_earth = step_airbreathing_engine(
        &jet,
        &mut jet_state,
        &mut jet_net,
        AirbreathingStepInput {
            dt_s: 0.5,
            ambient: earth_air,
            true_airspeed_m_s: 100.0,
            fuel_cutoff: false,
        },
    )
    .unwrap();
    assert!(out_earth.thrust_n > 150_000.0);

    let mars_co2 =
        AmbientGas::from_state_and_composition(610.0, 215.0, GasComposition::mars_air()).unwrap();
    let out_mars = step_airbreathing_engine(
        &jet,
        &mut jet_state,
        &mut jet_net,
        AirbreathingStepInput {
            dt_s: 0.5,
            ambient: mars_co2,
            true_airspeed_m_s: 100.0,
            fuel_cutoff: false,
        },
    )
    .unwrap();
    assert_eq!(out_mars.thrust_n, 0.0);
    assert!(jet_state.flamed_out);
}

#[test]
fn plume_impingement_computes_target_loads_and_ground_cushion() {
    let plume = PlumeImpingementSpec {
        nozzle_origin_m: DVec3::new(0.0, 0.0, 2.0),
        exhaust_dir: DVec3::NEG_Z,
        nozzle_radius_m: 0.25,
        cone_half_angle_rad: 0.35,
        thrust_n: 3000.0,
        exhaust_thermal_power_w: 400_000.0,
    };
    let targets = [PlumeSurfaceTarget {
        target_site: ComponentId(2),
        surface_center_m: DVec3::new(0.1, 0.0, 0.5),
        point_local_m: DVec3::ZERO,
        rotation_root_from_target: glam::DQuat::IDENTITY,
        outward_normal: DVec3::Z,
        area_m2: 1.5,
        thermal_absorptivity: 0.8,
    }];
    let res = evaluate_plume_impingement(&plume, &targets, Some(2.0)).unwrap();
    assert_eq!(res.target_wrenches.len(), 1);
    assert!(res.target_wrenches[0].force_local_n.z < 0.0);
    assert!(res.target_thermal_loads[0].heat_rate_w > 0.0);
    assert!(res.ground_cushion_force_n > 0.0);
}

#[test]
fn fluid_unporting_starves_liquid_rocket() {
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(5),
            component: ComponentId(0),
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Box {
                half_extents_m: DVec3::splat(1.0),
            },
            species: FluidSpecies::JetA1,
            fluid_density_kg_m3: 804.0,
            capacity_mass_kg: 1000.0,
            minimum_unusable_kg: 10.0,
            feed_regime: FeedRegime::Settled {
                pickup_axis_local: DVec3::NEG_Z,
                min_settling_accel_m_s2: 1.0,
                settle_time_s: 0.0,
            },
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(500.0, 101_325.0, 290.0),
    )
    .unwrap();
    let t_spec_copy = net.tank_spec(TankId(5)).unwrap().clone();
    let t_state = net.tank_state_mut(TankId(5)).unwrap();
    let _ = step_tank(
        &t_spec_copy,
        t_state,
        TankStepInput {
            dt_s: 0.1,
            specific_force_site_m_s2: DVec3::NEG_Z * 5.0,
            heat_input_w: None,
            relief_pressure_pa: None,
        },
    )
    .unwrap();

    let rocket = LiquidRocketSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::ZERO,
        thrust_axis_local: DVec3::X,
        gimbal_max_rad: 0.0,
        max_vacuum_thrust_n: 1000.0,
        isp_vac_s: 300.0,
        isp_sl_s: 280.0,
        fuel_tank: Some(TankId(5)),
        oxidizer_tank: None,
        mixture_ratio_o_f: 0.0,
        min_throttle: 0.0,
        startup_tau_s: 0.0,
        shutdown_tau_s: 0.0,
        nominal_heat_to_site_w: 0.0,
    };
    let mut r_state = LiquidRocketState {
        throttle_cmd: 1.0,
        ..LiquidRocketState::default()
    };
    let out = step_liquid_rocket(
        &rocket,
        &mut r_state,
        &mut net,
        LiquidRocketStepInput {
            dt_s: 0.1,
            ambient: AmbientGas::VACUUM,
            manifold_isolated: false,
        },
    )
    .unwrap();
    assert!(out.propellant_starved);
    assert_eq!(out.thrust_n, 0.0);
}

#[test]
fn bipropellant_oxidizer_starvation_conserves_fuel() {
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(1),
            component: ComponentId(0),
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Sphere { radius_m: 1.0 },
            species: FluidSpecies::Rp1,
            fluid_density_kg_m3: 810.0,
            capacity_mass_kg: 100.0,
            minimum_unusable_kg: 0.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(50.0, 200_000.0, 290.0),
    )
    .unwrap();
    net.add_tank(
        TankSpec {
            id: TankId(2),
            component: ComponentId(0),
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Sphere { radius_m: 1.0 },
            species: FluidSpecies::LiquidOxygen,
            fluid_density_kg_m3: 1141.0,
            capacity_mass_kg: 200.0,
            minimum_unusable_kg: 5.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState {
            availability: crate::fluid::FeedAvailability::Depleted,
            ..TankState::nominal(5.0, 200_000.0, 90.0)
        },
    )
    .unwrap();

    let engine = LiquidRocketSpec {
        site: ComponentId(0),
        mount_offset_local_m: DVec3::ZERO,
        thrust_axis_local: DVec3::X,
        gimbal_max_rad: 0.0,
        max_vacuum_thrust_n: 50_000.0,
        isp_vac_s: 311.0,
        isp_sl_s: 282.0,
        fuel_tank: Some(TankId(1)),
        oxidizer_tank: Some(TankId(2)),
        mixture_ratio_o_f: 2.5,
        min_throttle: 0.0,
        startup_tau_s: 0.0,
        shutdown_tau_s: 0.0,
        nominal_heat_to_site_w: 0.0,
    };
    let mut state = LiquidRocketState {
        throttle_cmd: 1.0,
        ..LiquidRocketState::default()
    };
    let out = step_liquid_rocket(
        &engine,
        &mut state,
        &mut net,
        LiquidRocketStepInput {
            dt_s: 1.0,
            ambient: AmbientGas::VACUUM,
            manifold_isolated: false,
        },
    )
    .unwrap();
    assert_eq!(out.thrust_n, 0.0);
    assert_eq!(out.fuel_consumed_kg, 0.0);
    assert!((net.tank_state(TankId(1)).unwrap().mass_kg - 50.0).abs() < 1e-12);
}
