use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement, PowerRailState,
};

use super::{
    CgTrimController, FeedAvailability, FeedRegime, FluidNetwork, FluidSpecies, SloshModeSpec,
    TankGeometry, TankId, TankSpec, TankState, TankStepInput, TransferLineSpec, TransferLineState,
    step_tank,
};
use crate::error::SystemsError;

fn make_two_node_tree() -> (ComponentTree, ComponentId, ComponentId) {
    let root = ComponentId(0);
    let tree = ComponentTree::new([
        (
            ComponentSpec {
                name: "fuselage".into(),
                placement: Placement::ROOT,
                dry_mass_kg: 10_000.0,
                dry_inertia_body: DMat3::IDENTITY * 1000.0,
                thermal_capacity_j_k: 500_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState::at_temperature(293.15),
        ),
        (
            ComponentSpec {
                name: "booster".into(),
                placement: Placement::child_of(root, DVec3::new(-10.0, 0.0, 0.0)),
                dry_mass_kg: 2_000.0,
                dry_inertia_body: DMat3::IDENTITY * 200.0,
                thermal_capacity_j_k: 100_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    (tree, root, ComponentId(1))
}

#[test]
fn settled_tank_unifies_aircraft_negative_g_and_rocket_ullage_settling() {
    let spec = TankSpec {
        id: TankId(1),
        component: ComponentId(0),
        center_local_m: DVec3::ZERO,
        geometry: TankGeometry::Cylinder {
            radius_m: 1.0,
            length_m: 4.0,
            axis_local: DVec3::X,
        },
        species: FluidSpecies::LiquidHydrogen,
        fluid_density_kg_m3: 70.85,
        capacity_mass_kg: 1000.0,
        minimum_unusable_kg: 10.0,
        feed_regime: FeedRegime::Settled {
            pickup_axis_local: DVec3::NEG_X,
            min_settling_accel_m_s2: 0.5,
            settle_time_s: 2.0,
        },
        slosh: None,
        passive_heat_leak_w: 0.0,
    };
    let mut state = TankState::nominal(500.0, 200_000.0, 20.3);

    // Zero-G coast leaves propellant unsettled -> UnportedOrUnsettled
    let out_coast = step_tank(
        &spec,
        &mut state,
        TankStepInput {
            dt_s: 1.0,
            specific_force_site_m_s2: DVec3::ZERO,
            heat_input_w: None,
            relief_pressure_pa: None,
        },
    )
    .unwrap();
    assert_eq!(
        out_coast.availability,
        FeedAvailability::UnportedOrUnsettled
    );

    // Forward ullage firing +1.0 m/s^2 for 1.5 s: settled toward -X, but < settle_time_s (2.0 s)
    let out_settling = step_tank(
        &spec,
        &mut state,
        TankStepInput {
            dt_s: 1.5,
            specific_force_site_m_s2: DVec3::new(1.0, 0.0, 0.0),
            heat_input_w: None,
            relief_pressure_pa: None,
        },
    )
    .unwrap();
    assert_eq!(
        out_settling.availability,
        FeedAvailability::UnportedOrUnsettled
    );

    // Another 0.6 s exceeds settle_time_s -> feed opens!
    let out_ready = step_tank(
        &spec,
        &mut state,
        TankStepInput {
            dt_s: 0.6,
            specific_force_site_m_s2: DVec3::new(1.0, 0.0, 0.0),
            heat_input_w: None,
            relief_pressure_pa: None,
        },
    )
    .unwrap();
    assert_eq!(out_ready.availability, FeedAvailability::Available);
}

fn setup_trim_network(root: ComponentId, booster: ComponentId) -> FluidNetwork {
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(10),
            component: root,
            center_local_m: DVec3::new(8.0, 0.0, 0.0),
            geometry: TankGeometry::Box {
                half_extents_m: DVec3::new(1.0, 0.8, 0.4),
            },
            species: FluidSpecies::JetA1,
            fluid_density_kg_m3: 804.0,
            capacity_mass_kg: 3000.0,
            minimum_unusable_kg: 50.0,
            feed_regime: FeedRegime::HeaderTank,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(2000.0, 101_325.0, 290.0),
    )
    .unwrap();
    net.add_tank(
        TankSpec {
            id: TankId(20),
            component: booster,
            center_local_m: DVec3::new(-12.0, 0.0, 0.0),
            geometry: TankGeometry::Box {
                half_extents_m: DVec3::new(1.0, 0.8, 0.4),
            },
            species: FluidSpecies::JetA1,
            fluid_density_kg_m3: 804.0,
            capacity_mass_kg: 3000.0,
            minimum_unusable_kg: 50.0,
            feed_regime: FeedRegime::HeaderTank,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(500.0, 101_325.0, 290.0),
    )
    .unwrap();
    net.add_transfer_line(
        TransferLineSpec {
            id: 1,
            from_tank: TankId(10),
            to_tank: TankId(20),
            max_flow_rate_kg_s: 25.0,
            power_w_per_kg_s: 120.0,
            bidirectional: true,
            requires_attachment: true,
            priority: 2,
        },
        TransferLineState::default(),
    )
    .unwrap();
    net
}

#[test]
fn fluid_network_conserves_mass_during_concorde_cg_trim_and_stops_on_separation() {
    let (mut tree, root, booster) = make_two_node_tree();
    let mut net = setup_trim_network(root, booster);

    let trim = CgTrimController {
        target_cg_coord_m: -1.5,
        deadband_m: 0.05,
        gain_kg_s_per_m: 40.0,
        max_rate_kg_s: 25.0,
    };
    let cmd = trim.compute_command(-0.8).unwrap();
    net.set_line_command(1, cmd.abs()).unwrap();

    let m0 = net.total_fluid_mass_kg();
    let out1 = net
        .step(
            2.0,
            &tree,
            |_| DVec3::new(0.0, 0.0, 9.81),
            |_| None,
            |_| PowerRailState::nominal(28.0),
        )
        .unwrap();
    assert!((net.total_fluid_mass_kg() - m0).abs() < 1e-12);
    assert!(out1.total_transferred_kg > 10.0);
    assert!(!out1.electrical_loads.is_empty());

    let contrib = net.mass_contributions().unwrap();
    let total_dyn_m: f64 = contrib.iter().map(|c| c.mass_kg).sum();
    assert!((total_dyn_m - m0).abs() < 1e-12);

    let mut b_state = *tree.state(booster).unwrap();
    b_state.attached = false;
    tree.set_state(booster, b_state).unwrap();

    let out2 = net
        .step(
            2.0,
            &tree,
            |_| DVec3::new(0.0, 0.0, 9.81),
            |_| None,
            |_| PowerRailState::nominal(28.0),
        )
        .unwrap();
    assert_eq!(out2.total_transferred_kg, 0.0);
}

#[test]
fn rejects_species_mismatch_transfer_and_steps_slosh_and_boiloff() {
    let (tree, root, _) = make_two_node_tree();
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(1),
            component: root,
            center_local_m: DVec3::ZERO,
            geometry: TankGeometry::Cylinder {
                radius_m: 1.5,
                length_m: 5.0,
                axis_local: DVec3::new(1.0, 0.2, 0.0),
            },
            species: FluidSpecies::LiquidOxygen,
            fluid_density_kg_m3: 1141.0,
            capacity_mass_kg: 10_000.0,
            minimum_unusable_kg: 10.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: Some(SloshModeSpec {
                mass_fraction: 0.3,
                frequency_1g_hz: 0.8,
                damping_ratio: 0.05,
                max_displacement_m: 0.4,
            }),
            passive_heat_leak_w: 2130.0,
        },
        TankState::nominal(5000.0, 300_000.0, 90.2),
    )
    .unwrap();
    net.add_tank(
        TankSpec {
            id: TankId(2),
            component: root,
            center_local_m: DVec3::new(2.0, 0.0, 0.0),
            geometry: TankGeometry::Sphere { radius_m: 1.0 },
            species: FluidSpecies::LiquidHydrogen,
            fluid_density_kg_m3: 70.85,
            capacity_mass_kg: 500.0,
            minimum_unusable_kg: 5.0,
            feed_regime: FeedRegime::BladderOrDiaphragm,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(200.0, 200_000.0, 20.3),
    )
    .unwrap();

    let err = net
        .add_transfer_line(
            TransferLineSpec {
                id: 99,
                from_tank: TankId(1),
                to_tank: TankId(2),
                max_flow_rate_kg_s: 1.0,
                power_w_per_kg_s: 0.0,
                bidirectional: false,
                requires_attachment: false,
                priority: 0,
            },
            TransferLineState::default(),
        )
        .unwrap_err();
    assert!(matches!(err, SystemsError::SpeciesMismatch { .. }));

    // Lateral acceleration excites slosh and boils off exact 2130 * 10 / 213_000 = 0.1 kg
    let out = net
        .step(
            10.0,
            &tree,
            |_| DVec3::new(9.81, 1.5, 0.0),
            |_| None,
            |_| PowerRailState::nominal(28.0),
        )
        .unwrap();
    assert!((out.total_boiloff_kg - 0.1).abs() < 1e-12);
    assert!(!out.slosh_wrenches.is_empty());
}

#[test]
fn fluid_network_remap_subset_and_merge_preserve_mass_and_avoid_id_collisions() {
    let (_tree, root, booster) = make_two_node_tree();
    let net = setup_trim_network(root, booster);

    let retained = net.remap_subset(&[(root, ComponentId(0))]).unwrap();
    let detached = net.remap_subset(&[(booster, ComponentId(0))]).unwrap();
    assert_eq!(retained.tanks().len(), 1);
    assert_eq!(detached.tanks().len(), 1);
    assert!(retained.transfer_lines().is_empty());
    assert!(
        (retained.total_fluid_mass_kg() + detached.total_fluid_mass_kg()
            - net.total_fluid_mass_kg())
        .abs()
            < 1e-12
    );

    let (merged, other_map) = retained
        .merge_remapped(
            &[(ComponentId(0), ComponentId(0))],
            &detached,
            &[(ComponentId(0), ComponentId(1))],
        )
        .unwrap();
    assert_eq!(merged.tanks().len(), 2);
    assert_eq!(other_map.len(), 1);
    assert_ne!(other_map.first().map(|p| p.1), Some(TankId(10)));
    assert!((merged.total_fluid_mass_kg() - net.total_fluid_mass_kg()).abs() < 1e-12);
}
