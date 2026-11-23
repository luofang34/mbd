use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_systems::{
    FeedRegime, FluidNetwork, FluidSpecies, TankGeometry, TankId, TankSpec, TankState,
};

use super::{TankFillScenario, analyze_cg_envelope};

fn setup_concorde_trim_tanks(airframe: ComponentId) -> FluidNetwork {
    let mut net = FluidNetwork::new();
    net.add_tank(
        TankSpec {
            id: TankId(9),
            component: airframe,
            center_local_m: DVec3::new(15.0, 0.0, 0.0),
            geometry: TankGeometry::Box {
                half_extents_m: DVec3::new(2.0, 1.0, 0.6),
            },
            species: FluidSpecies::JetA1,
            fluid_density_kg_m3: 804.0,
            capacity_mass_kg: 12_000.0,
            minimum_unusable_kg: 100.0,
            feed_regime: FeedRegime::HeaderTank,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(12_000.0, 101_325.0, 290.0),
    )
    .unwrap();
    net.add_tank(
        TankSpec {
            id: TankId(11),
            component: airframe,
            center_local_m: DVec3::new(-18.0, 0.0, 0.0),
            geometry: TankGeometry::Box {
                half_extents_m: DVec3::new(2.0, 1.0, 0.6),
            },
            species: FluidSpecies::JetA1,
            fluid_density_kg_m3: 804.0,
            capacity_mass_kg: 12_000.0,
            minimum_unusable_kg: 100.0,
            feed_regime: FeedRegime::HeaderTank,
            slosh: None,
            passive_heat_leak_w: 0.0,
        },
        TankState::nominal(0.0, 101_325.0, 290.0),
    )
    .unwrap();
    net
}

#[test]
fn concorde_style_fuel_trim_shifts_cg_aft_while_preserving_static_margin() {
    let airframe = ComponentId(0);
    let tree = ComponentTree::new([(
        ComponentSpec {
            name: "concorde_airframe".into(),
            placement: Placement::ROOT,
            dry_mass_kg: 80_000.0,
            dry_inertia_body: DMat3::IDENTITY * 1_000_000.0,
            thermal_capacity_j_k: 5_000_000.0,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(293.15),
    )])
    .unwrap();

    let net = setup_concorde_trim_tanks(airframe);
    let scenarios = [
        TankFillScenario {
            name: "subsonic_fwd_cg".into(),
            fill_fractions: vec![(9, 1.0), (11, 0.0)],
            specific_force_local_m_s2: [0.0, 0.0, 9.81],
        },
        TankFillScenario {
            name: "mach2_aft_trim".into(),
            fill_fractions: vec![(9, 0.0), (11, 1.0)],
            specific_force_local_m_s2: [0.0, 0.0, 9.81],
        },
    ];

    let report = analyze_cg_envelope(&tree, &net, &scenarios, 27.66, -4.0).unwrap();
    assert_eq!(report.scenarios.len(), 2);
    assert!(report.scenarios[0].cg_body_m[0] > 0.0);
    assert!(report.scenarios[1].cg_body_m[0] < 0.0);
    assert!(report.statically_stable_all);
    assert!(report.max_cg_x_m - report.min_cg_x_m > 3.5);
}
