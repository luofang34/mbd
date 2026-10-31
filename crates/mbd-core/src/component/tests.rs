use std::f64::consts::FRAC_PI_2;

use glam::{DMat3, DQuat, DVec3};

use super::*;
use crate::MbdError;

const AMBIENT_K: f64 = 293.15;
const CORE: ComponentId = ComponentId(0);
const TANK: ComponentId = ComponentId(1);
const SITE: ComponentId = ComponentId(2);

fn isotropic(moment: f64) -> DMat3 {
    DMat3::from_diagonal(DVec3::splat(moment))
}

fn spec(name: &str, placement: Placement, dry_mass_kg: f64, inertia: DMat3) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg,
        dry_inertia_body: inertia,
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
    }
}

fn state() -> ComponentState {
    ComponentState::at_temperature(AMBIENT_K)
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "{actual} is not {expected}"
    );
}

fn assert_vec_close(actual: DVec3, expected: DVec3) {
    assert!(
        (actual - expected).length() < 1e-9,
        "{actual} is not {expected}"
    );
}

/// 100 kg core at the origin, a 50 kg tank holding 50 kg of fluid two metres
/// ahead, and a massless sensor site one metre behind.
fn tank_vehicle() -> ComponentTree {
    let tank = Placement::child_of(CORE, DVec3::new(2.0, 0.0, 0.0));
    let site = Placement::child_of(CORE, DVec3::new(-1.0, 0.0, 0.0));
    ComponentTree::new([
        (
            spec("core", Placement::ROOT, 100.0, isotropic(10.0)),
            state(),
        ),
        (
            spec("tank", tank, 50.0, isotropic(5.0)),
            ComponentState {
                fluid_mass_kg: 50.0,
                ..state()
            },
        ),
        (spec("imu", site, 0.0, DMat3::ZERO), state()),
    ])
    .unwrap()
}

#[test]
fn full_tank_matches_the_parallel_axis_theorem() {
    let mass = tank_vehicle().mass_properties().unwrap();
    assert_close(mass.total_mass_kg, 200.0);
    assert_vec_close(mass.cg_body_m, DVec3::X);
    assert_close(mass.inertia_cg_body.x_axis.x, 15.0);
    let transverse = 10.0 + 100.0 * 1.0 + 5.0 + 100.0 * 1.0;
    assert_close(mass.inertia_cg_body.y_axis.y, transverse);
    assert_close(mass.inertia_cg_body.z_axis.z, transverse);
}

#[test]
fn draining_propellant_moves_mass_cg_inertia_and_lever_arm() {
    let mut tree = tank_vehicle();
    let full_arm = tree.lever_arm_from_cg(SITE).unwrap();
    assert_vec_close(full_arm, DVec3::new(-2.0, 0.0, 0.0));

    let drained = ComponentState {
        fluid_mass_kg: 0.0,
        ..*tree.state(TANK).unwrap()
    };
    tree.set_state(TANK, drained).unwrap();

    let mass = tree.mass_properties().unwrap();
    let cg_x = 2.0 / 3.0;
    assert_close(mass.total_mass_kg, 150.0);
    assert_vec_close(mass.cg_body_m, DVec3::new(cg_x, 0.0, 0.0));
    let tank_arm = 2.0 - cg_x;
    let transverse = 15.0 + 100.0 * cg_x * cg_x + 50.0 * tank_arm * tank_arm;
    assert_close(mass.inertia_cg_body.z_axis.z, transverse);
    let arm = tree.lever_arm_from_cg(SITE).unwrap();
    assert_vec_close(arm, DVec3::new(-1.0 - cg_x, 0.0, 0.0));
}

#[test]
fn payload_counts_as_a_point_mass_at_its_component() {
    let mut tree = tank_vehicle();
    let loaded = ComponentState {
        payload_mass_kg: 100.0,
        ..*tree.state(CORE).unwrap()
    };
    tree.set_state(CORE, loaded).unwrap();
    let mass = tree.mass_properties().unwrap();
    assert_close(mass.total_mass_kg, 300.0);
    assert_vec_close(mass.cg_body_m, DVec3::new(2.0 / 3.0, 0.0, 0.0));
}

/// Core at the origin carrying a stage at `x = -3`, itself carrying an engine
/// at `x = -4`. The stage and engine weigh 50 kg each, the core 100 kg.
fn staged_vehicle() -> ComponentTree {
    let stage = Placement::child_of(CORE, DVec3::new(-3.0, 0.0, 0.0));
    let engine = Placement::child_of(ComponentId(1), DVec3::new(-1.0, 0.0, 0.0));
    ComponentTree::new([
        (
            spec("core", Placement::ROOT, 100.0, isotropic(10.0)),
            state(),
        ),
        (spec("stage", stage, 50.0, isotropic(5.0)), state()),
        (spec("engine", engine, 50.0, isotropic(1.0)), state()),
    ])
    .unwrap()
}

#[test]
fn separating_a_stage_removes_its_whole_subtree() {
    let mut tree = staged_vehicle();
    let joined = tree.mass_properties().unwrap();
    assert_close(joined.total_mass_kg, 200.0);
    assert_vec_close(joined.cg_body_m, DVec3::new(-1.75, 0.0, 0.0));
    let core_arm = tree.lever_arm_from_cg(CORE).unwrap();
    assert_vec_close(core_arm, DVec3::new(1.75, 0.0, 0.0));

    let separated = ComponentState {
        attached: false,
        ..*tree.state(ComponentId(1)).unwrap()
    };
    tree.set_state(ComponentId(1), separated).unwrap();

    let remaining = tree.mass_properties().unwrap();
    assert_close(remaining.total_mass_kg, 100.0);
    assert_vec_close(remaining.cg_body_m, DVec3::ZERO);
    assert_eq!(remaining.inertia_cg_body, isotropic(10.0));
    assert_vec_close(tree.lever_arm_from_cg(CORE).unwrap(), DVec3::ZERO);
    assert!(!tree.is_attached(ComponentId(2)).unwrap());
}

#[test]
fn a_detached_site_has_no_lever_arm_but_keeps_its_frame() {
    let mut tree = staged_vehicle();
    let separated = ComponentState {
        attached: false,
        ..*tree.state(ComponentId(1)).unwrap()
    };
    tree.set_state(ComponentId(1), separated).unwrap();
    assert_eq!(
        tree.lever_arm_from_cg(ComponentId(2)),
        Err(MbdError::DetachedComponent { id: ComponentId(2) })
    );
    let (position, _) = tree.world_transform_in_root(ComponentId(2)).unwrap();
    assert_vec_close(position, DVec3::new(-4.0, 0.0, 0.0));
}

#[test]
fn nothing_attached_is_an_error_rather_than_a_nan_cg() {
    let mut tree = staged_vehicle();
    let separated = ComponentState {
        attached: false,
        ..*tree.state(CORE).unwrap()
    };
    tree.set_state(CORE, separated).unwrap();
    assert_eq!(tree.mass_properties(), Err(MbdError::NoAttachedMass));
}

#[test]
fn transforms_compose_placement_and_articulation_down_the_chain() {
    let quarter_turn = DQuat::from_rotation_z(FRAC_PI_2);
    let gimbal = Placement {
        parent: Some(CORE),
        translation_m: DVec3::new(0.0, 0.0, 1.0),
        rotation: quarter_turn,
    };
    let nozzle = Placement::child_of(ComponentId(1), DVec3::X);
    let mut tree = ComponentTree::new([
        (spec("core", Placement::ROOT, 1.0, DMat3::ZERO), state()),
        (spec("gimbal", gimbal, 0.0, DMat3::ZERO), state()),
        (spec("nozzle", nozzle, 1.0, DMat3::ZERO), state()),
    ])
    .unwrap();
    let (position, rotation) = tree.world_transform_in_root(ComponentId(2)).unwrap();
    assert_vec_close(position, DVec3::new(0.0, 1.0, 1.0));
    assert_vec_close(rotation * DVec3::X, DVec3::Y);

    let deflected = ComponentState {
        articulation: quarter_turn,
        ..state()
    };
    tree.set_state(ComponentId(1), deflected).unwrap();
    let (position, rotation) = tree.world_transform_in_root(ComponentId(2)).unwrap();
    assert_vec_close(position, DVec3::new(-1.0, 0.0, 1.0));
    assert_vec_close(rotation * DVec3::X, -DVec3::X);
    let mass = tree.mass_properties().unwrap();
    assert_vec_close(mass.cg_body_m, DVec3::new(-0.5, 0.0, 0.5));
}

#[test]
fn dry_inertia_follows_the_components_rotation() {
    let placement = Placement {
        rotation: DQuat::from_rotation_z(FRAC_PI_2),
        ..Placement::ROOT
    };
    let inertia = DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0));
    let tree = ComponentTree::new([(spec("rotated", placement, 1.0, inertia), state())]).unwrap();
    let rotated = tree.mass_properties().unwrap().inertia_cg_body;
    assert_close(rotated.x_axis.x, 2.0);
    assert_close(rotated.y_axis.y, 1.0);
    assert_close(rotated.z_axis.z, 3.0);
}

#[test]
fn parents_may_be_listed_after_their_children() {
    let child = Placement::child_of(ComponentId(1), DVec3::X);
    let tree = ComponentTree::new([
        (spec("child", child, 1.0, DMat3::ZERO), state()),
        (spec("parent", Placement::ROOT, 1.0, DMat3::ZERO), state()),
    ])
    .unwrap();
    assert_eq!(tree.len(), 2);
    let (position, _) = tree.world_transform_in_root(CORE).unwrap();
    assert_vec_close(position, DVec3::X);
}

#[test]
fn parent_cycles_are_rejected() {
    let a = Placement::child_of(ComponentId(1), DVec3::X);
    let b = Placement::child_of(ComponentId(0), DVec3::X);
    let result = ComponentTree::new([
        (spec("a", a, 1.0, DMat3::ZERO), state()),
        (spec("b", b, 1.0, DMat3::ZERO), state()),
    ]);
    assert_eq!(result, Err(MbdError::HierarchyCycle { id: ComponentId(0) }));

    let own_parent = Placement::child_of(ComponentId(0), DVec3::X);
    let result = ComponentTree::new([(spec("loop", own_parent, 1.0, DMat3::ZERO), state())]);
    assert_eq!(result, Err(MbdError::HierarchyCycle { id: ComponentId(0) }));
}

#[test]
fn a_cycle_above_a_component_is_found_from_below_it() {
    let leaf = Placement::child_of(ComponentId(1), DVec3::X);
    let a = Placement::child_of(ComponentId(2), DVec3::X);
    let b = Placement::child_of(ComponentId(1), DVec3::X);
    let result = ComponentTree::new([
        (spec("leaf", leaf, 1.0, DMat3::ZERO), state()),
        (spec("a", a, 1.0, DMat3::ZERO), state()),
        (spec("b", b, 1.0, DMat3::ZERO), state()),
    ]);
    assert_eq!(result, Err(MbdError::HierarchyCycle { id: ComponentId(0) }));
}

#[test]
fn unknown_ids_are_reported_as_unknown() {
    let orphan = Placement::child_of(ComponentId(7), DVec3::X);
    let result = ComponentTree::new([(spec("orphan", orphan, 1.0, DMat3::ZERO), state())]);
    let unknown = MbdError::UnknownComponent { id: ComponentId(7) };
    assert_eq!(result, Err(unknown.clone()));

    let empty = ComponentTree::new([]).unwrap();
    assert!(empty.is_empty());
    assert_eq!(
        empty.world_transform_in_root(ComponentId(7)),
        Err(unknown.clone())
    );
    assert_eq!(tank_vehicle().spec(ComponentId(7)).err(), Some(unknown));
}

fn inertia_is_rejected(inertia: DMat3) -> bool {
    let result = ComponentTree::new([(spec("bad", Placement::ROOT, 1.0, inertia), state())]);
    matches!(
        result,
        Err(MbdError::InvalidComponent {
            field: "dry_inertia_body",
            ..
        })
    )
}

#[test]
fn unphysical_inertias_are_rejected() {
    let diagonal = |x, y, z| DMat3::from_diagonal(DVec3::new(x, y, z));
    assert!(inertia_is_rejected(diagonal(-1.0, 1.0, 1.0)));
    assert!(inertia_is_rejected(diagonal(1.0, 1.0, 3.0)), "triangle");
    assert!(inertia_is_rejected(diagonal(f64::NAN, 1.0, 1.0)));

    let asymmetric = DMat3::from_cols(
        DVec3::new(2.0, 0.5, 0.0),
        DVec3::new(0.0, 2.0, 0.0),
        DVec3::new(0.0, 0.0, 2.0),
    );
    assert!(inertia_is_rejected(asymmetric));

    let indefinite = DMat3::from_cols(
        DVec3::new(1.0, 0.9, 0.0),
        DVec3::new(0.9, 1.0, 0.0),
        DVec3::new(0.0, 0.0, 1.0),
    );
    assert!(inertia_is_rejected(indefinite));

    // Every 2x2 minor of the second-moment matrix passes; only its determinant
    // exposes principal moments 3.5, 1.25, 1.25.
    let determinant_only = DMat3::from_cols(
        DVec3::new(2.0, 0.75, 0.75),
        DVec3::new(0.75, 2.0, 0.75),
        DVec3::new(0.75, 0.75, 2.0),
    );
    assert!(inertia_is_rejected(determinant_only));
}

#[test]
fn physical_inertias_of_every_scale_are_accepted() {
    let diagonal = |x, y, z| DMat3::from_diagonal(DVec3::new(x, y, z));
    assert!(!inertia_is_rejected(DMat3::ZERO), "point mass");
    assert!(!inertia_is_rejected(diagonal(0.0, 2.0, 2.0)), "thin rod");
    assert!(!inertia_is_rejected(diagonal(1.0, 2.0, 3.0)), "flat plate");
    assert!(!inertia_is_rejected(diagonal(1e-7, 2e-7, 2.5e-7)), "sensor");
    let products = DMat3::from_cols(
        DVec3::new(2.0, 0.5, 0.0),
        DVec3::new(0.5, 2.0, 0.0),
        DVec3::new(0.0, 0.0, 3.0),
    );
    assert!(!inertia_is_rejected(products));
}

#[test]
fn unphysical_spec_values_are_rejected_with_their_component_name() {
    let mut negative = spec("heavy", Placement::ROOT, -1.0, DMat3::ZERO);
    let result = ComponentTree::new([(negative.clone(), state())]);
    assert!(matches!(
        result,
        Err(MbdError::InvalidComponent { name, field: "dry_mass_kg", .. }) if name == "heavy"
    ));
    negative.dry_mass_kg = 1.0;
    negative.placement.rotation = DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    assert!(ComponentTree::new([(negative, state())]).is_err());
}

#[test]
fn invalid_runtime_state_leaves_the_tree_unchanged() {
    let mut tree = tank_vehicle();
    let before = tree.clone();
    let bad_states = [
        ComponentState {
            fluid_mass_kg: -1.0,
            ..state()
        },
        ComponentState {
            temperature_k: 0.0,
            ..state()
        },
        ComponentState {
            articulation: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.5),
            ..state()
        },
    ];
    for bad in bad_states {
        assert!(tree.set_state(TANK, bad).is_err());
        assert!(tree.restore_states(&[state(), bad, state()]).is_err());
    }
    assert_eq!(tree, before);
}

#[test]
fn a_state_snapshot_restores_the_tree_exactly() {
    let mut tree = tank_vehicle();
    let snapshot = tree.states().to_vec();
    let before = tree.mass_properties().unwrap();
    let drained = ComponentState {
        fluid_mass_kg: 0.0,
        attached: false,
        ..*tree.state(TANK).unwrap()
    };
    tree.set_state(TANK, drained).unwrap();
    assert_ne!(tree.mass_properties().unwrap(), before);

    tree.restore_states(&snapshot).unwrap();
    assert_eq!(tree.mass_properties().unwrap(), before);
    assert_eq!(
        tree.restore_states(&snapshot[1..]),
        Err(MbdError::StateCountMismatch {
            expected: 3,
            actual: 2
        })
    );
}
