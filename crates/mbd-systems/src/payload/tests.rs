use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};

use super::{drop_payload_mass, transfer_payload_mass};
use crate::error::SystemsError;

fn make_apollo_cm_lm() -> (ComponentTree, ComponentId, ComponentId) {
    let cm = ComponentId(0);
    let tree = ComponentTree::new([
        (
            ComponentSpec {
                name: "cm".into(),
                placement: Placement::ROOT,
                dry_mass_kg: 5000.0,
                dry_inertia_body: DMat3::IDENTITY * 100.0,
                thermal_capacity_j_k: 100_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState {
                payload_mass_kg: 140.0,
                ..ComponentState::at_temperature(293.15)
            },
        ),
        (
            ComponentSpec {
                name: "lm_ascent".into(),
                placement: Placement::child_of(cm, DVec3::X * 4.0),
                dry_mass_kg: 2100.0,
                dry_inertia_body: DMat3::IDENTITY * 100.0,
                thermal_capacity_j_k: 50_000.0,
                nominal_power_w: 0.0,
                joint: JointSpec::FIXED,
            },
            ComponentState {
                payload_mass_kg: 22.0,
                ..ComponentState::at_temperature(293.15)
            },
        ),
    ])
    .unwrap();
    (tree, cm, ComponentId(1))
}

#[test]
fn transfers_apollo_lunar_samples_while_docked_and_rejects_after_undocking() {
    let (mut tree, cm, lm) = make_apollo_cm_lm();
    let total_before = tree.mass_properties().unwrap().total_mass_kg;

    let moved = transfer_payload_mass(&mut tree, lm, cm, 21.5, true).unwrap();
    assert!((moved - 21.5).abs() < 1e-12);
    let total_after = tree.mass_properties().unwrap().total_mass_kg;
    assert!((total_after - total_before).abs() < 1e-12);
    assert!((tree.state(cm).unwrap().payload_mass_kg - 161.5).abs() < 1e-12);
    assert!((tree.state(lm).unwrap().payload_mass_kg - 0.5).abs() < 1e-12);

    // Undock LM and verify payload transfer across gap is rejected
    let mut lm_state = *tree.state(lm).unwrap();
    lm_state.attached = false;
    tree.set_state(lm, lm_state).unwrap();

    let err = transfer_payload_mass(&mut tree, lm, cm, 0.5, true).unwrap_err();
    assert!(matches!(err, SystemsError::Rejected { .. }));

    let dropped = drop_payload_mass(&mut tree, cm, 10.0).unwrap();
    assert!((dropped - 10.0).abs() < 1e-12);
}
