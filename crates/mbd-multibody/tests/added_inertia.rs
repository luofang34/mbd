//! Inertia added to a link through the element hook, as an aerodynamic or
//! hydrodynamic element contributes it: it resists acceleration but has no
//! weight.

#![cfg(test)]

use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement, WrenchAtSite,
};
use mbd_multibody::{
    ElementContext, FloatingBaseState, ForceFn, Loads, MultibodyError, MultibodyModel, SpatialMat6,
    UniformGravity, Workspace,
};

const MASS_KG: f64 = 2.0;
const ADDED_KG: f64 = 3.0;
const G: f64 = 9.81;

fn entrained_fluid(
    _ctx: &ElementContext<'_, ()>,
    loads: &mut Loads<'_>,
) -> Result<(), MultibodyError> {
    let added = SpatialMat6::from_rigid_inertia(ADDED_KG, DVec3::ZERO, DMat3::ZERO);
    loads.add_inertia(ComponentId(0), added)
}

/// Linear acceleration of a free body under `elements`.
fn linear_accel<E: mbd_multibody::ForceElement<()>>(elements: &E) -> DVec3 {
    let spec = ComponentSpec {
        name: "body".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: MASS_KG,
        dry_inertia_body: DMat3::IDENTITY,
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let model = MultibodyModel::new(&tree, &[]).unwrap();
    let state = model
        .state(&tree, &FloatingBaseState::AT_REST, elements, &())
        .unwrap();
    let mut work = Workspace::new();
    let mut rates = vec![0.0; state.as_slice().len()];
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            elements,
            &(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    work.base_accel().lin
}

#[test]
fn added_inertia_resists_a_force_and_carries_no_weight() {
    let push = WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: DVec3::new(10.0, 0.0, 0.0),
        torque_local_nm: DVec3::ZERO,
    };
    let gravity = UniformGravity(DVec3::new(0.0, 0.0, -G));
    let total = MASS_KG + ADDED_KG;

    let pushed = linear_accel(&(push, ForceFn(entrained_fluid)));
    assert!((pushed - DVec3::new(10.0 / total, 0.0, 0.0)).length() < 1e-13);

    // Weight acts on the body's own mass only, so the fall is slowed.
    let falling = linear_accel(&(gravity, ForceFn(entrained_fluid)));
    assert!((falling - DVec3::new(0.0, 0.0, -G * MASS_KG / total)).length() < 1e-13);

    let bare = linear_accel(&(gravity, push));
    assert!((bare - DVec3::new(10.0 / MASS_KG, 0.0, -G)).length() < 1e-13);
}
