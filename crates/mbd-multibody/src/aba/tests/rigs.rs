//! Test rigs shared by the reference-dynamics test modules: component
//! builders, a quasi-fixed "ground" base held against gravity, and a stepping
//! loop that keeps that support wrench current at every RK4 stage.

use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
    WrenchAtSite,
};

use crate::{FloatingBaseState, JointDriveInput, MultibodyError, step_rk4_with_forces};

/// Mass and isotropic inertia of the quasi-fixed ground body. Every link in
/// these rigs is O(1) kg, so the ground's reaction motion is O(1e-12)
/// relative, far below every tolerance used against a fixed-base reference.
pub(super) const GROUND_KG: f64 = 1.0e12;

/// Revolute travel limits wide enough that the hard-coded penalty stop in
/// `aba/chain.rs` never engages in tests that are not about stops.
pub(super) const WIDE_RAD: f64 = 1.0e6;

/// Spherical cone at its largest legal value; runs keep the relative rotation
/// under pi so the quaternion angle never reaches the cone.
pub(super) const OPEN_CONE_RAD: f64 = core::f64::consts::PI;

pub(super) fn part(
    name: &str,
    placement: Placement,
    mass_kg: f64,
    inertia: DMat3,
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg: mass_kg,
        dry_inertia_body: inertia,
        thermal_capacity_j_k: 100.0,
        nominal_power_w: 0.0,
        joint,
    }
}

pub(super) fn idle() -> ComponentState {
    ComponentState::at_temperature(293.15)
}

pub(super) fn posed(joint: JointState) -> ComponentState {
    ComponentState { joint, ..idle() }
}

pub(super) fn diag(x: f64, y: f64, z: f64) -> DMat3 {
    DMat3::from_diagonal(DVec3::new(x, y, z))
}

/// Root body standing in for an inertial frame.
pub(super) fn ground() -> (ComponentSpec, ComponentState) {
    (
        part(
            "ground",
            Placement::ROOT,
            GROUND_KG,
            DMat3::from_diagonal(DVec3::splat(GROUND_KG)),
            JointSpec::FIXED,
        ),
        idle(),
    )
}

/// Massless revolute hinge about Z placed at `at` in `parent`.
pub(super) fn hinge_z(
    parent: u32,
    at: DVec3,
    q: f64,
    q_dot: f64,
) -> (ComponentSpec, ComponentState) {
    (
        part(
            "hinge",
            Placement::child_of(ComponentId(parent), at),
            0.0,
            DMat3::ZERO,
            JointSpec::revolute(DVec3::Z, -WIDE_RAD, WIDE_RAD),
        ),
        posed(JointState::revolute(q, q_dot)),
    )
}

/// Point mass rigidly fixed at `at` in `parent`.
pub(super) fn point_mass(parent: u32, at: DVec3, mass_kg: f64) -> (ComponentSpec, ComponentState) {
    (
        part(
            "bob",
            Placement::child_of(ComponentId(parent), at),
            mass_kg,
            DMat3::ZERO,
            JointSpec::FIXED,
        ),
        idle(),
    )
}

/// Wrench on the ground body cancelling its own weight, so uniform gravity
/// acts on the links as it would on a pendulum hanging from a fixed pivot.
pub(super) fn ground_support(base: &FloatingBaseState, gravity_world: DVec3) -> WrenchAtSite {
    let g_root = base.rotation_world_from_root.conjugate() * gravity_world;
    WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: -g_root * GROUND_KG,
        torque_local_nm: DVec3::ZERO,
    }
}

/// Steps `steps` RK4 steps of `dt_s` with the ground held against gravity.
pub(super) fn run_on_ground(
    tree: &mut ComponentTree,
    base: FloatingBaseState,
    gravity_world: DVec3,
    drives: &[JointDriveInput],
    dt_s: f64,
    steps: usize,
) -> FloatingBaseState {
    let mut state = base;
    for _ in 0..steps {
        state = step_rk4_with_forces::<MultibodyError, _>(tree, &[], &state, dt_s, |_, st, _| {
            Ok((
                gravity_world,
                drives.to_vec(),
                vec![ground_support(st, gravity_world)],
            ))
        })
        .unwrap();
    }
    state
}

pub(super) fn joint_of(tree: &ComponentTree, id: u32) -> JointState {
    tree.state(ComponentId(id)).unwrap().joint
}

/// Relative error of `observed` against `expected`, normalised by `scale`
/// when the expected value is near zero.
pub(super) fn rel_err(observed: DVec3, expected: DVec3, scale: f64) -> f64 {
    (observed - expected).length() / expected.length().max(scale)
}
