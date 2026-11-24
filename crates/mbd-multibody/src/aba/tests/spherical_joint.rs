//! Spherical (ball) joint forward dynamics against Euler's rigid-body
//! equations, with a still parent, a spinning parent, and a non-leaf link.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointSpec,
    JointState, Placement, WrenchAtSite,
};

use super::rigs::{OPEN_CONE_RAD, diag, ground, joint_of, part, posed, rel_err, run_on_ground};
use crate::{FloatingBaseState, JointDriveInput, MultiBodyDynamics};

fn inertia() -> DMat3 {
    diag(1.0, 2.0, 3.0)
}

fn tilt() -> DQuat {
    DQuat::from_euler(glam::EulerRot::XYZ, 0.4, -0.3, 0.9)
}

/// Ball-jointed body with its mass centre on the joint, so the joint force
/// exerts no torque about the centre of mass and the body is torque-free.
fn ball(parent: u32, at: DVec3, q: DQuat, w_placed: DVec3) -> (ComponentSpec, ComponentState) {
    (
        part(
            "ball",
            Placement::child_of(ComponentId(parent), at),
            3.0,
            inertia(),
            JointSpec::spherical(OPEN_CONE_RAD),
        ),
        posed(JointState::spherical(q, w_placed)),
    )
}

fn spun_ground(w_root: DVec3) -> FloatingBaseState {
    FloatingBaseState {
        angular_vel_root_rad_s: w_root,
        ..FloatingBaseState::AT_REST
    }
}

/// Euler: `I alpha = -w x I w` in body axes for the absolute rate `w`.
fn euler_alpha_body(w_body: DVec3) -> DVec3 {
    inertia().inverse() * (-w_body.cross(inertia() * w_body))
}

fn joint_angular_accel(tree: &ComponentTree, base: &FloatingBaseState, torque: DVec3) -> DVec3 {
    let drive = JointDriveInput {
        component: ComponentId(1),
        effort_scalar: 0.0,
        torque_3d_nm: torque,
        force_3d_n: DVec3::ZERO,
    };
    let out =
        MultiBodyDynamics::forward_dynamics(tree, &[], base, DVec3::ZERO, &[drive], &[]).unwrap();
    out.joint_accels[0].angular_accel_rad_s2
}

#[test]
fn spherical_leaf_on_still_parent_matches_euler_equations() {
    let w_rel = DVec3::new(0.3, -0.7, 1.1);
    let tree = ComponentTree::new([ground(), ball(0, DVec3::ZERO, tilt(), w_rel)]).unwrap();
    for torque_placed in [DVec3::ZERO, DVec3::new(0.4, -0.2, 0.7)] {
        let observed = joint_angular_accel(&tree, &FloatingBaseState::AT_REST, torque_placed);
        // Euler with torque: I alpha = tau - w x I w in body axes; the drive
        // torque is given in placed axes.
        let w_body = tilt().conjugate() * w_rel;
        let tau_body = tilt().conjugate() * torque_placed;
        let expected = tilt() * (euler_alpha_body(w_body) + inertia().inverse() * tau_body);
        // Still ground: the relative and absolute rates coincide. Only rounding
        // and the 1e-12 ground reaction separate the two, so 1e-10 relative.
        let rel = rel_err(observed, expected, 1.0);
        assert!(
            rel < 1e-10,
            "tau {torque_placed:?}: {observed:?} vs Euler {expected:?} (rel {rel:e})"
        );
    }
}

/// Relative rate derivative in placed (parent) axes when the parent spins at
/// the constant `w_parent`: `R_j alpha_body - w_parent x w_abs`, the transport
/// term of differentiating in the rotating parent frame.
fn expected_relative_accel(q: DQuat, w_parent: DVec3, w_rel: DVec3) -> DVec3 {
    let w_abs = w_parent + w_rel;
    q * euler_alpha_body(q.conjugate() * w_abs) - w_parent.cross(w_abs)
}

#[test]
fn spherical_leaf_under_spinning_parent_matches_euler_equations() {
    // Parent rate deliberately not parallel to the relative rate, so the
    // transport term w_parent x w_rel is nonzero.
    let w_parent = DVec3::new(0.0, 0.0, 2.0);
    let w_rel = DVec3::new(0.5, 0.2, 0.0);
    let tree = ComponentTree::new([ground(), ball(0, DVec3::ZERO, tilt(), w_rel)]).unwrap();
    let observed = joint_angular_accel(&tree, &spun_ground(w_parent), DVec3::ZERO);
    let expected = expected_relative_accel(tilt(), w_parent, w_rel);
    // Isotropic ground spinning about a principal axis keeps a constant rate
    // to 1e-12; 1e-10 relative is a rounding-only bound.
    let rel = rel_err(observed, expected, 1.0);
    assert!(
        rel < 1e-10,
        "relative accel {observed:?} vs Euler-in-rotating-frame {expected:?} (rel {rel:e}); \
         parent transport term = {:?}",
        -w_parent.cross(w_parent + w_rel)
    );
}

/// Absolute angular momentum (world axes) and kinetic energy of the ball.
fn ball_invariants(tree: &ComponentTree, base: &FloatingBaseState) -> (DVec3, f64) {
    let j = joint_of(tree, 1);
    let w_body = j.rotation.conjugate() * (base.angular_vel_root_rad_s + j.angular_vel_rad_s);
    let l_world = base.rotation_world_from_root * (j.rotation * (inertia() * w_body));
    (l_world, 0.5 * w_body.dot(inertia() * w_body))
}

/// Returns `(|dL|/|L|, dE/E, min quaternion w)`; the cone stop can only
/// engage once `w < 0` (relative rotation past pi).
fn spin_invariant_drift(w_parent: DVec3, w_rel: DVec3) -> (f64, f64, f64) {
    let mut tree = ComponentTree::new([ground(), ball(0, DVec3::ZERO, tilt(), w_rel)]).unwrap();
    let mut base = spun_ground(w_parent);
    let (l0, e0) = ball_invariants(&tree, &base);
    let (mut dl, mut de, mut min_w): (f64, f64, f64) = (0.0, 0.0, 1.0);
    for _ in 0..100 {
        base = run_on_ground(&mut tree, base, DVec3::ZERO, &[], 1.0e-3, 10);
        let (l, e) = ball_invariants(&tree, &base);
        dl = dl.max((l - l0).length() / l0.length());
        de = de.max((e - e0).abs() / e0);
        min_w = min_w.min(joint_of(&tree, 1).rotation.w);
    }
    (dl, de, min_w)
}

#[test]
fn spherical_leaf_conserves_momentum_and_energy_on_still_parent() {
    let (dl, de, min_w) = spin_invariant_drift(DVec3::ZERO, DVec3::new(0.3, -0.7, 1.1));
    assert!(
        min_w > 0.0,
        "relative rotation passed pi; cone stop may engage"
    );
    // 1 s at dt = 1 ms with |w| ~ 1.3 rad/s: RK4 truncation ~ (w dt)^4 ~ 3e-12
    // per step; 1e-8 relative leaves margin for quaternion renormalisation.
    assert!(dl < 1e-8 && de < 1e-8, "|dL|/|L| = {dl:e}, dE/E = {de:e}");
}

#[test]
fn spherical_leaf_conserves_momentum_and_energy_under_spinning_parent() {
    let (dl, de, min_w) =
        spin_invariant_drift(DVec3::new(0.0, 0.0, 2.0), DVec3::new(0.5, 0.2, 0.0));
    assert!(
        min_w > 0.0,
        "relative rotation passed pi; cone stop may engage"
    );
    // Same truncation budget as the still-parent case: the ball is torque-free
    // about its centre, so its inertial L and its T are exact invariants.
    assert!(dl < 1e-8 && de < 1e-8, "|dL|/|L| = {dl:e}, dE/E = {de:e}");
}

const ARM_TIP: DVec3 = DVec3::new(0.6, 0.3, -0.2);

/// Light free base whose origin is pushed, carrying a ball-jointed body at
/// `(1, 0, 0)`; the body is either one leaf carrying the tip mass as a
/// distributed contribution, or the same masses split into a body plus a
/// rigidly fixed tip link (non-leaf).
fn arm_on_pushed_base(split: bool) -> (ComponentTree, Vec<DynamicMassContribution>) {
    let base = part(
        "base",
        Placement::ROOT,
        4.0,
        diag(0.5, 0.7, 0.9),
        JointSpec::FIXED,
    );
    let mut items = vec![
        (base, super::rigs::idle()),
        ball(0, DVec3::X, tilt(), DVec3::ZERO),
    ];
    let tip_inertia = diag(0.02, 0.03, 0.04);
    if split {
        items.push((
            part(
                "tip",
                Placement::child_of(ComponentId(1), ARM_TIP),
                1.5,
                tip_inertia,
                JointSpec::FIXED,
            ),
            super::rigs::idle(),
        ));
        return (ComponentTree::new(items).unwrap(), Vec::new());
    }
    let tip = DynamicMassContribution {
        component: ComponentId(1),
        mass_kg: 1.5,
        cg_local_m: ARM_TIP,
        inertia_cg_local: tip_inertia,
    };
    (ComponentTree::new(items).unwrap(), vec![tip])
}

#[test]
fn spherical_non_leaf_matches_equivalent_leaf_under_accelerating_parent() {
    let push = [WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: DVec3::new(3.0, -5.0, 2.0),
        torque_local_nm: DVec3::new(0.0, 0.0, 0.0),
    }];
    let solve = |split: bool| {
        let (tree, extra) = arm_on_pushed_base(split);
        let base = FloatingBaseState::AT_REST;
        MultiBodyDynamics::forward_dynamics(&tree, &extra, &base, DVec3::ZERO, &[], &push).unwrap()
    };
    let (leaf, split) = (solve(false), solve(true));
    // Same rigid masses at the same places: only the bookkeeping differs, so
    // the results must agree to rounding (1e-10 relative).
    let a_leaf = leaf.joint_accels[0].angular_accel_rad_s2;
    let a_split = split.joint_accels[0].angular_accel_rad_s2;
    let rel = rel_err(a_split, a_leaf, 1.0);
    assert!(
        rel < 1e-10,
        "non-leaf {a_split:?} vs leaf {a_leaf:?} (rel {rel:e})"
    );
    let base_rel = rel_err(
        split.base_linear_accel_world_m_s2,
        leaf.base_linear_accel_world_m_s2,
        1.0,
    );
    assert!(base_rel < 1e-10, "base accel differs (rel {base_rel:e})");
}
