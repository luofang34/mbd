//! Non-leaf spherical pendulum (ball-jointed body plus a rigidly fixed tip)
//! on the quasi-fixed ground, against Euler's equations about the fixed pivot.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, JointSpec, JointState, Placement,
    WrenchAtSite,
};

use super::rigs::{
    GROUND_KG, OPEN_CONE_RAD, diag, ground, ground_support, idle, part, posed, rel_err,
};
use crate::{FloatingBaseState, MultiBodyDynamics};

const BODY_KG: f64 = 3.0;
const TIP_KG: f64 = 1.5;
const TIP_AT: DVec3 = DVec3::new(0.6, -0.9, 0.2);

fn body_inertia() -> DMat3 {
    diag(1.0, 2.0, 3.0)
}

fn tip_inertia() -> DMat3 {
    diag(0.02, 0.03, 0.04)
}

fn gravity() -> DVec3 {
    DVec3::new(0.0, -9.81, 0.0)
}

/// Ball-jointed body plus tip, either as a fixed tip link (`split`, the
/// non-leaf case) or as one leaf carrying the tip as a distributed mass.
fn pendulum(q: DQuat, w: DVec3, split: bool) -> (ComponentTree, Vec<DynamicMassContribution>) {
    let mut items = vec![
        ground(),
        (
            part(
                "ball",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                BODY_KG,
                body_inertia(),
                JointSpec::spherical(OPEN_CONE_RAD),
            ),
            posed(JointState::spherical(q, w)),
        ),
    ];
    if !split {
        let tip = DynamicMassContribution {
            component: ComponentId(1),
            mass_kg: TIP_KG,
            cg_local_m: TIP_AT,
            inertia_cg_local: tip_inertia(),
        };
        return (ComponentTree::new(items).unwrap(), vec![tip]);
    }
    items.push((
        part(
            "tip",
            Placement::child_of(ComponentId(1), TIP_AT),
            TIP_KG,
            tip_inertia(),
            JointSpec::FIXED,
        ),
        idle(),
    ));
    (ComponentTree::new(items).unwrap(), Vec::new())
}

/// `I_O alpha = r x m f - w x I_O w` about the pivot, in parent axes, where
/// `f` is the uniform specific force in the pivot's (non-rotating) frame:
/// gravity for a still pivot, minus the pivot acceleration for a pushed one.
fn euler_about_pivot(q: DQuat, w: DVec3, specific_force: DVec3) -> DVec3 {
    let c = TIP_AT;
    let transfer = DMat3::from_diagonal(DVec3::splat(c.length_squared()))
        - DMat3::from_cols(c * c.x, c * c.y, c * c.z);
    let i_body = body_inertia() + tip_inertia() + transfer * TIP_KG;
    let r = DMat3::from_quat(q);
    let i_parent = r * i_body * r.transpose();
    let torque = (q * c).cross(specific_force * TIP_KG);
    i_parent.inverse() * (torque - w.cross(i_parent * w))
}

fn cases() -> [(DQuat, DVec3); 3] {
    [
        (DQuat::IDENTITY, DVec3::ZERO),
        (
            DQuat::from_euler(glam::EulerRot::XYZ, 0.4, -0.3, 0.9),
            DVec3::ZERO,
        ),
        (
            DQuat::from_euler(glam::EulerRot::XYZ, -0.2, 0.8, 0.1),
            DVec3::new(0.6, -1.2, 0.4),
        ),
    ]
}

#[test]
fn spherical_non_leaf_on_still_pivot_matches_euler_about_pivot() {
    for (q, w) in cases() {
        let (tree, _) = pendulum(q, w, true);
        let base = FloatingBaseState::AT_REST;
        let out = MultiBodyDynamics::forward_dynamics(
            &tree,
            &[],
            &base,
            gravity(),
            &[],
            &[ground_support(&base, gravity())],
        )
        .unwrap();
        let observed = out.joint_accels[0].angular_accel_rad_s2;
        let expected = euler_about_pivot(q, w, gravity());
        // Still pivot: relative and absolute rates coincide; the ground
        // reacts at 1e-12, so 1e-10 relative to g / |TIP_AT| is rounding only.
        let rel = rel_err(observed, expected, 9.81 / TIP_AT.length());
        assert!(
            rel < 1e-10,
            "q {q:?} w {w:?}: {observed:?} vs Euler about pivot {expected:?} (rel {rel:e})"
        );
    }
}

/// Pivot acceleration imposed by pushing the ground at its centre.
const PIVOT_ACCEL: DVec3 = DVec3::new(2.0, -1.0, 3.0);

/// Zero gravity; the ground is pushed so the pivot accelerates at
/// `PIVOT_ACCEL` without rotating. Returns `(observed, expected, rel)` worst case.
fn pushed_pivot_worst(split: bool) -> (DVec3, DVec3, f64) {
    let push = [WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: PIVOT_ACCEL * GROUND_KG,
        torque_local_nm: DVec3::ZERO,
    }];
    let mut worst = (DVec3::ZERO, DVec3::ZERO, 0.0);
    for (q, w) in cases() {
        let (tree, extra) = pendulum(q, w, split);
        let base = FloatingBaseState::AT_REST;
        let out =
            MultiBodyDynamics::forward_dynamics(&tree, &extra, &base, DVec3::ZERO, &[], &push)
                .unwrap();
        let observed = out.joint_accels[0].angular_accel_rad_s2;
        let expected = euler_about_pivot(q, w, -PIVOT_ACCEL);
        let rel = rel_err(observed, expected, PIVOT_ACCEL.length() / TIP_AT.length());
        if rel >= worst.2 {
            worst = (observed, expected, rel);
        }
    }
    worst
}

// Ground reaction ~1e-12 and rounding: 1e-10 relative, as for the still pivot.
#[test]
fn spherical_leaf_on_accelerating_pivot_matches_euler_about_pivot() {
    let (observed, expected, rel) = pushed_pivot_worst(false);
    assert!(
        rel < 1e-10,
        "leaf {observed:?} vs {expected:?} (rel {rel:e})"
    );
}

#[test]
fn spherical_non_leaf_on_accelerating_pivot_matches_euler_about_pivot() {
    let (observed, expected, rel) = pushed_pivot_worst(true);
    assert!(
        rel < 1e-10,
        "non-leaf {observed:?} vs {expected:?} (rel {rel:e})"
    );
}
