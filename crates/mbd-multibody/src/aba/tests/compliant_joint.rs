//! Compliant 6-DoF joint at rest with a deflected spring, on the quasi-fixed
//! ground, against Newton-Euler for the rigid body it carries. The same
//! masses are entered either as one leaf link or as a link plus a fixed tip.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, JointSpec, JointState, Placement,
};

use super::rigs::{diag, ground, idle, part, posed, rel_err};
use crate::{FloatingBaseState, MultiBodyDynamics};

const BODY_KG: f64 = 2.0;
const TIP_KG: f64 = 1.2;
const TIP_AT: DVec3 = DVec3::new(0.5, 0.25, -0.15);
const K_LIN: DVec3 = DVec3::new(200.0, 300.0, 400.0);
const K_ANG: DVec3 = DVec3::new(50.0, 60.0, 70.0);

fn body_inertia() -> DMat3 {
    diag(0.3, 0.4, 0.5)
}

fn tip_inertia() -> DMat3 {
    diag(0.02, 0.03, 0.04)
}

fn deflection() -> (DVec3, DQuat) {
    (
        DVec3::new(0.05, -0.02, 0.01),
        DQuat::from_euler(glam::EulerRot::XYZ, 0.1, -0.05, 0.08),
    )
}

fn rig(split: bool) -> (ComponentTree, Vec<DynamicMassContribution>) {
    let (t, q) = deflection();
    let joint = JointSpec::compliant_6dof(K_LIN, DVec3::splat(5.0), K_ANG, DVec3::splat(1.0));
    let mut items = vec![
        ground(),
        (
            part(
                "dock",
                Placement::child_of(ComponentId(0), DVec3::X),
                BODY_KG,
                body_inertia(),
                joint,
            ),
            posed(JointState::compliant_6dof(t, DVec3::ZERO, q, DVec3::ZERO)),
        ),
    ];
    if split {
        let tip = part(
            "tip",
            Placement::child_of(ComponentId(1), TIP_AT),
            TIP_KG,
            tip_inertia(),
            JointSpec::FIXED,
        );
        items.push((tip, idle()));
        return (ComponentTree::new(items).unwrap(), Vec::new());
    }
    let extra = DynamicMassContribution {
        component: ComponentId(1),
        mass_kg: TIP_KG,
        cg_local_m: TIP_AT,
        inertia_cg_local: tip_inertia(),
    };
    (ComponentTree::new(items).unwrap(), vec![extra])
}

fn transfer(d: DVec3) -> DMat3 {
    DMat3::from_diagonal(DVec3::splat(d.length_squared()))
        - DMat3::from_cols(d * d.x, d * d.y, d * d.z)
}

/// Newton-Euler at rest in placed axes: `a_cm = F/m`,
/// `I_cm alpha = tau - r x F`, joint-origin accel `a_cm - alpha x r`.
fn newton_euler() -> (DVec3, DVec3) {
    let (t, q) = deflection();
    let force = -K_LIN * t;
    let torque = -K_ANG * q.to_scaled_axis();
    let m = BODY_KG + TIP_KG;
    let c_cm = TIP_AT * (TIP_KG / m);
    let i_cm_body = body_inertia()
        + transfer(c_cm) * BODY_KG
        + tip_inertia()
        + transfer(TIP_AT - c_cm) * TIP_KG;
    let r = DMat3::from_quat(q);
    let i_cm = r * i_cm_body * r.transpose();
    let arm = q * c_cm;
    let alpha = i_cm.inverse() * (torque - arm.cross(force));
    (force / m - alpha.cross(arm), alpha)
}

fn solver(split: bool) -> (DVec3, DVec3) {
    let (tree, extra) = rig(split);
    let base = FloatingBaseState::AT_REST;
    let out =
        MultiBodyDynamics::forward_dynamics(&tree, &extra, &base, DVec3::ZERO, &[], &[]).unwrap();
    let j = out.joint_accels[0];
    (j.linear_accel_m_s2, j.angular_accel_rad_s2)
}

/// The ground reacts at ~1e-12 of the link accelerations and the rest is
/// rounding on O(100) numbers, so 1e-10 relative is a rounding-only bound.
const TOL: f64 = 1e-10;

#[test]
fn compliant_leaf_at_rest_matches_newton_euler() {
    let (lin, ang) = solver(false);
    let (lin_ne, ang_ne) = newton_euler();
    let (el, ea) = (rel_err(lin, lin_ne, 1.0), rel_err(ang, ang_ne, 1.0));
    assert!(
        el < TOL && ea < TOL,
        "leaf lin {lin:?} vs {lin_ne:?} (rel {el:e}); ang {ang:?} vs {ang_ne:?} (rel {ea:e})"
    );
}

#[test]
fn compliant_non_leaf_at_rest_matches_newton_euler() {
    let (lin, ang) = solver(true);
    let (lin_ne, ang_ne) = newton_euler();
    let (el, ea) = (rel_err(lin, lin_ne, 1.0), rel_err(ang, ang_ne, 1.0));
    assert!(
        el < TOL && ea < TOL,
        "non-leaf lin {lin:?} vs {lin_ne:?} (rel {el:e}); ang {ang:?} vs {ang_ne:?} (rel {ea:e})"
    );
}
