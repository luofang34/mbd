//! Free-floating base carrying an articulated arm, no gravity and no external
//! wrench: total linear and angular momentum must be conserved. Momentum is
//! computed here from joint states and poses, independently of the solver's
//! own `system_momentum` kinematics.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, JointState, Placement};

use super::rigs::{OPEN_CONE_RAD, WIDE_RAD, diag, idle, joint_of, part, posed};
use super::stepping::step_rk4;
use crate::{FloatingBaseState, JointDriveInput};

const BASE_KG: f64 = 6.0;
const ARM_KG: f64 = 2.0;
const TIP_KG: f64 = 1.0;
const SHOULDER: DVec3 = DVec3::new(1.0, 0.2, -0.1);
const TIP_AT: DVec3 = DVec3::new(0.7, 0.2, 0.1);
const HINGE_AXIS: DVec3 = DVec3::Y;

fn base_inertia() -> DMat3 {
    diag(2.0, 2.5, 3.0)
}

fn arm_inertia() -> DMat3 {
    diag(0.2, 0.3, 0.4)
}

fn tip_inertia() -> DMat3 {
    diag(0.01, 0.02, 0.03)
}

#[derive(Clone, Copy)]
enum Shoulder {
    Ball,
    Hinge,
}

fn arm(shoulder: Shoulder, with_tip: bool) -> ComponentTree {
    let (joint, state) = match shoulder {
        Shoulder::Ball => (
            JointSpec::spherical(OPEN_CONE_RAD),
            JointState::spherical(
                DQuat::from_euler(glam::EulerRot::XYZ, 0.3, 0.2, -0.4),
                DVec3::new(0.4, -0.3, 0.5),
            ),
        ),
        Shoulder::Hinge => (
            JointSpec::revolute(HINGE_AXIS, -WIDE_RAD, WIDE_RAD),
            JointState::revolute(0.3, 0.6),
        ),
    };
    let mut items = vec![
        (
            part(
                "base",
                Placement::ROOT,
                BASE_KG,
                base_inertia(),
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "arm",
                Placement::child_of(ComponentId(0), SHOULDER),
                ARM_KG,
                arm_inertia(),
                joint,
            ),
            posed(state),
        ),
    ];
    if with_tip {
        let at = Placement::child_of(ComponentId(1), TIP_AT);
        items.push((
            part("tip", at, TIP_KG, tip_inertia(), JointSpec::FIXED),
            idle(),
        ));
    }
    ComponentTree::new(items).unwrap()
}

/// `(rotation, rate)` of the arm relative to the base, in base axes.
fn shoulder_motion(tree: &ComponentTree, shoulder: Shoulder) -> (DQuat, DVec3) {
    let j = joint_of(tree, 1);
    match shoulder {
        Shoulder::Ball => (j.rotation, j.angular_vel_rad_s),
        Shoulder::Hinge => (
            DQuat::from_axis_angle(HINGE_AXIS, j.q_scalar),
            HINGE_AXIS * j.q_vel_scalar,
        ),
    }
}

/// One body's contribution `(m v, x x m v + R I R^T w)` about the world origin.
fn body_momentum(m: f64, inertia: DMat3, x: DVec3, v: DVec3, r: DQuat, w: DVec3) -> (DVec3, DVec3) {
    let rm = DMat3::from_quat(r);
    let p = v * m;
    (p, x.cross(p) + rm * inertia * rm.transpose() * w)
}

fn total_momentum(
    tree: &ComponentTree,
    base: &FloatingBaseState,
    shoulder: Shoulder,
    with_tip: bool,
) -> (DVec3, DVec3) {
    let r0 = base.rotation_world_from_root;
    let (x0, v0, w0) = (
        base.position_world_m,
        base.linear_vel_world_m_s(),
        base.angular_vel_world_rad_s(),
    );
    let (q_rel, w_rel) = shoulder_motion(tree, shoulder);
    let (ra, xa) = (r0 * q_rel, x0 + r0 * SHOULDER);
    let wa = w0 + r0 * w_rel;
    let va = v0 + w0.cross(r0 * SHOULDER);
    let (mut p, mut l) = body_momentum(BASE_KG, base_inertia(), x0, v0, r0, w0);
    let (pa, la) = body_momentum(ARM_KG, arm_inertia(), xa, va, ra, wa);
    (p, l) = (p + pa, l + la);
    if with_tip {
        let arm_to_tip = ra * TIP_AT;
        let vt = va + wa.cross(arm_to_tip);
        let (pt, lt) = body_momentum(TIP_KG, tip_inertia(), xa + arm_to_tip, vt, ra, wa);
        (p, l) = (p + pt, l + lt);
    }
    (p, l)
}

fn momentum_drift(shoulder: Shoulder, with_tip: bool) -> (f64, f64) {
    let mut tree = arm(shoulder, with_tip);
    let mut base = FloatingBaseState {
        position_world_m: DVec3::new(3.0, -1.0, 2.0),
        rotation_world_from_root: DQuat::from_rotation_x(0.3),
        linear_vel_root_m_s: DVec3::new(0.5, -0.2, 0.1),
        angular_vel_root_rad_s: DVec3::new(0.1, -0.2, 0.15),
    };
    let drive = [JointDriveInput {
        component: ComponentId(1),
        effort_scalar: 0.4,
        torque_3d_nm: DVec3::new(0.2, -0.1, 0.3),
        force_3d_n: DVec3::ZERO,
    }];
    let (p0, l0) = total_momentum(&tree, &base, shoulder, with_tip);
    for _ in 0..1000 {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &drive, &[], 1.0e-3).unwrap();
    }
    let (p1, l1) = total_momentum(&tree, &base, shoulder, with_tip);
    // Characteristic scales: total mass x 1 m/s and x 1 m^2/s, so the bound
    // stays meaningful even if a component of p0 or l0 is near zero.
    let mass = BASE_KG + ARM_KG + TIP_KG;
    (
        (p1 - p0).length() / p0.length().max(mass),
        (l1 - l0).length() / l0.length().max(mass),
    )
}

/// 1 s at 1 ms with rates below 1 rad/s: RK4 truncation is ~1e-12 per step,
/// so a 1e-8 relative drift bound is generous for a correct right-hand side.
const DRIFT: f64 = 1e-8;

#[test]
fn floating_base_with_hinged_arm_and_tip_conserves_momentum() {
    let (dp, dl) = momentum_drift(Shoulder::Hinge, true);
    assert!(dp < DRIFT && dl < DRIFT, "|dp| {dp:e}, |dL| {dl:e}");
}

#[test]
fn floating_base_with_ball_jointed_leaf_conserves_momentum() {
    let (dp, dl) = momentum_drift(Shoulder::Ball, false);
    assert!(dp < DRIFT && dl < DRIFT, "|dp| {dp:e}, |dL| {dl:e}");
}

#[test]
fn floating_base_with_ball_jointed_arm_and_tip_conserves_momentum() {
    let (dp, dl) = momentum_drift(Shoulder::Ball, true);
    assert!(dp < DRIFT && dl < DRIFT, "|dp| {dp:e}, |dL| {dl:e}");
}
