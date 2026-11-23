//! One small link at a time on the quasi-fixed ground, so each absolute
//! threshold in the solver is exercised alone. Each test first checks the
//! same rig at unit scale, then at small scale where the closed form is the
//! same `effort / inertia` law.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, JointState, Placement};

use super::rigs::{OPEN_CONE_RAD, ground, hinge_z, idle, part, posed};
use crate::{FloatingBaseState, JointAccel, JointDriveInput, MultiBodyDynamics};

fn accel(tree: &ComponentTree, drive: JointDriveInput) -> JointAccel {
    let base = FloatingBaseState::AT_REST;
    MultiBodyDynamics::forward_dynamics(tree, &[], &base, DVec3::ZERO, &[drive], &[])
        .unwrap()
        .joint_accels[0]
}

/// The ground reacts at link-mass / 1e12 and the rest is rounding, so 1e-9
/// relative separates "same law" from "frozen" (relative error 1).
const TOL: f64 = 1e-9;

/// Hinged wing panel of mass `m` with its centre at `l` from the hinge;
/// inertia about the hinge is `1.1 m l^2`.
fn wing(m: f64, l: f64) -> ComponentTree {
    ComponentTree::new([
        ground(),
        hinge_z(0, DVec3::ZERO, 0.0, 0.0),
        (
            part(
                "panel",
                Placement::child_of(ComponentId(1), DVec3::X * l),
                m,
                DMat3::from_diagonal(DVec3::splat(0.1 * m * l * l)),
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ])
    .unwrap()
}

fn wing_case(m: f64, l: f64) -> (f64, f64) {
    let i_hinge = 1.1 * m * l * l;
    let torque = 0.5 * i_hinge;
    let a = accel(&wing(m, l), JointDriveInput::scalar(ComponentId(1), torque));
    (a.q_accel_scalar, torque / i_hinge)
}

#[test]
fn small_hinged_wing_accelerates_under_torque() {
    let (big, big_expected) = wing_case(1.0, 1.0);
    assert!(
        (big - big_expected).abs() < TOL * big_expected,
        "unit scale {big} vs {big_expected}"
    );
    // 1e-8 kg at 1 mm: D = 1.1e-14 kg m^2, below the solver's 1e-12 cut-off.
    let (small, expected) = wing_case(1.0e-8, 1.0e-3);
    assert!(
        (small - expected).abs() < TOL * expected,
        "1e-8 kg wing at 1 mm: q_ddot {small} rad/s^2 vs tau/I {expected} rad/s^2"
    );
}

fn ball_case(scale: f64) -> (DVec3, DVec3) {
    let inertia = DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0) * scale);
    let tree = ComponentTree::new([
        ground(),
        (
            part(
                "ball",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                scale,
                inertia,
                JointSpec::spherical(OPEN_CONE_RAD),
            ),
            posed(JointState::spherical(DQuat::IDENTITY, DVec3::ZERO)),
        ),
    ])
    .unwrap();
    let torque = DVec3::new(1.0, 2.0, 3.0) * scale;
    let drive = JointDriveInput {
        component: ComponentId(1),
        effort_scalar: 0.0,
        torque_3d_nm: torque,
        force_3d_n: DVec3::ZERO,
    };
    (
        accel(&tree, drive).angular_accel_rad_s2,
        inertia.inverse() * torque,
    )
}

#[test]
fn small_ball_jointed_body_accelerates_under_torque() {
    let (big, big_expected) = ball_case(1.0);
    assert!(
        (big - big_expected).length() < TOL * big_expected.length(),
        "{big:?}"
    );
    // Principal moments 1e-8..3e-8 kg m^2 (a ~1 g, ~5 mm body): det(aa)
    // ~6e-24, below the solver's 1e-18 cut-off.
    let (small, expected) = ball_case(1.0e-8);
    assert!(
        (small - expected).length() < TOL * expected.length(),
        "small ball: alpha {small:?} vs I^-1 tau {expected:?}"
    );
}

fn dock_case(m: f64) -> (DVec3, DVec3) {
    let (k, offset) = (m, 1.0e-3);
    let joint =
        JointSpec::compliant_6dof(DVec3::splat(k), DVec3::ZERO, DVec3::splat(k), DVec3::ZERO);
    let tree = ComponentTree::new([
        ground(),
        (
            part(
                "pod",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                m,
                DMat3::from_diagonal(DVec3::splat(m * 1.0e-6)),
                joint,
            ),
            posed(JointState::compliant_6dof(
                DVec3::new(offset, 0.0, 0.0),
                DVec3::ZERO,
                DQuat::IDENTITY,
                DVec3::ZERO,
            )),
        ),
    ])
    .unwrap();
    let a = accel(&tree, JointDriveInput::scalar(ComponentId(1), 0.0));
    (a.linear_accel_m_s2, DVec3::new(-k * offset / m, 0.0, 0.0))
}

#[test]
fn small_compliant_pod_accelerates_under_spring() {
    let (big, big_expected) = dock_case(1.0);
    assert!(
        (big - big_expected).length() < TOL * big_expected.length(),
        "{big:?}"
    );
    // 1e-8 kg pod with 1e-14 kg m^2 inertia: the 6x6 solve rejects it and the
    // compliant branch substitutes zero acceleration.
    let (small, expected) = dock_case(1.0e-8);
    assert!(
        (small - expected).length() < TOL * expected.length(),
        "1e-8 kg pod: accel {small:?} vs F/m {expected:?}"
    );
}
