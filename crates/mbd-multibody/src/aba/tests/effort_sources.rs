//! Actuator commands against unlimited joint loads, and the joints the solver
//! must refuse: a massless joint under effort and a moving root joint.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointActuatorSpec, JointSpec, JointState, Placement};

use super::rigs::{OPEN_CONE_RAD, WIDE_RAD, ground, idle, part, posed};
use crate::{
    FloatingBaseState, JointDriveInput, JointLoadInput, MultiBodyDynamics, MultibodyError,
};

const MAX_EFFORT_NM: f64 = 2.0;

/// 2 kg point mass at 1 m on a hinge whose actuator saturates at 2 N m:
/// inertia about the hinge is 2 kg m^2.
fn limited_arm() -> ComponentTree {
    let actuator = JointActuatorSpec {
        max_effort: MAX_EFFORT_NM,
        max_velocity: 10.0,
        damping: 0.0,
        stiffness: 0.0,
        coulomb_friction: 0.0,
        backdrivable: true,
    };
    let hinge = JointSpec::revolute(DVec3::Z, -WIDE_RAD, WIDE_RAD).with_actuator(actuator);
    ComponentTree::new([
        ground(),
        (
            part(
                "hinge",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                0.0,
                DMat3::ZERO,
                hinge,
            ),
            idle(),
        ),
        (
            part(
                "arm",
                Placement::child_of(ComponentId(1), DVec3::X),
                2.0,
                DMat3::ZERO,
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ])
    .unwrap()
}

fn q_ddot(
    tree: &ComponentTree,
    drives: &[JointDriveInput],
    loads: &[JointLoadInput],
) -> Result<f64, MultibodyError> {
    let base = FloatingBaseState::AT_REST;
    MultiBodyDynamics::forward_dynamics_with_loads(
        tree,
        &[],
        &base,
        DVec3::ZERO,
        drives,
        loads,
        &[],
    )
    .map(|out| out.joint_accels[0].q_accel_scalar)
}

#[test]
fn actuator_commands_sum_then_saturate_while_loads_add_unlimited() {
    let tree = limited_arm();
    let id = ComponentId(1);
    let drives = [
        JointDriveInput::scalar(id, 3.0),
        JointDriveInput::scalar(id, 4.0),
    ];
    // 3 + 4 N m of command saturates at 2 N m: 1 rad/s^2 on 2 kg m^2.
    let saturated = q_ddot(&tree, &drives, &[]).unwrap();
    assert!((saturated - 1.0).abs() < 1e-9, "{saturated}");
    // A 5 N m constraint load is not the actuator's command and is not
    // clipped: (2 + 5) / 2 = 3.5 rad/s^2.
    let loads = [JointLoadInput::scalar(id, 5.0)];
    let with_load = q_ddot(&tree, &drives, &loads).unwrap();
    assert!((with_load - 3.5).abs() < 1e-9, "{with_load}");
    // Loads alone sum without any limit: (5 + 5) / 2.
    let loads_only = q_ddot(&tree, &[], &[loads[0], loads[0]]).unwrap();
    assert!((loads_only - 5.0).abs() < 1e-9, "{loads_only}");
}

/// Ground plus a massless hinge carrying nothing.
fn bare_hinge() -> ComponentTree {
    ComponentTree::new([
        ground(),
        (
            part(
                "hinge",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                0.0,
                DMat3::ZERO,
                JointSpec::revolute(DVec3::Z, -WIDE_RAD, WIDE_RAD),
            ),
            idle(),
        ),
    ])
    .unwrap()
}

#[test]
fn massless_joint_is_inert_without_effort_and_singular_with_it() {
    let tree = bare_hinge();
    let id = ComponentId(1);
    assert_eq!(q_ddot(&tree, &[], &[]).unwrap(), 0.0);
    let driven = q_ddot(&tree, &[JointDriveInput::scalar(id, 1.0e-9)], &[]);
    assert_eq!(driven, Err(MultibodyError::SingularJointInertia { id }));
}

fn massless_leaf(joint: JointSpec, state: JointState) -> ComponentTree {
    let leaf = part(
        "massless",
        Placement::child_of(ComponentId(0), DVec3::ZERO),
        0.0,
        DMat3::ZERO,
        joint,
    );
    ComponentTree::new([ground(), (leaf, posed(state))]).unwrap()
}

fn solve_err(tree: &ComponentTree, drives: &[JointDriveInput]) -> Option<MultibodyError> {
    let base = FloatingBaseState::AT_REST;
    MultiBodyDynamics::forward_dynamics(tree, &[], &base, DVec3::ZERO, drives, &[]).err()
}

#[test]
fn massless_ball_and_compliant_joints_under_effort_are_singular() {
    let id = ComponentId(1);
    let ball = massless_leaf(
        JointSpec::spherical(OPEN_CONE_RAD),
        JointState::spherical(DQuat::IDENTITY, DVec3::ZERO),
    );
    assert_eq!(solve_err(&ball, &[]), None);
    let torque = JointDriveInput {
        torque_3d_nm: DVec3::new(0.0, 1.0e-9, 0.0),
        ..JointDriveInput::scalar(id, 0.0)
    };
    assert_eq!(
        solve_err(&ball, &[torque]),
        Some(MultibodyError::SingularJointInertia { id })
    );
    // A deflected spring on a massless pod is the solve that must not
    // silently freeze.
    let k = DVec3::splat(100.0);
    let spring = JointSpec::compliant_6dof(k, DVec3::ZERO, k, DVec3::ZERO);
    let rest = JointState::compliant_6dof(DVec3::ZERO, DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO);
    assert_eq!(solve_err(&massless_leaf(spring, rest), &[]), None);
    let deflected = JointState {
        translation_m: DVec3::new(0.01, 0.0, 0.0),
        ..rest
    };
    assert_eq!(
        solve_err(&massless_leaf(spring, deflected), &[]),
        Some(MultibodyError::SingularJointInertia { id })
    );
}

#[test]
fn moving_joint_on_the_root_is_refused() {
    let root = part(
        "root",
        Placement::ROOT,
        1.0,
        DMat3::IDENTITY,
        JointSpec::revolute(DVec3::Z, -1.0, 1.0),
    );
    let tree = ComponentTree::new([(root, idle())]).unwrap();
    let base = FloatingBaseState::AT_REST;
    let out = MultiBodyDynamics::forward_dynamics(&tree, &[], &base, DVec3::ZERO, &[], &[]);
    assert_eq!(
        out,
        Err(MultibodyError::NonFixedRootJoint { id: ComponentId(0) })
    );
}
