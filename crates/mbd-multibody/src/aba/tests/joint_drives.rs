//! Several drive inputs addressed to one joint.

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, Placement};

use super::rigs::{WIDE_RAD, ground, idle, part};
use crate::{FloatingBaseState, JointDriveInput, MultiBodyDynamics};

fn arm() -> ComponentTree {
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

#[test]
fn two_drives_on_one_joint_sum_or_are_rejected() {
    let tree = arm();
    let base = FloatingBaseState::AT_REST;
    let id = ComponentId(1);
    let solve = |drives: &[JointDriveInput]| {
        MultiBodyDynamics::forward_dynamics(&tree, &[], &base, DVec3::ZERO, drives, &[])
    };
    let summed = solve(&[JointDriveInput::scalar(id, 7.0)])
        .unwrap()
        .joint_accels[0]
        .q_accel_scalar;
    // Two efforts on one joint are superposed torques (a motor plus a brake).
    // Either the API refuses the ambiguous input, or it must apply the sum:
    // 7 N m on 2 kg m^2 is 3.5 rad/s^2. Silently keeping one is neither.
    let split = [
        JointDriveInput::scalar(id, 3.0),
        JointDriveInput::scalar(id, 4.0),
    ];
    let Ok(out) = solve(&split) else {
        return;
    };
    let q_ddot = out.joint_accels[0].q_accel_scalar;
    assert!(
        (q_ddot - summed).abs() < 1e-12 * summed.abs(),
        "drives [3, 4] accepted, giving q_ddot {q_ddot} rad/s^2; a single 7 gives {summed} rad/s^2"
    );
}
