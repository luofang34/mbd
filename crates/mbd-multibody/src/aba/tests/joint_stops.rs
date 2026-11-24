//! Revolute travel stop just overrun at two geometrically similar scales.

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, JointState, Placement};

use super::rigs::{ground, idle, part, posed};
use crate::{FloatingBaseState, JointDriveInput, MultiBodyDynamics};

const STOP_RAD: f64 = 0.5;

/// Arm of mass `m` at radius `l` on a hinge limited to +/-0.5 rad, overrun
/// by 0.01 rad and still moving outward at 0.1 rad/s.
fn overrun_arm(m: f64, l: f64) -> ComponentTree {
    ComponentTree::new([
        ground(),
        (
            part(
                "hinge",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                0.0,
                DMat3::ZERO,
                JointSpec::revolute(DVec3::Z, -STOP_RAD, STOP_RAD),
            ),
            posed(JointState::revolute(STOP_RAD + 0.01, 0.1)),
        ),
        (
            part(
                "arm",
                Placement::child_of(ComponentId(1), DVec3::X * l),
                m,
                DMat3::ZERO,
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ])
    .unwrap()
}

fn q_ddot(tree: &ComponentTree, drives: &[JointDriveInput]) -> f64 {
    let base = FloatingBaseState::AT_REST;
    MultiBodyDynamics::forward_dynamics(tree, &[], &base, DVec3::ZERO, drives, &[])
        .unwrap()
        .joint_accels[0]
        .q_accel_scalar
}

#[test]
fn revolute_stop_response_is_scale_invariant() {
    // Same angles, same angular rate, same time unit: geometric similarity
    // leaves the nondimensional problem unchanged, so q_ddot (rad/s^2) must
    // be identical at a 0.01 kg / 0.1 m arm and a 10 kg / 1 m arm. Neither
    // scale is near the solver's small-inertia cut-offs (I = 1e-4 and 10 kg m^2),
    // and both stay 1e-11 or less of the ground inertia so it does not yield.
    let small = q_ddot(&overrun_arm(0.01, 0.1), &[]);
    let large = q_ddot(&overrun_arm(10.0, 1.0), &[]);
    assert!(small < 0.0, "stop does not push back: {small}");
    // Both are pure functions of the same nondimensional state; 1e-9 relative
    // is a rounding-only bound.
    assert!(
        (small - large).abs() < 1e-9 * small.abs().max(large.abs()),
        "stop q_ddot: 0.01 kg/0.1 m arm {small} rad/s^2 vs 10 kg/1 m arm {large} rad/s^2"
    );
}
