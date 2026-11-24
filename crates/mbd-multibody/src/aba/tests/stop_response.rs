//! Travel stops follow the joint's `JointStopSpec` exactly, for hinges and
//! for the spherical cone, independent of the carried inertia.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, JointState, JointStopSpec, Placement};

use super::rigs::{ground, idle, part, posed};
use crate::{FloatingBaseState, JointAccel, MultiBodyDynamics};

const STOP: JointStopSpec = JointStopSpec {
    natural_frequency_rad_s: 20.0,
    damping_ratio: 0.5,
};

fn first_joint(tree: &ComponentTree) -> JointAccel {
    let base = FloatingBaseState::AT_REST;
    MultiBodyDynamics::forward_dynamics(tree, &[], &base, DVec3::ZERO, &[], &[])
        .unwrap()
        .joint_accels[0]
}

#[test]
fn hinge_stop_demands_its_second_order_response() {
    let (over, rate) = (0.02, 0.3);
    let hinge = JointSpec::revolute(DVec3::Z, -0.5, 0.5).with_stop(STOP);
    let tree = ComponentTree::new([
        ground(),
        (
            part(
                "hinge",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                0.0,
                DMat3::ZERO,
                hinge,
            ),
            posed(JointState::revolute(0.5 + over, rate)),
        ),
        (
            part(
                "arm",
                Placement::child_of(ComponentId(1), DVec3::X * 0.7),
                3.0,
                DMat3::IDENTITY * 0.1,
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ])
    .unwrap();
    let observed = first_joint(&tree).q_accel_scalar;
    let w = STOP.natural_frequency_rad_s;
    let expected = -w * w * over - 2.0 * STOP.damping_ratio * w * rate;
    // Ground reaction ~1e-12 relative; the rest is rounding.
    assert!(
        (observed - expected).abs() < 1e-9 * expected.abs(),
        "{observed} vs {expected}"
    );
}

fn cone_overrun(scale: f64) -> DVec3 {
    let cone = 0.3;
    let tilt = DQuat::from_axis_angle(DVec3::X, cone + 0.05);
    let ball = part(
        "ball",
        Placement::child_of(ComponentId(0), DVec3::ZERO),
        scale,
        DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0) * scale),
        JointSpec::spherical(cone),
    );
    let state = posed(JointState::spherical(tilt, DVec3::new(0.2, 0.0, 0.0)));
    let tree = ComponentTree::new([ground(), (ball, state)]).unwrap();
    first_joint(&tree).angular_accel_rad_s2
}

#[test]
fn cone_stop_response_is_scale_invariant() {
    let (small, large) = (cone_overrun(1.0e-6), cone_overrun(10.0));
    assert!(small.x < 0.0, "cone stop does not push back: {small:?}");
    assert!(
        (small - large).length() < 1e-9 * large.length(),
        "{small:?} vs {large:?}"
    );
}
