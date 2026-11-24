//! One nondimensional free-floating flapper (body plus hinged wing panel)
//! built at insect scale and at aircraft scale with consistently scaled time.
//! Newtonian mechanics has no intrinsic scale, so nondimensional trajectories
//! must coincide.

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, JointState, Placement};

use super::rigs::{WIDE_RAD, diag, idle, joint_of, part, posed};
use super::stepping::step_rk4;
use crate::{FloatingBaseState, JointDriveInput, MultibodyError};

/// Characteristic length (m), mass (kg) and time (s).
#[derive(Clone, Copy, Debug)]
struct Scale {
    l: f64,
    m: f64,
    t: f64,
}

/// Wing-to-body mass ratio; at insect scale a 1 mg body carries a 1e-8 kg wing.
const WING_RATIO: f64 = 1.0e-2;

fn flapper(s: Scale) -> ComponentTree {
    let ml2 = s.m * s.l * s.l;
    let wm = WING_RATIO * s.m;
    ComponentTree::new([
        (
            part(
                "body",
                Placement::ROOT,
                s.m,
                diag(0.4, 0.5, 0.6) * ml2,
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "wing_root",
                Placement::child_of(ComponentId(0), DVec3::new(0.0, 0.5, 0.1) * s.l),
                0.0,
                diag(0.0, 0.0, 0.0),
                JointSpec::revolute(DVec3::X, -WIDE_RAD, WIDE_RAD),
            ),
            posed(JointState::revolute(0.2, 1.0 / s.t)),
        ),
        (
            part(
                "wing",
                Placement::child_of(ComponentId(1), DVec3::new(0.05, 1.0, 0.0) * s.l),
                wm,
                diag(0.1, 0.02, 0.1) * (wm * s.l * s.l),
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ])
    .unwrap()
}

/// Nondimensional state after `steps` steps of `0.01 T`:
/// `[q, q_dot T, x/L (3), quat (4), w T (3), v T/L (3)]`.
fn nondimensional_run(s: Scale, steps: usize) -> Result<Vec<f64>, MultibodyError> {
    let mut tree = flapper(s);
    let mut base = FloatingBaseState {
        position_world_m: DVec3::new(0.3, -0.1, 0.2) * s.l,
        rotation_world_from_root: DQuat::from_rotation_y(0.2),
        linear_vel_world_m_s: DQuat::from_rotation_y(0.2)
            * DVec3::new(0.1, 0.0, 0.05)
            * (s.l / s.t),
        angular_vel_root_rad_s: DVec3::new(0.1, 0.2, 0.3) / s.t,
    };
    let torque = 2.0e-3 * s.m * s.l * s.l / (s.t * s.t);
    let drive = [JointDriveInput::scalar(ComponentId(1), torque)];
    for _ in 0..steps {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &drive, &[], 0.01 * s.t)?;
    }
    let j = joint_of(&tree, 1);
    let x = base.position_world_m / s.l;
    let q = base.rotation_world_from_root;
    let w = base.angular_vel_root_rad_s * s.t;
    let v = base.linear_vel_world_m_s * (s.t / s.l);
    Ok(vec![
        j.q_scalar,
        j.q_vel_scalar * s.t,
        x.x,
        x.y,
        x.z,
        q.x,
        q.y,
        q.z,
        q.w,
        w.x,
        w.y,
        w.z,
        v.x,
        v.y,
        v.z,
    ])
}

#[test]
fn flapper_trajectory_is_scale_invariant() {
    let insect = Scale {
        l: 1.0e-3,
        m: 1.0e-6,
        t: 1.0e-3,
    };
    let aircraft = Scale {
        l: 10.0,
        m: 1.0e3,
        t: 1.0,
    };
    let big = nondimensional_run(aircraft, 300).unwrap();
    let small = match nondimensional_run(insect, 300) {
        Ok(state) => state,
        Err(err) => panic!("insect-scale flapper ({insect:?}) failed to step: {err:?}"),
    };
    // Both runs perform the same nondimensional arithmetic; the scale factors
    // are not powers of two, so they differ only by rounding (~1e-16 per
    // operation) over 300 steps of a non-chaotic motion. 1e-9 of the O(1)
    // nondimensional state is a rounding-only bound.
    for (k, (a, b)) in small.iter().zip(&big).enumerate() {
        assert!(
            (a - b).abs() < 1e-9 * a.abs().max(b.abs()).max(1.0),
            "component {k}: insect {a} vs aircraft {b}"
        );
    }
    // The wing must actually have moved under its drive at both scales.
    assert!(
        (big[1] - 1.0).abs() > 1e-3,
        "wing rate unchanged: {}",
        big[1]
    );
}
