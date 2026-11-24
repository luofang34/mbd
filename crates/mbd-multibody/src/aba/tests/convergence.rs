//! Observed order of accuracy of the RK4 integrator: errors at dt, dt/2 and
//! dt/4 against a closed form, or Richardson differences where none exists.
//! A fourth-order method shrinks each error ~16x per halving (order ~4).

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, JointState, Placement};

use super::rigs::{
    OPEN_CONE_RAD, diag, ground, hinge_z, idle, joint_of, part, point_mass, posed, run_on_ground,
};
use super::stepping::{step_rk4, step_rk4_with_forces};
use crate::{FloatingBaseState, MultibodyError};

/// Orders from successive error ratios. Asymptotic RK4 gives 4; 3.6..4.4
/// admits the higher-order terms still visible at the coarsest step while
/// rejecting a third- or fifth-order method.
fn assert_fourth_order(label: &str, errors: &[f64]) {
    for pair in errors.windows(2) {
        let order = (pair[0] / pair[1]).log2();
        assert!(
            (3.6..=4.4).contains(&order),
            "{label}: observed order {order:.3} from errors {errors:?}"
        );
    }
    // Errors must sit far above double rounding for the ratio to mean anything.
    assert!(
        errors.iter().all(|e| *e > 1e-11),
        "{label}: errors at rounding floor {errors:?}"
    );
}

fn solo_body(inertia: glam::DMat3) -> ComponentTree {
    ComponentTree::new([(
        part("body", Placement::ROOT, 1.0, inertia, JointSpec::FIXED),
        idle(),
    )])
    .unwrap()
}

fn kepler_error(dt: f64, t_end: f64) -> f64 {
    let mut tree = solo_body(diag(1.0, 1.0, 1.0));
    let mut base = FloatingBaseState {
        position_world_m: DVec3::X,
        linear_vel_world_m_s: DVec3::Y,
        ..FloatingBaseState::AT_REST
    };
    for _ in 0..(t_end / dt).round() as usize {
        base = step_rk4_with_forces::<MultibodyError, _>(&mut tree, &[], &base, dt, |_, st, _| {
            let r = st.position_world_m;
            Ok((
                -r / (r.length_squared() * r.length()),
                Vec::new(),
                Vec::new(),
            ))
        })
        .unwrap();
    }
    (base.position_world_m - DVec3::new(t_end.cos(), t_end.sin(), 0.0)).length()
}

#[test]
fn circular_kepler_orbit_converges_at_fourth_order() {
    // mu = r = v = 1: the exact orbit is (cos t, sin t, 0).
    let errors: Vec<f64> = [0.1, 0.05, 0.025]
        .iter()
        .map(|&dt| kepler_error(dt, 2.0))
        .collect();
    assert_fourth_order("Kepler position", &errors);
}

fn pendulum_state(dt: f64, t_end: f64) -> DVec3 {
    let mut tree = ComponentTree::new([
        ground(),
        hinge_z(0, DVec3::ZERO, 1.0, 0.0),
        point_mass(1, DVec3::new(0.0, -1.0, 0.0), 1.0),
    ])
    .unwrap();
    let steps = (t_end / dt).round() as usize;
    run_on_ground(
        &mut tree,
        FloatingBaseState::AT_REST,
        DVec3::new(0.0, -9.81, 0.0),
        &[],
        dt,
        steps,
    );
    let j = joint_of(&tree, 1);
    DVec3::new(j.q_scalar, j.q_vel_scalar, 0.0)
}

/// Richardson: `|y(h) - y(h/2)|` falls 2^p per halving for an order-p method.
fn richardson<F: Fn(f64) -> DVec3>(run: F, h: f64) -> Vec<f64> {
    let ys: Vec<DVec3> = (0..4).map(|k| run(h / f64::from(1 << k))).collect();
    ys.windows(2).map(|w| (w[0] - w[1]).length()).collect()
}

#[test]
fn large_angle_pendulum_converges_at_fourth_order() {
    let errors = richardson(|dt| pendulum_state(dt, 3.0), 0.1);
    assert_fourth_order("pendulum (q, q_dot)", &errors);
}

fn spin_attitude(inertia: glam::DMat3, w0: DVec3, dt: f64, t_end: f64) -> DQuat {
    let mut tree = solo_body(inertia);
    let mut base = FloatingBaseState {
        angular_vel_root_rad_s: w0,
        ..FloatingBaseState::AT_REST
    };
    for _ in 0..(t_end / dt).round() as usize {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &[], &[], dt).unwrap();
    }
    base.rotation_world_from_root
}

#[test]
fn constant_axis_spin_attitude_converges_at_fourth_order() {
    // Spin about a principal axis keeps w constant; exact attitude is Rz(w t).
    let w = 1.5;
    let errors: Vec<f64> = [0.2, 0.1, 0.05]
        .iter()
        .map(|&dt| {
            let q = spin_attitude(diag(1.0, 2.0, 3.0), DVec3::Z * w, dt, 2.0);
            // Axis images, not `angle_between`: its acos flattens errors
            // below ~1e-8 rad to zero.
            (frame_vector(q) - frame_vector(DQuat::from_rotation_z(w * 2.0))).length()
        })
        .collect();
    assert_fourth_order("principal spin attitude", &errors);
}

/// Attitude as the images of two body axes, a chart-free comparison.
fn frame_vector(q: DQuat) -> DVec3 {
    q * DVec3::X + (q * DVec3::Y) * 2.0
}

#[test]
fn tumbling_body_attitude_converges_at_fourth_order() {
    let w0 = DVec3::new(0.8, 0.3, -0.6);
    let errors = richardson(
        |dt| frame_vector(spin_attitude(diag(1.0, 2.0, 3.0), w0, dt, 2.0)),
        0.1,
    );
    assert_fourth_order("tumbling base attitude", &errors);
}

fn ball_attitude(dt: f64, t_end: f64) -> DVec3 {
    let mut tree = ComponentTree::new([
        ground(),
        (
            part(
                "ball",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                3.0,
                diag(1.0, 2.0, 3.0),
                JointSpec::spherical(OPEN_CONE_RAD),
            ),
            posed(JointState::spherical(
                DQuat::IDENTITY,
                DVec3::new(0.8, 0.3, -0.6),
            )),
        ),
    ])
    .unwrap();
    let steps = (t_end / dt).round() as usize;
    run_on_ground(
        &mut tree,
        FloatingBaseState::AT_REST,
        DVec3::ZERO,
        &[],
        dt,
        steps,
    );
    frame_vector(joint_of(&tree, 1).rotation)
}

#[test]
fn spherical_joint_attitude_converges_at_fourth_order() {
    let errors = richardson(|dt| ball_attitude(dt, 2.0), 0.1);
    assert_fourth_order("spherical joint attitude", &errors);
}
