//! The operators a constraint solver is built from, against closed forms for
//! a planar double pendulum of point masses: the inverse mass matrix applied
//! by the articulated-body pass, and the Jacobian of a body-fixed point with
//! its transpose.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
};
use mbd_multibody::{FloatingBaseState, ModelState, MultibodyModel, Rk4, TimeScale, Workspace};

const L1: f64 = 1.1;
const L2: f64 = 0.7;
const M1: f64 = 1.5;
const M2: f64 = 0.9;
/// Mass of the body standing in for an inertial frame; the pendulum's
/// reaction moves it by ~1e-12 relative.
const GROUND_KG: f64 = 1.0e12;

fn part(
    parent: Option<u32>,
    at: DVec3,
    mass_kg: f64,
    inertia: DMat3,
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement: parent.map_or(Placement::ROOT, |p| Placement::child_of(ComponentId(p), at)),
        dry_mass_kg: mass_kg,
        dry_inertia_body: inertia,
        thermal_capacity_j_k: 100.0,
        nominal_power_w: 0.0,
        joint,
    }
}

fn posed(joint: JointState) -> ComponentState {
    ComponentState {
        joint,
        ..ComponentState::at_temperature(293.15)
    }
}

fn hinge(parent: u32, q: f64) -> (ComponentSpec, ComponentState) {
    let joint = JointSpec::revolute(DVec3::Z, -1.0e6, 1.0e6);
    (
        part(Some(parent), DVec3::ZERO, 0.0, DMat3::ZERO, joint),
        posed(JointState::revolute(q, 0.0)),
    )
}

fn bob(parent: u32, length: f64, mass_kg: f64) -> (ComponentSpec, ComponentState) {
    let at = DVec3::new(0.0, -length, 0.0);
    (
        part(Some(parent), at, mass_kg, DMat3::ZERO, JointSpec::FIXED),
        posed(JointState::FIXED),
    )
}

/// Ground, hinge 1, bob 1, hinge 2 (relative angle), bob 2.
fn double_pendulum(q1: f64, q2: f64) -> (MultibodyModel, ModelState) {
    let ground = part(
        None,
        DVec3::ZERO,
        GROUND_KG,
        DMat3::from_diagonal(DVec3::splat(GROUND_KG)),
        JointSpec::FIXED,
    );
    let tree = ComponentTree::new([
        (ground, posed(JointState::FIXED)),
        hinge(0, q1),
        bob(1, L1, M1),
        hinge(2, q2),
        bob(3, L2, M2),
    ])
    .unwrap();
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let state = model
        .state(&tree, &FloatingBaseState::AT_REST, &(), &())
        .unwrap();
    (model, state)
}

/// Generalized index of hinge 1 and hinge 2: six base entries come first.
const HINGE_1: usize = 6;
const HINGE_2: usize = 7;

/// Inverse of the joint-space mass matrix in relative coordinates,
/// `[[a, b], [b, c]]⁻¹`, from the Lagrangian of two point masses.
fn inverse_mass_closed_form(q2: f64) -> [[f64; 2]; 2] {
    let a = (M1 + M2) * L1 * L1 + M2 * L2 * L2 + 2.0 * M2 * L1 * L2 * q2.cos();
    let b = M2 * L2 * L2 + M2 * L1 * L2 * q2.cos();
    let c = M2 * L2 * L2;
    let det = a * c - b * b;
    [[c / det, -b / det], [-b / det, a / det]]
}

#[test]
fn unit_joint_effort_response_is_a_column_of_the_inverse_mass_matrix() {
    let (q1, q2) = (0.4, -1.1);
    let (model, state) = double_pendulum(q1, q2);
    let nv = model.layout().velocity_len();
    assert_eq!(nv, 8);
    let expected = inverse_mass_closed_form(q2);
    let mut work = Workspace::new();
    for (col, hinge) in [HINGE_1, HINGE_2].into_iter().enumerate() {
        let mut tau = vec![0.0; nv];
        tau[hinge] = 1.0;
        let mut accel = vec![0.0; nv];
        model
            .apply_inverse_mass(state.as_slice(), &tau, &mut work, &mut accel)
            .unwrap();
        for (row, other) in [HINGE_1, HINGE_2].into_iter().enumerate() {
            let want = expected[row][col];
            assert!(
                (accel[other] - want).abs() < 1e-9 * want.abs(),
                "M⁻¹[{row}][{col}]: {} vs closed form {want}",
                accel[other]
            );
        }
    }
}

/// Tip position derivatives: `(∂p/∂q1, ∂p/∂q2)` for the tip of bob 2.
fn tip_partials(q1: f64, q2: f64) -> (DVec3, DVec3) {
    let (p1, p2) = (q1, q1 + q2);
    let d1 = DVec3::new(p1.cos(), p1.sin(), 0.0) * L1;
    let d2 = DVec3::new(p2.cos(), p2.sin(), 0.0) * L2;
    (d1 + d2, d2)
}

#[test]
fn point_jacobian_action_matches_the_tip_velocity() {
    let (q1, q2) = (0.4, -1.1);
    let (model, state) = double_pendulum(q1, q2);
    let (dq1, dq2) = tip_partials(q1, q2);
    let mut v = vec![0.0; model.layout().velocity_len()];
    (v[HINGE_1], v[HINGE_2]) = (0.7, -1.3);
    let mut work = Workspace::new();
    let tip = model
        .point_velocity(state.as_slice(), &v, ComponentId(4), DVec3::ZERO, &mut work)
        .unwrap();
    let want = dq1 * 0.7 + dq2 * -1.3;
    assert!((tip - want).length() < 1e-12, "{tip:?} vs {want:?}");
}

#[test]
fn point_jacobian_transpose_matches_the_virtual_work_of_a_tip_force() {
    let (q1, q2) = (0.4, -1.1);
    let (model, state) = double_pendulum(q1, q2);
    let (dq1, dq2) = tip_partials(q1, q2);
    let force = DVec3::new(2.0, -3.0, 0.0);
    let mut tau = vec![0.0; model.layout().velocity_len()];
    let mut work = Workspace::new();
    model
        .point_force_to_generalized(
            state.as_slice(),
            ComponentId(4),
            DVec3::ZERO,
            force,
            &mut work,
            &mut tau,
        )
        .unwrap();
    assert!(
        (tau[HINGE_1] - force.dot(dq1)).abs() < 1e-12,
        "{}",
        tau[HINGE_1]
    );
    assert!(
        (tau[HINGE_2] - force.dot(dq2)).abs() < 1e-12,
        "{}",
        tau[HINGE_2]
    );
    // The base rows are the force itself and its moment about the root.
    assert!((DVec3::new(tau[0], tau[1], tau[2]) - force).length() < 1e-12);
}

/// Free-floating body carrying a ball-jointed arm with a hinged, offset tip:
/// every kind of generalized coordinate except the compliant joint.
fn floating_arm() -> (MultibodyModel, ModelState) {
    let tree = ComponentTree::new([
        (
            part(
                None,
                DVec3::ZERO,
                6.0,
                DMat3::from_diagonal(DVec3::new(2.0, 2.5, 3.0)),
                JointSpec::FIXED,
            ),
            posed(JointState::FIXED),
        ),
        (
            part(
                Some(0),
                DVec3::new(1.0, 0.2, -0.1),
                2.0,
                DMat3::from_diagonal(DVec3::new(0.2, 0.3, 0.4)),
                JointSpec::spherical(core::f64::consts::PI),
            ),
            posed(JointState::spherical(
                DQuat::from_euler(glam::EulerRot::XYZ, 0.3, 0.2, -0.4),
                DVec3::ZERO,
            )),
        ),
        (
            part(
                Some(1),
                DVec3::new(0.7, 0.2, 0.1),
                1.0,
                DMat3::from_diagonal(DVec3::new(0.01, 0.02, 0.03)),
                JointSpec::revolute(DVec3::Y, -3.0, 3.0),
            ),
            posed(JointState::revolute(0.5, 0.0)),
        ),
    ])
    .unwrap();
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(3.0, -1.0, 2.0),
        rotation_world_from_root: DQuat::from_rotation_x(0.3),
        ..FloatingBaseState::AT_REST
    };
    let state = model.state(&tree, &base, &(), &()).unwrap();
    (model, state)
}

#[test]
fn inverse_mass_is_symmetric_and_dual_to_the_point_jacobian() {
    let (model, state) = floating_arm();
    let nv = model.layout().velocity_len();
    assert_eq!(nv, 10);
    let mut work = Workspace::new();
    let mut columns = Vec::new();
    for j in 0..nv {
        let mut tau = vec![0.0; nv];
        tau[j] = 1.0;
        let mut accel = vec![0.0; nv];
        model
            .apply_inverse_mass(state.as_slice(), &tau, &mut work, &mut accel)
            .unwrap();
        columns.push(accel);
    }
    for i in 0..nv {
        for j in 0..i {
            let (a, b) = (columns[j][i], columns[i][j]);
            assert!(
                (a - b).abs() < 1e-12 * columns[i][i].abs().max(columns[j][j].abs()),
                "M⁻¹[{i}][{j}] = {a} but M⁻¹[{j}][{i}] = {b}"
            );
        }
    }
    // Power balance: (Jᵀ f) · v = f · (J v) for an arbitrary velocity.
    let v: Vec<f64> = (0..nv).map(|k| 0.3 - 0.17 * k as f64).collect();
    let (tip, offset) = (ComponentId(2), DVec3::new(0.2, -0.1, 0.4));
    let force = DVec3::new(1.5, -0.4, 2.2);
    let mut tau = vec![0.0; nv];
    model
        .point_force_to_generalized(state.as_slice(), tip, offset, force, &mut work, &mut tau)
        .unwrap();
    let velocity = model
        .point_velocity(state.as_slice(), &v, tip, offset, &mut work)
        .unwrap();
    let power: f64 = tau.iter().zip(&v).map(|(t, v)| t * v).sum();
    assert!((power - force.dot(velocity)).abs() < 1e-12, "{power}");
}

#[test]
fn point_acceleration_matches_the_tip_acceleration() {
    let (q1, q2) = (0.4, -1.1);
    let (model, mut state) = double_pendulum(q1, q2);
    let (w1, w2) = (0.7, -1.3);
    model
        .set_joint_state(&mut state, ComponentId(1), &JointState::revolute(q1, w1))
        .unwrap();
    model
        .set_joint_state(&mut state, ComponentId(3), &JointState::revolute(q2, w2))
        .unwrap();
    let mut accel = vec![0.0; model.layout().velocity_len()];
    let (a1, a2) = (0.9, 0.6);
    (accel[HINGE_1], accel[HINGE_2]) = (a1, a2);
    let mut work = Workspace::new();
    let tip = model
        .point_acceleration(
            state.as_slice(),
            &accel,
            ComponentId(4),
            DVec3::ZERO,
            &mut work,
        )
        .unwrap();
    // Each arm contributes a tangential and a centripetal part in its
    // absolute angle.
    let arm = |phi: f64, rate: f64, acc: f64, length: f64| {
        let tangent = DVec3::new(phi.cos(), phi.sin(), 0.0);
        let radial = DVec3::new(phi.sin(), -phi.cos(), 0.0);
        (tangent * acc - radial * (rate * rate)) * length
    };
    let want = arm(q1, w1, a1, L1) + arm(q1 + q2, w1 + w2, a1 + a2, L2);
    assert!((tip - want).length() < 1e-12, "{tip:?} vs {want:?}");
}

#[test]
fn point_acceleration_is_the_rate_of_the_point_velocity() {
    // A tumbling body with a spinning ball joint and a moving hinge.
    let (model, mut state) = floating_arm();
    let mut base = state.base();
    base.linear_vel_world_m_s = base.rotation_world_from_root * DVec3::new(0.5, -0.2, 0.1);
    base.angular_vel_root_rad_s = DVec3::new(0.4, -0.6, 0.3);
    state.set_base(&base);
    let ball = JointState::spherical(
        DQuat::from_euler(glam::EulerRot::XYZ, 0.3, 0.2, -0.4),
        DVec3::new(0.7, -0.5, 0.9),
    );
    model
        .set_joint_state(&mut state, ComponentId(1), &ball)
        .unwrap();
    model
        .set_joint_state(&mut state, ComponentId(2), &JointState::revolute(0.5, -0.8))
        .unwrap();
    let (tip, offset) = (ComponentId(2), DVec3::new(0.2, -0.1, 0.4));
    let mut work = Workspace::new();
    let mut rates = vec![0.0; state.as_slice().len()];
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            &(),
            &(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    let accel = &rates[model.layout().position_len()..];
    let predicted = model
        .point_acceleration(state.as_slice(), accel, tip, offset, &mut work)
        .unwrap();
    // One-sided second-order difference of the tip velocity over two steps.
    let h = 1.0e-4;
    let mut velocity = Vec::new();
    for i in 0..3 {
        let kinematics = model.kinematics(&state, &mut work).unwrap();
        velocity.push(kinematics.point(tip, offset).unwrap().velocity_world_m_s);
        model
            .advance(
                &mut Rk4::new(),
                &(),
                &(),
                &mut state,
                f64::from(i) * h,
                h,
                &mut work,
            )
            .unwrap();
    }
    let measured = (velocity[1] * 4.0 - velocity[0] * 3.0 - velocity[2]) / (2.0 * h);
    assert!(predicted.length() > 0.5, "acceleration too small to test");
    assert!(
        (predicted - measured).length() < 1.0e-6 * predicted.length(),
        "{predicted:?} vs {measured:?}"
    );
}
