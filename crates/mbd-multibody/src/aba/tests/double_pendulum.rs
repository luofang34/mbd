//! Planar double pendulum of point masses on the quasi-fixed ground: joint
//! accelerations against the Lagrangian closed form, and energy conservation.

use glam::DVec3;
use mbd_core::ComponentTree;

use super::rigs::{ground, ground_support, hinge_z, joint_of, point_mass, run_on_ground};
use crate::{FloatingBaseState, MultiBodyDynamics};

const G: f64 = 9.81;
const L1: f64 = 1.1;
const L2: f64 = 0.7;
const M1: f64 = 1.5;
const M2: f64 = 0.9;

fn gravity() -> DVec3 {
    DVec3::new(0.0, -G, 0.0)
}

/// Ground, hinge 1, bob 1, hinge 2 (relative angle) on bob 1, bob 2.
fn double_pendulum(q1: f64, q1_dot: f64, q2: f64, q2_dot: f64) -> ComponentTree {
    ComponentTree::new([
        ground(),
        hinge_z(0, DVec3::ZERO, q1, q1_dot),
        point_mass(1, DVec3::new(0.0, -L1, 0.0), M1),
        hinge_z(2, DVec3::ZERO, q2, q2_dot),
        point_mass(3, DVec3::new(0.0, -L2, 0.0), M2),
    ])
    .unwrap()
}

/// Lagrange's equations in absolute angles `phi1 = q1`, `phi2 = q1 + q2`
/// (each measured from the downward vertical), solved by Cramer's rule and
/// mapped back to the relative joint coordinates `(q1_ddot, q2_ddot)`.
fn lagrangian_accels(q1: f64, q1_dot: f64, q2: f64, q2_dot: f64) -> (f64, f64) {
    let (p1, p2) = (q1, q1 + q2);
    let (w1, w2) = (q1_dot, q1_dot + q2_dot);
    let d = p1 - p2;
    let a11 = (M1 + M2) * L1;
    let a12 = M2 * L2 * d.cos();
    let a21 = L1 * d.cos();
    let a22 = L2;
    let b1 = -M2 * L2 * w2 * w2 * d.sin() - (M1 + M2) * G * p1.sin();
    let b2 = L1 * w1 * w1 * d.sin() - G * p2.sin();
    let det = a11 * a22 - a12 * a21;
    let p1_dd = (b1 * a22 - a12 * b2) / det;
    let p2_dd = (a11 * b2 - a21 * b1) / det;
    (p1_dd, p2_dd - p1_dd)
}

fn solver_accels(tree: &ComponentTree) -> (f64, f64) {
    let base = FloatingBaseState::AT_REST;
    let out = MultiBodyDynamics::forward_dynamics(
        tree,
        &[],
        &base,
        gravity(),
        &[],
        &[ground_support(&base, gravity())],
    )
    .unwrap();
    assert_eq!(out.joint_accels.len(), 2);
    (
        out.joint_accels[0].q_accel_scalar,
        out.joint_accels[1].q_accel_scalar,
    )
}

#[test]
fn double_pendulum_rests_in_equilibrium() {
    // Rig anchor: hanging straight down at rest must not accelerate, so any
    // mismatch below is in the dynamics, not in the support or sign setup.
    let (a1, a2) = solver_accels(&double_pendulum(0.0, 0.0, 0.0, 0.0));
    let tol = 1e-12 * G / L2;
    assert!(a1.abs() < tol && a2.abs() < tol, "{a1} {a2}");
}

#[test]
fn double_pendulum_joint_accels_match_lagrangian() {
    let states = [
        (0.7, 1.3, -1.1, -0.4),
        (2.4, -0.8, 1.9, 2.2),
        (-0.3, 3.0, 0.0, -3.5),
    ];
    for (q1, w1, q2, w2) in states {
        let (a1, a2) = solver_accels(&double_pendulum(q1, w1, q2, w2));
        let (e1, e2) = lagrangian_accels(q1, w1, q2, w2);
        // The ground reacts at ~M/GROUND_KG = 1e-12; everything else is
        // double-precision arithmetic on O(10) quantities, so 1e-9 of the
        // acceleration scale (g / L) is a rounding-only bound.
        let scale = G / L2;
        assert!(
            (a1 - e1).abs() < 1e-9 * scale && (a2 - e2).abs() < 1e-9 * scale,
            "state {:?}: solver ({a1}, {a2}) vs Lagrangian ({e1}, {e2})",
            (q1, w1, q2, w2)
        );
    }
}

fn energy(tree: &ComponentTree) -> f64 {
    let (j1, j2) = (joint_of(tree, 1), joint_of(tree, 3));
    let (p1, p2) = (j1.q_scalar, j1.q_scalar + j2.q_scalar);
    let (w1, w2) = (j1.q_vel_scalar, j1.q_vel_scalar + j2.q_vel_scalar);
    let v1 = DVec3::new(p1.cos(), p1.sin(), 0.0) * (L1 * w1);
    let v2 = v1 + DVec3::new(p2.cos(), p2.sin(), 0.0) * (L2 * w2);
    let kinetic = 0.5 * M1 * v1.length_squared() + 0.5 * M2 * v2.length_squared();
    let potential = -M1 * G * L1 * p1.cos() - M2 * G * (L1 * p1.cos() + L2 * p2.cos());
    kinetic + potential
}

fn energy_drift(dt: f64, duration_s: f64) -> f64 {
    let mut tree = double_pendulum(2.0, 0.0, 1.0, 0.0);
    let e0 = energy(&tree);
    let steps = (duration_s / dt).round() as usize;
    let mut base = FloatingBaseState::AT_REST;
    let mut worst: f64 = 0.0;
    for _ in 0..steps / 100 {
        base = run_on_ground(&mut tree, base, gravity(), &[], dt, 100);
        worst = worst.max((energy(&tree) - e0).abs());
    }
    worst / ((M1 + M2) * G * (L1 + L2))
}

#[test]
fn chaotic_double_pendulum_conserves_energy() {
    let coarse = energy_drift(2.0e-3, 10.0);
    let fine = energy_drift(1.0e-3, 10.0);
    // Released from 2 rad / 3 rad absolute the motion is chaotic; the fastest
    // rates are ~6 rad/s, so omega*dt ~ 1e-2 and RK4 truncation is ~1e-8 per
    // unit time. 1e-6 of the potential-energy scale is a truncation-only bound.
    assert!(coarse < 1.0e-6, "energy drift {coarse:e} at dt = 2 ms");
    // Truncation falls ~16x on halving dt; a wrong right-hand side does not.
    assert!(
        fine < coarse / 8.0,
        "drift does not converge: 2 ms {coarse:e}, 1 ms {fine:e}"
    );
}
