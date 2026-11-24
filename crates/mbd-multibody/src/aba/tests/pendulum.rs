//! Single revolute compound pendulum hanging from the quasi-fixed ground,
//! checked against the closed-form period and against energy conservation.

use core::f64::consts::PI;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, Placement};

use super::rigs::{ground, hinge_z, idle, joint_of, part, run_on_ground};
use crate::FloatingBaseState;

const G: f64 = 9.81;
const LENGTH_M: f64 = 0.8;
const MASS_KG: f64 = 2.0;
const I_CM: f64 = 0.05;

fn gravity() -> DVec3 {
    DVec3::new(0.0, -G, 0.0)
}

/// Inertia about the pivot axis: bob inertia plus parallel-axis transfer.
fn pivot_inertia() -> f64 {
    I_CM + MASS_KG * LENGTH_M * LENGTH_M
}

fn pendulum(q0: f64) -> ComponentTree {
    ComponentTree::new([
        ground(),
        hinge_z(0, DVec3::ZERO, q0, 0.0),
        (
            part(
                "bob",
                Placement {
                    parent: Some(ComponentId(1)),
                    translation_m: DVec3::new(0.0, -LENGTH_M, 0.0),
                    rotation: DQuat::IDENTITY,
                },
                MASS_KG,
                DMat3::from_diagonal(DVec3::splat(I_CM)),
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ])
    .unwrap()
}

fn small_angle_period() -> f64 {
    2.0 * PI * (pivot_inertia() / (MASS_KG * G * LENGTH_M)).sqrt()
}

/// Exact period of a compound pendulum released from rest at `amplitude`:
/// `4 sqrt(I/(m g l)) K(sin(amplitude/2))`, with `K` from the
/// arithmetic-geometric mean, an independent closed form.
fn exact_period(amplitude: f64) -> f64 {
    let k = (amplitude / 2.0).sin();
    let (mut a, mut b) = (1.0_f64, (1.0 - k * k).sqrt());
    for _ in 0..30 {
        (a, b) = ((a + b) / 2.0, (a * b).sqrt());
    }
    let k_elliptic = PI / (2.0 * a);
    4.0 * (pivot_inertia() / (MASS_KG * G * LENGTH_M)).sqrt() * k_elliptic
}

fn energy(tree: &ComponentTree) -> f64 {
    let j = joint_of(tree, 1);
    0.5 * pivot_inertia() * j.q_vel_scalar * j.q_vel_scalar
        + MASS_KG * G * LENGTH_M * (1.0 - j.q_scalar.cos())
}

/// Period from successive downward zero crossings of the hinge angle,
/// linearly interpolated; the angle is nearly linear at the crossing because
/// `q_ddot ~ sin q` vanishes there.
fn measured_period(amplitude: f64, steps_per_period: usize, periods: usize) -> f64 {
    let mut tree = pendulum(amplitude);
    let dt = exact_period(amplitude) / steps_per_period as f64;
    let mut base = FloatingBaseState::AT_REST;
    let mut crossings = Vec::new();
    let mut t = 0.0;
    let mut q_prev = amplitude;
    while crossings.len() <= periods {
        base = run_on_ground(&mut tree, base, gravity(), &[], dt, 1);
        t += dt;
        let j = joint_of(&tree, 1);
        if q_prev > 0.0 && j.q_scalar <= 0.0 {
            crossings.push(t - dt + dt * q_prev / (q_prev - j.q_scalar));
        }
        q_prev = j.q_scalar;
    }
    (crossings[periods] - crossings[0]) / periods as f64
}

#[test]
fn small_angle_period_matches_compound_pendulum_formula() {
    let observed = measured_period(0.01, 2000, 3);
    let expected = small_angle_period();
    // At 0.01 rad the finite-amplitude lengthening is theta0^2/16 = 6.25e-6;
    // RK4 at 2000 steps per period contributes ~1e-12. A 1e-5 bound admits
    // the amplitude correction and nothing else.
    let rel = (observed - expected).abs() / expected;
    assert!(
        rel < 1.0e-5,
        "period {observed} s vs 2*pi*sqrt(I/(m g l)) = {expected} s (rel {rel:e})"
    );
}

#[test]
fn large_angle_period_matches_elliptic_integral() {
    let amplitude = 2.0;
    let observed = measured_period(amplitude, 2000, 3);
    let expected = exact_period(amplitude);
    // RK4 phase error at 2000 steps/period is ~(2*pi/2000)^4 ~ 1e-10, the
    // ground's reaction ~1e-12; 1e-7 leaves margin for crossing interpolation.
    let rel = (observed - expected).abs() / expected;
    assert!(
        rel < 1.0e-7,
        "period {observed} s vs elliptic {expected} s (rel {rel:e})"
    );
}

fn energy_drift(amplitude: f64, steps_per_period: usize, periods: usize) -> f64 {
    let mut tree = pendulum(amplitude);
    let e0 = energy(&tree);
    let dt = exact_period(amplitude) / steps_per_period as f64;
    let mut base = FloatingBaseState::AT_REST;
    let mut worst: f64 = 0.0;
    for _ in 0..periods {
        base = run_on_ground(&mut tree, base, gravity(), &[], dt, steps_per_period);
        worst = worst.max((energy(&tree) - e0).abs());
    }
    worst / (MASS_KG * G * LENGTH_M)
}

#[test]
fn large_angle_energy_is_conserved_over_fifty_periods() {
    let amplitude = 2.5;
    let coarse = energy_drift(amplitude, 200, 50);
    let fine = energy_drift(amplitude, 400, 50);
    // RK4's secular energy error scales as (omega dt)^4; at 200 steps per
    // period, (2*pi/200)^4 ~ 1e-6 per period before its small leading constant,
    // so 50 periods stay below 1e-5 m g l for a correct right-hand side.
    assert!(
        coarse < 1.0e-5,
        "energy drift {coarse:e} m g l over 50 periods at T/200"
    );
    // A wrong equation of motion does not shrink with dt; truncation drift
    // falls ~16x on halving, so require at least 8x.
    assert!(
        fine < coarse / 8.0,
        "drift does not converge: T/200 {coarse:e}, T/400 {fine:e}"
    );
}
