//! Free flight: the fly released from rest with its wings on the
//! prescribed motion, integrated for 50 wingbeats at 100 steps per beat.
//!
//! The bookkeeping closes over the bodies and the entrained air together:
//! the air's impulse changes only by the circulatory forces, so
//! `d(P_bodies + P_air)/dt = F_circ + M g`, and likewise for angular
//! momentum about a fixed point.

use std::io::Write;
use std::time::Instant;

use glam::DVec3;
use mbd_multibody::{DormandPrince54, FloatingBaseState, Integrator, Rk4, Tolerances, Workspace};

use super::fly::{Fly, FlySpec, FlyWorld};

const STROKE_RIGHT: mbd_core::ComponentId = mbd_core::ComponentId(1);
use super::probe;

pub const BEATS: u32 = 50;
pub const STEPS_PER_BEAT: u32 = 100;

/// Samples at every step boundary of one flight.
pub struct Trace {
    pub com: Vec<DVec3>,
    /// Vertical circulatory force after each step, N.
    pub lift: Vec<f64>,
    pub body_attitude: Vec<glam::DQuat>,
    pub momentum_error: f64,
    pub angular_error: f64,
    pub mean_force: DVec3,
    pub mean_climb_force: f64,
    /// Climb of the centre of mass predicted from the force history alone:
    /// `M ż = P_sys(0) + ∫(F_circ − W) − P_air`, integrated in time.
    pub predicted_climb: f64,
    /// Peak-to-peak stroke angle of the right wing over the last beat, rad.
    pub late_stroke_rad: f64,
    pub derivative_evals: u64,
    pub wall_s: f64,
}

/// Flies `spec` from rest for `beats` wingbeats with `integrator`.
pub fn fly_free<G: Integrator>(spec: FlySpec, integrator: &mut G, beats: u32, air: bool) -> Trace {
    let fly = Fly::new(spec);
    let model = probe::model(&fly);
    let elements = probe::elements(&fly, air, Vec::new());
    let world = FlyWorld::new(&spec);
    let base = FloatingBaseState::AT_REST;
    let mut state = probe::state_at(&fly, &model, &elements, &world, &base, 0.0);
    let mut work = Workspace::new();
    let h = spec.period_s() / f64::from(STEPS_PER_BEAT);
    let gravity = DVec3::new(0.0, 0.0, -spec.weight_n());
    let sample = |state: &mbd_multibody::ModelState, work: &mut Workspace| {
        let (p, l, com) = probe::bodies(&model, state);
        let kin = model.kinematics(state, work).unwrap();
        let w = probe::wings(&fly, &kin, &world);
        (p + w.fluid_linear, l + w.fluid_angular, com, w)
    };
    let (p0, l0, com0, w0) = sample(&state, &mut work);
    let mut trace = Trace {
        com: vec![com0],
        lift: vec![w0.force.z],
        body_attitude: vec![state.base().rotation_world_from_root],
        momentum_error: 0.0,
        angular_error: 0.0,
        mean_force: DVec3::ZERO,
        mean_climb_force: 0.0,
        predicted_climb: 0.0,
        late_stroke_rad: 0.0,
        derivative_evals: 0,
        wall_s: 0.0,
    };
    let (mut impulse, mut angular_impulse) = (DVec3::ZERO, DVec3::ZERO);
    let (mut abs_impulse, mut abs_angular) = (0.0, 0.0);
    let (mut prev_f, mut prev_tau) = (w0.force + gravity, w0.moment_origin + com0.cross(gravity));
    let mass = spec.total_mass_kg();
    let mut prev_vz = (p0.z - w0.fluid_linear.z) / mass;
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let start = Instant::now();
    for k in 0..beats * STEPS_PER_BEAT {
        let t = f64::from(k) * h;
        let report = model
            .advance(integrator, &elements, &world, &mut state, t, h, &mut work)
            .unwrap();
        trace.derivative_evals += u64::from(report.derivative_evals);
        let (_, _, com, w) = sample(&state, &mut work);
        let (f, tau) = (w.force + gravity, w.moment_origin + com.cross(gravity));
        impulse += 0.5 * h * (f + prev_f);
        angular_impulse += 0.5 * h * (tau + prev_tau);
        let vz = (p0.z + impulse.z - w.fluid_linear.z) / mass;
        trace.predicted_climb += 0.5 * h * (vz + prev_vz);
        prev_vz = vz;
        abs_impulse += h * w.force.length();
        abs_angular += h * w.moment_origin.length();
        trace.mean_force += w.force;
        (prev_f, prev_tau) = (f, tau);
        if k >= (beats - 1) * STEPS_PER_BEAT {
            let q = model.joint_state(&state, STROKE_RIGHT).unwrap().q_scalar;
            (lo, hi) = (lo.min(q), hi.max(q));
        }
        trace.com.push(com);
        trace.lift.push(w.force.z);
        trace
            .body_attitude
            .push(state.base().rotation_world_from_root);
    }
    trace.wall_s = start.elapsed().as_secs_f64();
    trace.late_stroke_rad = hi - lo;
    let (p1, l1, _, _) = sample(&state, &mut work);
    trace.momentum_error = ((p1 - p0) - impulse).length() / abs_impulse;
    trace.angular_error = ((l1 - l0) - angular_impulse).length() / abs_angular;
    trace.mean_force /= f64::from(beats * STEPS_PER_BEAT);
    trace.mean_climb_force = trace.mean_force.z - spec.weight_n();
    trace
}

#[test]
fn fifty_wingbeats_climb_as_lift_minus_weight_and_balance_momentum() {
    let spec = FlySpec::drosophila();
    let rk4 = fly_free(spec, &mut Rk4::new(), BEATS, true);
    let duration = f64::from(BEATS) * spec.period_s();
    let climb = rk4.com.last().unwrap().z - rk4.com[0].z;
    let expected = 0.5 * rk4.mean_climb_force / spec.total_mass_kg() * duration * duration;
    let tilt = rk4
        .body_attitude
        .iter()
        .map(|q| (*q * DVec3::Z).angle_between(DVec3::Z))
        .fold(0.0_f64, f64::max);
    writeln!(
        std::io::stderr(),
        "free flight RK4: climb {:.3} mm vs force history {:.3} mm vs (F̄−W)T²/2M {:.3} mm, mean lift/weight {:.3}, \
         momentum error {:.2e}, angular error {:.2e}, max body tilt {:.2} deg, \
         late stroke {:.1} deg, climb rate {:.3} m/s, {} evals in {:.3} s",
        climb * 1e3,
        rk4.predicted_climb * 1e3,
        expected * 1e3,
        rk4.mean_force.z / spec.weight_n(),
        rk4.momentum_error,
        rk4.angular_error,
        tilt.to_degrees(),
        rk4.late_stroke_rad.to_degrees(),
        (rk4.com[rk4.com.len() - 1].z - rk4.com[rk4.com.len() - 2].z) / (spec.period_s() / f64::from(STEPS_PER_BEAT)),
        rk4.derivative_evals,
        rk4.wall_s,
    )
    .ok();
    // The climb follows from the force history: the integrator's centre of
    // mass against twice-integrated lift minus weight. The force has kinks
    // at stroke reversal, so trapezoidal quadrature at 100 points per beat
    // carries an error of up to a few tenths of a percent, vanishing as the
    // sampling is refined while the RK4 climb stays put: 1e-2.
    let predicted = rk4.predicted_climb;
    assert!(
        (climb - predicted).abs() < 1e-2 * climb.abs(),
        "{climb} vs {predicted}"
    );
    // Lift exceeds weight, so the fly climbs. Its climb rate, a sizeable
    // fraction of the wing speed by the end, lowers the wings' incidence,
    // so the force falls over the flight and the constant-mean estimate is
    // approximate only (within a third).
    assert!(climb > 0.0 && (climb - expected).abs() < expected / 3.0);
    // The pitch reflex shifts the stroke slowly and leaves its amplitude:
    // the realised stroke over the last beat is the prescribed one to 2 %.
    let stroke = rk4.late_stroke_rad / spec.stroke_amplitude_rad;
    assert!((stroke - 1.0).abs() < 0.02, "realised stroke {stroke}");
    // Momentum of bodies plus entrained air against the impulse of
    // circulatory forces and weight, same quadrature: 1e-3 of the total
    // impulse magnitude.
    assert!(rk4.momentum_error < 1e-3, "{}", rk4.momentum_error);
    assert!(rk4.angular_error < 1e-3, "{}", rk4.angular_error);
    assert!(
        tilt < 10f64.to_radians(),
        "pitch control holds the body upright"
    );

    let tol = Tolerances::uniform(1e-9, 1e-9);
    let dp = fly_free(spec, &mut DormandPrince54::new(tol), BEATS, true);
    let gap = rk4
        .com
        .iter()
        .zip(&dp.com)
        .map(|(a, b)| (*a - *b).length())
        .fold(0.0_f64, f64::max);
    writeln!(
        std::io::stderr(),
        "free flight DP54: max COM gap to RK4 {:.2e} wing lengths, {} evals in {:.3} s",
        gap / spec.wing_length_m,
        dp.derivative_evals,
        dp.wall_s,
    )
    .ok();
    // RK4 at ω_n h ≈ 0.63 on the servo against an error-controlled
    // reference: a hundredth of a wing length over a 40-wing-length climb.
    assert!(gap < 1e-2 * spec.wing_length_m, "{gap}");
}
