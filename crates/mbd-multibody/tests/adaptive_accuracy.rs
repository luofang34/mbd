//! Dormand–Prince 5(4) against classical RK4: for the same final accuracy
//! the adaptive method needs fewer derivative evaluations, on an eccentric
//! Kepler orbit (closed form) and on a chaotic double pendulum (fine-step
//! reference), and it ends exactly on the requested time.

#![cfg(test)]

use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
    WrenchAtSite,
};
use mbd_multibody::{
    DormandPrince54, ElementContext, FloatingBaseState, ForceElement, ForceFn, Integrator, Loads,
    ModelScale, ModelState, MultibodyError, MultibodyModel, Rk4, TimeScale, Tolerances, Workspace,
};

fn part(
    parent: Option<u32>,
    at: DVec3,
    mass_kg: f64,
    inertia: f64,
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement: parent.map_or(Placement::ROOT, |p| Placement::child_of(ComponentId(p), at)),
        dry_mass_kg: mass_kg,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(inertia)),
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

/// A problem: model, initial state and loads, integrated over `duration_s`
/// in `intervals` equal host intervals.
struct Problem<E> {
    model: MultibodyModel,
    start: ModelState,
    elements: E,
    duration_s: f64,
    intervals: u32,
}

impl<E: ForceElement<()>> Problem<E> {
    /// Final state and derivative evaluations spent.
    fn solve<G: Integrator>(&self, integrator: &mut G) -> (ModelState, u32) {
        let mut state = self.start.clone();
        let mut work = Workspace::new();
        let dt = self.duration_s / f64::from(self.intervals);
        let mut evals = 0;
        for i in 0..self.intervals {
            let t0 = f64::from(i) * dt;
            let report = self
                .model
                .advance(
                    integrator,
                    &self.elements,
                    &(),
                    &mut state,
                    t0,
                    dt,
                    &mut work,
                )
                .unwrap();
            evals += report.derivative_evals;
        }
        (state, evals)
    }

    /// Fewest RK4 evaluations (uniform steps, searched in 10 % increments)
    /// whose error, measured by `error`, is at most `target`.
    fn rk4_evals_for(&self, target: f64, error: impl Fn(&ModelState) -> f64) -> u32 {
        let mut per_interval = 1.0_f64;
        loop {
            let n = per_interval.ceil() as u32;
            let (state, evals) = self.solve(&mut Rk4::with_substeps(n));
            if error(&state) <= target {
                return evals;
            }
            per_interval *= 1.1;
            assert!(n < 1_000_000, "RK4 cannot reach {target:e}");
        }
    }
}

const ECCENTRICITY: f64 = 0.5;

/// Position on the orbit `a = 1, mu = 1` at time `t` after periapsis, by
/// Newton iteration on Kepler's equation.
fn kepler_position(t: f64) -> DVec3 {
    let e = ECCENTRICITY;
    let mut anomaly = t;
    for _ in 0..50 {
        anomaly -= (anomaly - e * anomaly.sin() - t) / (1.0 - e * anomaly.cos());
    }
    DVec3::new(anomaly.cos() - e, (1.0 - e * e).sqrt() * anomaly.sin(), 0.0)
}

type Field = fn(&ElementContext<'_, ()>, &mut Loads<'_>) -> Result<(), MultibodyError>;

fn central_field(
    ctx: &ElementContext<'_, ()>,
    loads: &mut Loads<'_>,
) -> Result<(), MultibodyError> {
    let r = ctx.kinematics.base().position_world_m;
    loads.add_gravity(-r / (r.length_squared() * r.length()));
    Ok(())
}

fn kepler() -> Problem<ForceFn<Field>> {
    let e = ECCENTRICITY;
    let tree = ComponentTree::new([(
        part(None, DVec3::ZERO, 1.0, 1.0, JointSpec::FIXED),
        posed(JointState::FIXED),
    )])
    .unwrap();
    // Unit semi-major axis and gravitational parameter: the orbital period
    // is 2π, its time unit 1.
    let unit = ModelScale::new(1.0, 1.0, 1.0).unwrap();
    let model = MultibodyModel::new(&tree, &[], unit).unwrap();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(1.0 - e, 0.0, 0.0),
        linear_vel_root_m_s: DVec3::new(0.0, ((1.0 + e) / (1.0 - e)).sqrt(), 0.0),
        ..FloatingBaseState::AT_REST
    };
    let elements = ForceFn(central_field as Field);
    let start = model.state(&tree, &base, &elements, &()).unwrap();
    Problem {
        model,
        start,
        elements,
        duration_s: 3.0 * core::f64::consts::TAU,
        intervals: 24,
    }
}

#[test]
fn eccentric_kepler_orbit_costs_fewer_evaluations_than_rk4() {
    let problem = kepler();
    let exact = kepler_position(problem.duration_s);
    let error = |state: &ModelState| (state.base().position_world_m - exact).length();
    let (reached, evals) = adaptive_error(&problem, 1.0e-9, error);
    let (tighter, _) = adaptive_error(&problem, 1.0e-11, error);
    let rk4_evals = problem.rk4_evals_for(reached, error);
    // The local tolerance accumulates over a few hundred steps of an orbit
    // whose phase error grows linearly: three decades above it at most.
    assert!(reached < 1.0e-6, "error {reached:e}");
    // The error follows the tolerance: a hundredth of it buys at least a
    // thirtieth of the error.
    assert!(tighter < reached / 30.0, "{reached:e} -> {tighter:e}");
    assert!(evals < rk4_evals, "DP54 {evals} vs RK4 {rk4_evals}");
}

/// Final error and evaluations of DP54 at a uniform tolerance.
fn adaptive_error<E: ForceElement<()>>(
    problem: &Problem<E>,
    tolerance: f64,
    error: impl Fn(&ModelState) -> f64,
) -> (f64, u32) {
    let mut adaptive = DormandPrince54::new(Tolerances::uniform(tolerance, tolerance));
    let (state, evals) = problem.solve(&mut adaptive);
    (error(&state), evals)
}

const GROUND_KG: f64 = 1.0e12;
const GRAVITY: DVec3 = DVec3::new(0.0, -9.81, 0.0);

fn held_ground(ctx: &ElementContext<'_, ()>, loads: &mut Loads<'_>) -> Result<(), MultibodyError> {
    // Gravity on everything, and a support carrying the ground's own weight,
    // so the hinge at its origin is a fixed pivot.
    let g_root = ctx.kinematics.base().rotation_world_from_root.conjugate() * GRAVITY;
    loads.add_gravity(GRAVITY);
    loads.add_wrench(&WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: -g_root * GROUND_KG,
        torque_local_nm: DVec3::ZERO,
    })
}

fn hinge(parent: u32, q: f64) -> (ComponentSpec, ComponentState) {
    let joint = JointSpec::revolute(DVec3::Z, -1.0e6, 1.0e6);
    (
        part(Some(parent), DVec3::ZERO, 0.0, 0.0, joint),
        posed(JointState::revolute(q, 0.0)),
    )
}

fn bob(parent: u32, length: f64, mass_kg: f64) -> (ComponentSpec, ComponentState) {
    let at = DVec3::new(0.0, -length, 0.0);
    (
        part(Some(parent), at, mass_kg, 0.0, JointSpec::FIXED),
        posed(JointState::FIXED),
    )
}

/// Double pendulum released from 2 rad and 3 rad from the vertical.
fn double_pendulum() -> Problem<ForceFn<Field>> {
    let tree = ComponentTree::new([
        (
            part(None, DVec3::ZERO, GROUND_KG, GROUND_KG, JointSpec::FIXED),
            posed(JointState::FIXED),
        ),
        hinge(0, 2.0),
        bob(1, 1.1, 1.5),
        hinge(2, 1.0),
        bob(3, 0.7, 0.9),
    ])
    .unwrap();
    // Metre-scale arms and the bobs' mass; the held ground's 10¹² kg is a
    // stand-in for a fixed base and would set no useful scale.
    let scale = ModelScale::with_time(1.0, 1.5 + 0.9, TimeScale::Acceleration(9.81)).unwrap();
    let model = MultibodyModel::new(&tree, &[], scale).unwrap();
    let elements = ForceFn(held_ground as Field);
    let start = model
        .state(&tree, &FloatingBaseState::AT_REST, &elements, &())
        .unwrap();
    Problem {
        model,
        start,
        elements,
        duration_s: 3.0,
        intervals: 30,
    }
}

/// Joint angles and rates `[q1, q̇1, q2, q̇2]`.
fn pendulum_coordinates(problem: &Problem<ForceFn<Field>>, state: &ModelState) -> [f64; 4] {
    let j1 = problem.model.joint_state(state, ComponentId(1)).unwrap();
    let j2 = problem.model.joint_state(state, ComponentId(3)).unwrap();
    [j1.q_scalar, j1.q_vel_scalar, j2.q_scalar, j2.q_vel_scalar]
}

#[test]
fn chaotic_double_pendulum_costs_fewer_evaluations_than_rk4() {
    let problem = double_pendulum();
    // Reference: RK4 at 100 microseconds, whose own error is far below the
    // errors compared here.
    let (reference, _) = problem.solve(&mut Rk4::with_substeps(1_000));
    let reference = pendulum_coordinates(&problem, &reference);
    let error = |state: &ModelState| {
        let now = pendulum_coordinates(&problem, state);
        let sum: f64 = now
            .iter()
            .zip(&reference)
            .map(|(a, b)| (a - b) * (a - b))
            .sum();
        sum.sqrt()
    };
    let (reached, evals) = adaptive_error(&problem, 1.0e-8, error);
    let (tighter, _) = adaptive_error(&problem, 1.0e-10, error);
    let rk4_evals = problem.rk4_evals_for(reached, error);
    // The motion is chaotic: neighbouring trajectories separate by three to
    // four decades over the run, and the local tolerance with them.
    assert!(reached < 1.0e-4, "error {reached:e}");
    assert!(tighter < reached / 30.0, "{reached:e} -> {tighter:e}");
    assert!(evals < rk4_evals, "DP54 {evals} vs RK4 {rk4_evals}");
}

/// A clock: one state whose rate is one, so it reads the time the
/// integrator actually covered.
struct Clock;

impl<I: ?Sized> ForceElement<I> for Clock {
    fn state_len(&self) -> usize {
        1
    }

    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        _loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        state_dot.fill(1.0);
        Ok(())
    }
}

#[test]
fn adaptive_sub_steps_end_exactly_on_the_requested_time() {
    let problem = kepler();
    let elements = (ForceFn(central_field as Field), Clock);
    let tree = ComponentTree::new([(
        part(None, DVec3::ZERO, 1.0, 1.0, JointSpec::FIXED),
        posed(JointState::FIXED),
    )])
    .unwrap();
    let mut state = problem
        .model
        .state(&tree, &problem.start.base(), &elements, &())
        .unwrap();
    let mut adaptive = DormandPrince54::new(Tolerances::uniform(1.0e-10, 1.0e-10));
    let mut work = Workspace::new();
    // An interval that no power-of-two step divides.
    let (t0, dt) = (0.3, 2.345_678_9);
    let report = problem
        .model
        .advance(&mut adaptive, &elements, &(), &mut state, t0, dt, &mut work)
        .unwrap();
    assert!(report.accepted_steps > 10, "{report:?}");
    assert_eq!(report.end_time_s.to_bits(), (t0 + dt).to_bits());
    // The clock integrates exactly, so it reads the sum of the sub-steps.
    let clock = state.auxiliary()[0];
    assert!(
        (clock - dt).abs() <= 8.0 * f64::EPSILON * dt,
        "clock {clock}"
    );
}
