//! A milligram body on penalty elements at a real-time host step: explicit
//! RK4 diverges unless it sub-steps in proportion to the stiffness, the
//! L-stable Rosenbrock method runs at the host step whatever the stiffness,
//! and mechanical energy never grows.

#![cfg(test)]

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    ContactPointSpec, FloatingBaseState, ForceElement, Integrator, ModelScale, ModelState,
    MultibodyModel, Rk4, Rosenbrock23, SupportedLoad, SurfaceMedium, SurfaceSample, Tether,
    TetherAnchor, TetherCommand, TimeScale, Tolerances, UniformGravity, WinchTetherSpec, Workspace,
};

/// Body: 1 mg, 1 mm.
const MASS_KG: f64 = 1.0e-6;
const SIZE_M: f64 = 1.0e-3;
const G: f64 = 9.81;
/// Host step of a 100 Hz real-time loop.
const HOST_DT_S: f64 = 0.01;
const HOST_STEPS: u32 = 100;
/// Energy scale of the problems here: the body's weight over its own size.
const ENERGY_J: f64 = MASS_KG * G * SIZE_M;

fn mote() -> (MultibodyModel, ComponentTree) {
    let spec = ComponentSpec {
        name: "mote".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: MASS_KG,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(0.1 * MASS_KG * SIZE_M * SIZE_M)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let scale = ModelScale::with_time(SIZE_M, MASS_KG, TimeScale::Acceleration(G)).unwrap();
    let model = MultibodyModel::new(&tree, &[], scale).unwrap();
    (model, tree)
}

fn ground() -> SurfaceSample {
    SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        velocity_world_m_s: DVec3::ZERO,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    }
}

/// Contact under the body origin whose static penetration is
/// `penetration_fraction` of the body size.
fn foot(penetration_fraction: f64) -> ContactPointSpec {
    let load = SupportedLoad {
        mass_kg: MASS_KG,
        size_m: SIZE_M,
        gravity_m_s2: G,
        penetration_fraction,
        damping_ratio: 0.3,
    };
    ContactPointSpec::sized_for(ComponentId(0), DVec3::ZERO, &load, 0.9, 0.7)
}

fn at(position: DVec3) -> FloatingBaseState {
    FloatingBaseState {
        position_world_m: position,
        ..FloatingBaseState::AT_REST
    }
}

/// Outcome of running one integrator for [`HOST_STEPS`] host steps.
struct Run {
    finite: bool,
    /// Largest energy increase over any single host step, J.
    worst_energy_gain_j: f64,
    accepted_steps: u32,
    derivative_evals: u32,
    end: FloatingBaseState,
}

fn run<I, E, G>(
    integrator: &mut G,
    elements: &E,
    inputs: &I,
    start: &FloatingBaseState,
    energy: impl Fn(&ModelState) -> f64,
) -> Run
where
    I: ?Sized,
    E: ForceElement<I>,
    G: Integrator,
{
    let (model, tree) = mote();
    let mut work = Workspace::new();
    let mut state = model.state(&tree, start, elements, inputs).unwrap();
    let mut out = Run {
        finite: true,
        worst_energy_gain_j: 0.0,
        accepted_steps: 0,
        derivative_evals: 0,
        end: *start,
    };
    let mut before = energy(&state);
    for i in 0..HOST_STEPS {
        let t0 = f64::from(i) * HOST_DT_S;
        let step = model.advance(
            integrator, elements, inputs, &mut state, t0, HOST_DT_S, &mut work,
        );
        let Ok(report) = step else {
            out.finite = false;
            break;
        };
        if !state.as_slice().iter().all(|v| v.is_finite()) {
            out.finite = false;
            break;
        }
        let after = energy(&state);
        out.worst_energy_gain_j = out.worst_energy_gain_j.max(after - before);
        before = after;
        out.accepted_steps += report.accepted_steps;
        out.derivative_evals += report.derivative_evals;
    }
    out.end = state.base();
    out
}

/// A run is physically acceptable when it stays finite and no host step
/// gains more than a thousandth of the problem's energy scale.
fn stable(run: &Run) -> bool {
    run.finite && run.worst_energy_gain_j < 1.0e-3 * ENERGY_J
}

fn rosenbrock() -> Rosenbrock23 {
    Rosenbrock23::new(Tolerances::uniform(1.0e-3, 1.0e-3))
}

/// Fewest equal RK4 sub-steps per host step that give a stable run.
fn rk4_substeps_needed<I: ?Sized, E: ForceElement<I>>(
    elements: &E,
    inputs: &I,
    start: &FloatingBaseState,
    energy: impl Fn(&ModelState) -> f64,
) -> u32 {
    (1..=512)
        .find(|&n| {
            stable(&run(
                &mut Rk4::with_substeps(n),
                elements,
                inputs,
                start,
                &energy,
            ))
        })
        .unwrap()
}

/// Kinetic, gravitational and contact-spring energy of the body on `foot`.
fn contact_energy(foot: &ContactPointSpec) -> impl Fn(&ModelState) -> f64 + '_ {
    |state| {
        let base = state.base();
        let z = base.position_world_m.z;
        let pen = (-z).max(0.0);
        0.5 * MASS_KG * base.linear_vel_world_m_s.length_squared()
            + MASS_KG * G * z
            + 0.5 * foot.stiffness_n_m * pen * pen
    }
}

/// Runs the contact problem from `start` with penetration fraction
/// `fraction`; returns the Rosenbrock run and the RK4 sub-steps needed.
fn contact_case(fraction: f64, start: DVec3) -> (Run, u32) {
    let foot = foot(fraction);
    let elements = (UniformGravity(DVec3::new(0.0, 0.0, -G)), foot);
    let energy = contact_energy(&foot);
    let stiff = run(&mut rosenbrock(), &elements, &ground(), &at(start), &energy);
    let rk4_at_host_step = run(&mut Rk4::new(), &elements, &ground(), &at(start), &energy);
    assert!(
        !stable(&rk4_at_host_step),
        "RK4 at the host step should not survive a contact this stiff"
    );
    let needed = rk4_substeps_needed(&elements, &ground(), &at(start), &energy);
    let depth = fraction * SIZE_M;
    assert!(
        stable(&stiff),
        "energy gain {:e} J",
        stiff.worst_energy_gain_j
    );
    // Settled on the static penetration, at rest.
    assert!(
        (stiff.end.position_world_m.z + depth).abs() < 1.0e-3 * depth,
        "rest height {:e}",
        stiff.end.position_world_m.z
    );
    assert!(stiff.end.linear_vel_world_m_s.length() < 1.0e-6 * (G * SIZE_M).sqrt());
    (stiff, needed)
}

#[test]
fn mote_resting_on_a_penalty_contact_is_stable_at_the_host_step() {
    // Released half a static penetration too deep.
    let (stiff, needed) = contact_case(0.01, DVec3::new(0.0, 0.0, -1.5e-5));
    assert!(needed >= 3, "RK4 needed only {needed} sub-steps");
    // Essentially one linearised step per host step.
    assert!(stiff.accepted_steps <= HOST_STEPS + HOST_STEPS / 5);
}

#[test]
fn mote_bouncing_on_a_penalty_contact_is_stable_at_the_host_step() {
    // Dropped from one body length.
    let (stiff, needed) = contact_case(0.01, DVec3::new(0.0, 0.0, SIZE_M));
    assert!(needed >= 3, "RK4 needed only {needed} sub-steps");
    assert!(stiff.accepted_steps <= 2 * HOST_STEPS);
}

#[test]
fn stiffer_contact_costs_rk4_sub_steps_but_not_the_stiff_integrator() {
    // A hundredth of the penetration: ten times the contact frequency.
    let (soft, needed_soft) = contact_case(1.0e-2, DVec3::new(0.0, 0.0, SIZE_M));
    let (hard, needed_hard) = contact_case(1.0e-4, DVec3::new(0.0, 0.0, SIZE_M));
    assert!(
        needed_hard >= 5 * needed_soft,
        "{needed_soft} -> {needed_hard}"
    );
    assert!(hard.derivative_evals < 4 * needed_hard * HOST_STEPS);
    assert!(hard.accepted_steps < 3 * soft.accepted_steps);
}

/// Natural length of the tether, five body lengths.
const TETHER_M: f64 = 5.0 * SIZE_M;
/// Static stretch as a fraction of the natural length.
const STRETCH: f64 = 1.0e-3;

fn tether() -> Tether {
    let stiffness = MASS_KG * G / (STRETCH * TETHER_M);
    Tether {
        spec: WinchTetherSpec {
            stiffness_n_m: stiffness,
            damping_n_s_m: 2.0 * 0.2 * (stiffness * MASS_KG).sqrt(),
            max_length_m: 10.0 * TETHER_M,
            breaking_tension_n: 1.0e6 * MASS_KG * G,
            burn_through_energy_j: None,
            auto_cut_low_tension_n: None,
            auto_cut_dwell_s: 0.0,
        },
        anchor_a: TetherAnchor::Link {
            component: ComponentId(0),
            offset_local_m: DVec3::ZERO,
        },
        anchor_b: TetherAnchor::World {
            position_world_m: DVec3::ZERO,
            velocity_world_m_s: DVec3::ZERO,
        },
        initial_length_m: TETHER_M,
        command: TetherCommand::default(),
    }
}

fn tether_energy(line: &Tether) -> impl Fn(&ModelState) -> f64 + '_ {
    |state| {
        let base = state.base();
        let stretch = (base.position_world_m.length() - TETHER_M).max(0.0);
        0.5 * MASS_KG * base.linear_vel_world_m_s.length_squared()
            + MASS_KG * G * base.position_world_m.z
            + 0.5 * line.spec.stiffness_n_m * stretch * stretch
    }
}

/// Runs the tether problem from `start`; returns the Rosenbrock run and the
/// RK4 sub-steps needed.
fn tether_case(start: DVec3) -> (Run, u32) {
    let line = tether();
    let elements = (UniformGravity(DVec3::new(0.0, 0.0, -G)), line);
    let energy = tether_energy(&line);
    let stiff = run(&mut rosenbrock(), &elements, &(), &at(start), &energy);
    let explicit = run(&mut Rk4::new(), &elements, &(), &at(start), &energy);
    assert!(!stable(&explicit), "RK4 at the host step should diverge");
    assert!(
        stable(&stiff),
        "energy gain {:e} J",
        stiff.worst_energy_gain_j
    );
    // The line ends taut near its natural length.
    let length = stiff.end.position_world_m.length();
    assert!(
        (length / TETHER_M - 1.0).abs() < 5.0 * STRETCH,
        "length {length:e}"
    );
    (
        stiff,
        rk4_substeps_needed(&elements, &(), &at(start), &energy),
    )
}

#[test]
fn mote_hanging_on_a_taut_tether_is_stable_at_the_host_step() {
    // Released straight below the anchor, stretched to twice the static
    // stretch: a purely axial bob on the line.
    let start = DVec3::new(0.0, 0.0, -TETHER_M * (1.0 + 2.0 * STRETCH));
    let (stiff, needed) = tether_case(start);
    assert!(needed >= 4, "RK4 needed only {needed} sub-steps");
    assert!(stiff.accepted_steps <= HOST_STEPS + HOST_STEPS / 5);
}

#[test]
fn mote_swinging_on_a_taut_tether_is_stable_at_the_host_step() {
    // Released 30 degrees off the vertical with the line just taut. The
    // line turns by a large angle per host step, so the linearisation of a
    // step is only locally right and error control sub-steps to follow it.
    let start = DVec3::new(0.5, 0.0, -(0.75_f64.sqrt())) * TETHER_M;
    let (stiff, needed) = tether_case(start);
    assert!(needed >= 4, "RK4 needed only {needed} sub-steps");
    assert!(stiff.accepted_steps <= 10 * HOST_STEPS);
}
