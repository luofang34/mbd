//! A free articulated body driven only by its own joints, moving at orbital
//! speed: no external load acts, so total linear momentum and angular
//! momentum about the system centre of mass are constants of the motion.
//! Linear momentum is not a linear function of an articulated state, so
//! RK4 keeps it only to the truncation of the internal motion; at the step
//! chosen here that is far below 1e-13 of the momentum, which leaves
//! rounding as the only budget. Angular momentum is held to the step's
//! truncation, measured at rest.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
};
use mbd_multibody::{
    ElementContext, FloatingBaseState, ForceFn, JointDriveInput, Loads, ModelScale, ModelState,
    MultibodyError, MultibodyModel, Rk4, Workspace,
};

const STEP_S: f64 = 0.01;
const STEPS: usize = 1_000;
const TUMBLE_ROOT_RAD_S: DVec3 = DVec3::new(0.3, 0.35, -0.25);

fn orbital_velocity() -> DVec3 {
    7_700.0 * DVec3::new(-0.52, 0.31, 0.79).normalize()
}

/// Components with their masses and body inertias, in id order.
fn parts() -> [(ComponentId, f64, DMat3); 3] {
    [
        (
            ComponentId(0),
            200.0,
            DMat3::from_cols(
                DVec3::new(60.0, 4.0, -3.0),
                DVec3::new(4.0, 80.0, 6.0),
                DVec3::new(-3.0, 6.0, 95.0),
            ),
        ),
        (
            ComponentId(1),
            25.0,
            DMat3::from_diagonal(DVec3::new(1.2, 2.0, 0.9)),
        ),
        (
            ComponentId(2),
            8.0,
            DMat3::from_diagonal(DVec3::new(0.2, 0.3, 0.25)),
        ),
    ]
}

fn spec(
    placement: Placement,
    (_, mass, inertia): (ComponentId, f64, DMat3),
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement,
        dry_mass_kg: mass,
        dry_inertia_body: inertia,
        thermal_capacity_j_k: 1.0,
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

/// Body, an arm on a hinge and a slider on the arm. The travel limits are
/// out of reach, so no stop acts.
fn tree() -> ComponentTree {
    let [body, arm, slider] = parts();
    let hinge = JointSpec::revolute(DVec3::new(0.0, 0.6, 0.8), -1.0e3, 1.0e3);
    let slide = JointSpec::prismatic(DVec3::X, -1.0e3, 1.0e3);
    ComponentTree::new([
        (
            spec(Placement::ROOT, body, JointSpec::FIXED),
            posed(JointState::FIXED),
        ),
        (
            spec(
                Placement::child_of(ComponentId(0), DVec3::new(0.6, 0.2, 0.0)),
                arm,
                hinge,
            ),
            posed(JointState::revolute(0.3, 0.4)),
        ),
        (
            spec(
                Placement::child_of(ComponentId(1), DVec3::new(0.4, 0.0, 0.1)),
                slider,
                slide,
            ),
            posed(JointState::prismatic(0.15, -0.1)),
        ),
    ])
    .unwrap()
}

/// Internal drive only: a periodic hinge torque and a spring-damper on the
/// slider. Both act between neighbouring links, so neither changes the
/// system's momentum.
fn drives(ctx: &ElementContext<'_, ()>, loads: &mut Loads<'_>) -> Result<(), MultibodyError> {
    let torque = 3.0 * (0.7 * ctx.time_s).sin();
    loads.add_joint_drive(&JointDriveInput::scalar(ComponentId(1), torque))?;
    let slide = ctx.kinematics.joint(ComponentId(2))?;
    let force = -40.0 * (slide.q_scalar - 0.15) - 2.0 * slide.q_vel_scalar;
    loads.add_joint_drive(&JointDriveInput::scalar(ComponentId(2), force))
}

fn model() -> MultibodyModel {
    MultibodyModel::new(&tree(), &[], ModelScale::new(1.0, 233.0, 1.0).unwrap()).unwrap()
}

/// Tied to the present convention that the base linear rate is held in
/// root axes (as is `state.set_base` with its result).
fn base_moving_at(velocity_world_m_s: DVec3) -> FloatingBaseState {
    let rotation = DQuat::from_axis_angle(DVec3::new(0.5, 0.2, -0.84).normalize(), 0.9);
    FloatingBaseState {
        position_world_m: DVec3::new(1.0, -2.0, 3.0),
        rotation_world_from_root: rotation,
        linear_vel_root_m_s: rotation.inverse() * velocity_world_m_s,
        angular_vel_root_rad_s: TUMBLE_ROOT_RAD_S,
    }
}

/// Linear momentum and angular momentum about the centre of mass in world
/// axes, from link kinematics and the masses the test built. Angular
/// momentum is summed from positions and velocities relative to the centre
/// of mass, so it keeps its precision far from the origin and at speed.
#[derive(Clone, Copy, Debug)]
struct Momenta {
    linear: DVec3,
    angular: DVec3,
}

fn momenta(model: &MultibodyModel, state: &ModelState) -> Momenta {
    let mut work = Workspace::new();
    let kin = model.kinematics(state, &mut work).unwrap();
    let points: Vec<_> = parts()
        .iter()
        .map(|&(id, m, j)| (kin.point(id, DVec3::ZERO).unwrap(), m, j))
        .collect();
    let mass: f64 = points.iter().map(|p| p.1).sum();
    let linear = points
        .iter()
        .map(|(p, m, _)| p.velocity_world_m_s * *m)
        .sum::<DVec3>();
    let (r0, v0) = (points[0].0.position_world_m, points[0].0.velocity_world_m_s);
    let com_rel = points
        .iter()
        .map(|(p, m, _)| (p.position_world_m - r0) * *m)
        .sum::<DVec3>()
        / mass;
    let vel_rel = points
        .iter()
        .map(|(p, m, _)| (p.velocity_world_m_s - v0) * *m)
        .sum::<DVec3>()
        / mass;
    let angular = points
        .iter()
        .map(|(p, m, j)| {
            let r = p.rotation_world_from_link;
            let at = p.position_world_m - r0 - com_rel;
            let vel = p.velocity_world_m_s - v0 - vel_rel;
            r * *j * r.transpose() * p.angular_vel_world_rad_s + at.cross(vel * *m)
        })
        .sum();
    Momenta { linear, angular }
}

/// State whose centre of mass moves at `velocity_world_m_s`.
fn state_with_com_velocity(model: &MultibodyModel, velocity_world_m_s: DVec3) -> ModelState {
    let loads = ForceFn(drives);
    let mut state = model
        .state(&tree(), &base_moving_at(DVec3::ZERO), &loads, &())
        .unwrap();
    let mass: f64 = parts().iter().map(|p| p.1).sum();
    let internal = momenta(model, &state).linear / mass;
    state.set_base(&base_moving_at(velocity_world_m_s - internal));
    state
}

fn run(model: &MultibodyModel, velocity_world_m_s: DVec3, h: f64, steps: usize) -> Vec<Momenta> {
    let mut state = state_with_com_velocity(model, velocity_world_m_s);
    let (mut rk4, mut work, loads) = (Rk4::new(), Workspace::new(), ForceFn(drives));
    let mut out = vec![momenta(model, &state)];
    for i in 0..steps {
        model
            .advance(
                &mut rk4,
                &loads,
                &(),
                &mut state,
                i as f64 * h,
                h,
                &mut work,
            )
            .unwrap();
        out.push(momenta(model, &state));
    }
    out
}

fn max_change(track: &[Momenta], pick: fn(&Momenta) -> DVec3) -> f64 {
    let first = pick(&track[0]);
    track
        .iter()
        .map(|m| (pick(m) - first).length())
        .fold(0.0, f64::max)
}

#[test]
fn linear_momentum_is_constant_at_orbital_speed() {
    let model = model();
    let mass: f64 = parts().iter().map(|p| p.1).sum();
    let scale = mass * orbital_velocity().length();
    // At rest the drift is the truncation of the internal motion alone
    // (it falls sixteenfold per halving of the step: 4e-15 at this step);
    // it must leave the 1e-13 budget to rounding.
    let floor = max_change(&run(&model, DVec3::ZERO, STEP_S, STEPS), |m| m.linear) / scale;
    assert!(floor < 1.0e-14, "rest-frame drift {floor:e} of M|V|");
    // Rounding of a 7.7 km/s velocity is ~1e-16 relative per step; 1,000
    // steps of it, accumulated, stay within 1e-13.
    let drift = max_change(&run(&model, orbital_velocity(), STEP_S, STEPS), |m| {
        m.linear
    }) / scale;
    assert!(drift < 1.0e-13, "linear momentum drift {drift:e} of M|V|");
}

#[test]
fn angular_momentum_about_the_centre_of_mass_holds_to_truncation() {
    let model = model();
    let angular = |m: &Momenta| m.angular;
    // The truncation of the step does not depend on the uniform velocity:
    // measure it at rest at two smaller steps over the same span, check
    // RK4's fourth order, and extrapolate to the step used, with a factor
    // two for the higher-order remainder and 1e-13 for rounding.
    let rest = run(&model, DVec3::ZERO, STEP_S, STEPS);
    let l_scale = rest[0].angular.length();
    let half = max_change(&run(&model, DVec3::ZERO, STEP_S / 2.0, 2 * STEPS), angular) / l_scale;
    let quarter = max_change(&run(&model, DVec3::ZERO, STEP_S / 4.0, 4 * STEPS), angular) / l_scale;
    assert!(
        half > 8.0 * quarter,
        "not in RK4's asymptotic range: {half:e} at h/2, {quarter:e} at h/4"
    );
    let bound = 2.0 * 16.0 * half + 1.0e-13;
    let orbiting = run(&model, orbital_velocity(), STEP_S, STEPS);
    let drift = max_change(&orbiting, angular) / l_scale;
    // The same rotational problem at rest and at speed: the two histories
    // differ only by rounding over 1,000 steps. The momentum sum itself
    // differences link positions that reach 7.7e4 m (ulp 1.5e-11 m), worth
    // ~1e-12 of |L|; 1e-10 leaves room for that and the dynamics' rounding.
    let apart = rest
        .iter()
        .zip(&orbiting)
        .map(|(a, b)| (a.angular - b.angular).length() / l_scale)
        .fold(0.0, f64::max);
    assert!(
        drift < bound && apart < 1.0e-10,
        "angular momentum drift {drift:e} (truncation bound {bound:e}), \
         {apart:e} apart from the rest-frame run"
    );
}

#[test]
fn test_momenta_match_the_library_linear_sum() {
    let model = model();
    let state = state_with_com_velocity(&model, orbital_velocity());
    let ours = momenta(&model, &state);
    let lib = model
        .system_momentum(&state, &mut Workspace::new())
        .unwrap();
    let scale = lib.linear_world_kg_m_s.length();
    assert!((ours.linear - lib.linear_world_kg_m_s).length() < 1.0e-14 * scale);
    let mass: f64 = parts().iter().map(|p| p.1).sum();
    assert!((lib.linear_world_kg_m_s / mass - orbital_velocity()).length() < 1.0e-10);
}
