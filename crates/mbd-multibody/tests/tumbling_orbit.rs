//! A tumbling body in circular low orbit, integrated with a fixed 1 s RK4
//! step, must keep its centre of mass on the orbit a point mass flies with
//! the same step, and keep its angular momentum about the centre of mass to
//! what the step size alone costs. Gravity acts at the centre of mass, so
//! translation and rotation are decoupled in the exact dynamics; a sound
//! formulation keeps them decoupled numerically at any orbital speed.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
};
use mbd_multibody::{
    ElementContext, FloatingBaseState, ForceElement, ForceFn, JointDriveInput, Loads, ModelScale,
    ModelState, MultibodyError, MultibodyModel, Rk4, Workspace,
};

/// Earth gravitational parameter, m³/s².
const MU: f64 = 3.986_004_418e14;
/// Orbit radius: 400 km altitude.
const RADIUS_M: f64 = 6.778e6;
const STEP_S: f64 = 1.0;
const STEPS: usize = 5_000;
const BODY_KG: f64 = 1_000.0;
const ROTOR_KG: f64 = 20.0;
/// Rotor hub on the body; the rotor spins about its own symmetry axis
/// through its centre of mass, so the system centre of mass stays fixed in
/// the body but off the root origin.
const ROTOR_AT_M: DVec3 = DVec3::new(0.5, -0.2, 0.3);

/// Body inertia with products of inertia, so no body axis is principal.
fn body_inertia() -> DMat3 {
    DMat3::from_cols(
        DVec3::new(800.0, 40.0, -25.0),
        DVec3::new(40.0, 1_200.0, 60.0),
        DVec3::new(-25.0, 60.0, 1_500.0),
    )
}

fn rotor_inertia() -> DMat3 {
    DMat3::from_diagonal(DVec3::new(0.4, 0.4, 0.8))
}

/// Tumble about a non-principal axis, about 0.1 rad/s.
const TUMBLE_ROOT_RAD_S: DVec3 = DVec3::new(0.06, -0.05, 0.07);

fn attitude() -> DQuat {
    DQuat::from_axis_angle(DVec3::new(0.3, -0.7, 0.65).normalize(), 2.1)
}

/// Components with their masses and body inertias, in id order.
struct Body {
    tree: ComponentTree,
    parts: Vec<(ComponentId, f64, DMat3)>,
}

fn component(placement: Placement, mass: f64, inertia: DMat3, joint: JointSpec) -> ComponentSpec {
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

fn lone_body() -> Body {
    let spec = component(Placement::ROOT, BODY_KG, body_inertia(), JointSpec::FIXED);
    Body {
        tree: ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap(),
        parts: vec![(ComponentId(0), BODY_KG, body_inertia())],
    }
}

/// The body with a rotor on a revolute joint spinning at 3 rad/s. The
/// travel limits are far beyond what 5,000 s of spin reaches, so the stop
/// never acts.
fn body_with_rotor() -> Body {
    let hinge = JointSpec::revolute(DVec3::Z, -1.0e9, 1.0e9);
    let rotor = component(
        Placement::child_of(ComponentId(0), ROTOR_AT_M),
        ROTOR_KG,
        rotor_inertia(),
        hinge,
    );
    let root = component(Placement::ROOT, BODY_KG, body_inertia(), JointSpec::FIXED);
    let spinning = ComponentState {
        joint: JointState::revolute(0.0, 3.0),
        ..ComponentState::at_temperature(293.15)
    };
    Body {
        tree: ComponentTree::new([
            (root, ComponentState::at_temperature(293.15)),
            (rotor, spinning),
        ])
        .unwrap(),
        parts: vec![
            (ComponentId(0), BODY_KG, body_inertia()),
            (ComponentId(1), ROTOR_KG, rotor_inertia()),
        ],
    }
}

/// Centre of mass, its velocity and the angular momentum about it, all in
/// world axes, from link kinematics and the masses the test built. This
/// does not rely on the state layout or on the library's own momentum sum.
#[derive(Clone, Copy, Debug)]
struct Totals {
    com_m: DVec3,
    com_vel_m_s: DVec3,
    angular_nms: DVec3,
}

fn totals(
    model: &MultibodyModel,
    state: &ModelState,
    parts: &[(ComponentId, f64, DMat3)],
) -> Totals {
    let mut work = Workspace::new();
    let kin = model.kinematics(state, &mut work).unwrap();
    let points: Vec<_> = parts
        .iter()
        .map(|&(id, m, j)| (kin.point(id, DVec3::ZERO).unwrap(), m, j))
        .collect();
    let mass: f64 = points.iter().map(|p| p.1).sum();
    let com = points
        .iter()
        .map(|(p, m, _)| p.position_world_m * *m)
        .sum::<DVec3>()
        / mass;
    let vel = points
        .iter()
        .map(|(p, m, _)| p.velocity_world_m_s * *m)
        .sum::<DVec3>()
        / mass;
    let angular = points
        .iter()
        .map(|(p, m, j)| {
            let r = p.rotation_world_from_link;
            r * *j * r.transpose() * p.angular_vel_world_rad_s
                + (p.position_world_m - com).cross((p.velocity_world_m_s - vel) * *m)
        })
        .sum();
    Totals {
        com_m: com,
        com_vel_m_s: vel,
        angular_nms: angular,
    }
}

/// Uniform field evaluated at the system centre of mass: the whole weight
/// acts there, so it exerts no torque about it.
fn central_gravity(
    parts: Vec<(ComponentId, f64)>,
) -> impl Fn(&ElementContext<'_, ()>, &mut Loads<'_>) -> Result<(), MultibodyError> {
    move |ctx, loads| {
        let mass: f64 = parts.iter().map(|p| p.1).sum();
        let mut first = DVec3::ZERO;
        for &(id, m) in &parts {
            first += ctx.kinematics.point(id, DVec3::ZERO)?.position_world_m * m;
        }
        let r = first / mass;
        loads.add_gravity(-MU * r / (r.length_squared() * r.length()));
        Ok(())
    }
}

/// Periodic internal torque on the rotor: it changes the spin but not the
/// system's momentum.
fn rotor_drive(ctx: &ElementContext<'_, ()>, loads: &mut Loads<'_>) -> Result<(), MultibodyError> {
    let torque = 0.02 * (0.05 * ctx.time_s).sin();
    loads.add_joint_drive(&JointDriveInput::scalar(ComponentId(1), torque))
}

/// Tied to the present convention that the base linear rate is held in
/// root axes (as are the direct writes of `angular_vel_root_rad_s`): the
/// base state whose root origin moves at `velocity_world_m_s`.
fn base_moving_at(position: DVec3, velocity_world_m_s: DVec3) -> FloatingBaseState {
    let rotation = attitude();
    FloatingBaseState {
        position_world_m: position,
        rotation_world_from_root: rotation,
        linear_vel_root_m_s: rotation.inverse() * velocity_world_m_s,
        angular_vel_root_rad_s: TUMBLE_ROOT_RAD_S,
    }
}

fn orbit_start() -> (DVec3, DVec3) {
    let speed = (MU / RADIUS_M).sqrt();
    // Inclined plane, so no world or body axis is special.
    let along = DVec3::new(
        0.0,
        51.6_f64.to_radians().cos(),
        51.6_f64.to_radians().sin(),
    );
    (DVec3::new(RADIUS_M, 0.0, 0.0), along * speed)
}

fn model_of(body: &Body) -> MultibodyModel {
    let scale = ModelScale::new(1.0, BODY_KG, 1.0).unwrap();
    MultibodyModel::new(&body.tree, &[], scale).unwrap()
}

/// Runs `steps` RK4 steps of `h` and returns the totals before every step
/// and after the last.
fn run<E: ForceElement<()>>(
    body: &Body,
    elements: &E,
    base: &FloatingBaseState,
    h: f64,
    steps: usize,
) -> Vec<Totals> {
    let model = model_of(body);
    let mut state = model.state(&body.tree, base, elements, &()).unwrap();
    let (mut rk4, mut work) = (Rk4::new(), Workspace::new());
    let mut out = vec![totals(&model, &state, &body.parts)];
    for i in 0..steps {
        model
            .advance(
                &mut rk4,
                elements,
                &(),
                &mut state,
                i as f64 * h,
                h,
                &mut work,
            )
            .unwrap();
        out.push(totals(&model, &state, &body.parts));
    }
    out
}

/// Classical RK4 on `r'' = -μ r / |r|³` in world axes with step `h`.
fn point_mass_orbit(r0: DVec3, v0: DVec3, h: f64, steps: usize) -> Vec<DVec3> {
    let accel = |r: DVec3| -MU * r / (r.length_squared() * r.length());
    let (mut r, mut v) = (r0, v0);
    let mut out = vec![r];
    for _ in 0..steps {
        let (k1r, k1v) = (v, accel(r));
        let (k2r, k2v) = (v + 0.5 * h * k1v, accel(r + 0.5 * h * k1r));
        let (k3r, k3v) = (v + 0.5 * h * k2v, accel(r + 0.5 * h * k2r));
        let (k4r, k4v) = (v + h * k3v, accel(r + h * k3r));
        r += h / 6.0 * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
        v += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        out.push(r);
    }
    out
}

fn mass_ids(body: &Body) -> Vec<(ComponentId, f64)> {
    body.parts.iter().map(|&(id, m, _)| (id, m)).collect()
}

/// Largest distance between the body's centre of mass and the point-mass
/// orbit started from the same centre-of-mass position and velocity.
/// Base state that puts the centre of mass at `r` moving at `v`.
fn com_placed<E: ForceElement<()>>(
    body: &Body,
    elements: &E,
    r: DVec3,
    v: DVec3,
) -> FloatingBaseState {
    let model = model_of(body);
    let probe = model
        .state(&body.tree, &base_moving_at(r, v), elements, &())
        .unwrap();
    let t = totals(&model, &probe, &body.parts);
    base_moving_at(r + (r - t.com_m), v + (v - t.com_vel_m_s))
}

fn orbit_error<E: ForceElement<()>>(body: &Body, elements: &E) -> f64 {
    let (r0, v0) = orbit_start();
    let base = com_placed(body, elements, r0, v0);
    let track = run(body, elements, &base, STEP_S, STEPS);
    let start = track[0];
    let reference = point_mass_orbit(start.com_m, start.com_vel_m_s, STEP_S, STEPS);
    track
        .iter()
        .zip(&reference)
        .map(|(a, r)| (a.com_m - *r).length())
        .fold(0.0, f64::max)
}

// Bound for the orbit checks. With gravity at the centre of mass the centre
// of mass obeys exactly the point-mass equation, whatever the body does
// about it. For the lone body the root origin is the centre of mass, so a
// formulation that integrates the base translation in world axes performs
// the reference's arithmetic up to rounding: ~1e-9 m per step at 6.8e6 m,
// which over 5,000 steps of an orbit stays near 1e-7 m (the non-tumbling
// run, whose root-axis velocity is a fixed rotation of the world velocity,
// shows exactly that). With the rotor the centre of mass sits 1 cm off the
// root origin; the attitude's RK4 truncation (~1e-6 relative after 5,000
// steps) moves it by ~1e-8 m. 1e-5 m leaves two orders of margin over both.
const ORBIT_BOUND_M: f64 = 1.0e-5;

#[test]
fn tumbling_body_follows_the_point_mass_orbit() {
    let body = lone_body();
    let error = orbit_error(&body, &ForceFn(central_gravity(mass_ids(&body))));
    assert!(
        error < ORBIT_BOUND_M,
        "centre of mass off the point-mass orbit by {error:e} m"
    );
}

#[test]
fn tumbling_body_with_driven_rotor_follows_the_point_mass_orbit() {
    let body = body_with_rotor();
    let elements = (
        ForceFn(central_gravity(mass_ids(&body))),
        ForceFn(rotor_drive),
    );
    // The attitude truncation that moves the off-origin centre of mass is
    // independent of orbital speed: at rest it must sit far below the bound,
    // or the bound would not measure the orbit alone.
    let drive = ForceFn(rotor_drive);
    let at_rest = com_placed(&body, &drive, DVec3::ZERO, DVec3::ZERO);
    let resting = run(&body, &drive, &at_rest, STEP_S, STEPS);
    let floor = resting
        .iter()
        .map(|t| (t.com_m - resting[0].com_m).length())
        .fold(0.0, f64::max);
    assert!(
        floor < 1.0e-2 * ORBIT_BOUND_M,
        "rest-frame drift {floor:e} m"
    );
    let error = orbit_error(&body, &elements);
    assert!(
        error < ORBIT_BOUND_M,
        "centre of mass off the point-mass orbit by {error:e} m"
    );
}

/// Largest change of angular momentum about the centre of mass over a run,
/// relative to its initial magnitude.
fn angular_drift(track: &[Totals]) -> f64 {
    let l0 = track[0].angular_nms;
    track
        .iter()
        .map(|t| (t.angular_nms - l0).length() / l0.length())
        .fold(0.0, f64::max)
}

fn no_loads(_: &ElementContext<'_, ()>, _: &mut Loads<'_>) -> Result<(), MultibodyError> {
    Ok(())
}

#[test]
fn tumbling_orbit_conserves_angular_momentum_to_truncation() {
    let body = lone_body();
    let free = ForceFn(no_loads);
    // The rotational equations of a torque-free body do not involve its
    // translation, so the truncation of the 1 s step is measured at rest,
    // where no large velocity enters the arithmetic: two smaller steps over
    // the same span, which must show RK4's fourth order before they are
    // extrapolated to 1 s.
    let at_rest = base_moving_at(DVec3::ZERO, DVec3::ZERO);
    let half = angular_drift(&run(&body, &free, &at_rest, 0.5, 2 * STEPS));
    let quarter = angular_drift(&run(&body, &free, &at_rest, 0.25, 4 * STEPS));
    assert!(
        half > 8.0 * quarter,
        "not in RK4's asymptotic range: {half:e} at 0.5 s, {quarter:e} at 0.25 s"
    );
    // Sixteen times the half-step drift is the 1 s truncation; a factor two
    // covers the higher-order remainder, and 1e-12 the rounding floor of
    // 5,000 steps.
    let bound = 2.0 * 16.0 * half + 1.0e-12;
    let (r0, v0) = orbit_start();
    let orbiting = run(
        &body,
        &ForceFn(central_gravity(mass_ids(&body))),
        &base_moving_at(r0, v0),
        STEP_S,
        STEPS,
    );
    let drift = angular_drift(&orbiting);
    assert!(
        drift < bound,
        "angular momentum drift {drift:e}, truncation bound {bound:e}"
    );
    // Same step, same rotational problem: at rest and in orbit the angular
    // momentum histories differ only by rounding, ~1e-16 per operation over
    // 5,000 steps.
    let resting = run(&body, &free, &at_rest, STEP_S, STEPS);
    let l_scale = resting[0].angular_nms.length();
    let apart = resting
        .iter()
        .zip(&orbiting)
        .map(|(a, b)| (a.angular_nms - b.angular_nms).length() / l_scale)
        .fold(0.0, f64::max);
    assert!(
        apart < 1.0e-10,
        "orbit and rest angular momentum differ by {apart:e} relative"
    );
}

/// Momentum of the rotor model at a base moving at `v` from `r`, by the
/// test's sum and by the library's.
fn both_sums(r: DVec3, v: DVec3) -> (Totals, mbd_multibody::SystemMomentum) {
    let body = body_with_rotor();
    let model = model_of(&body);
    let state = model
        .state(&body.tree, &base_moving_at(r, v), &ForceFn(no_loads), &())
        .unwrap();
    let lib = model
        .system_momentum(&state, &mut Workspace::new())
        .unwrap();
    (totals(&model, &state, &body.parts), lib)
}

#[test]
fn test_sums_match_the_library_near_the_origin() {
    let (ours, lib) = both_sums(DVec3::new(3.0, -2.0, 1.0), DVec3::new(0.4, 0.1, -0.3));
    let mass = BODY_KG + ROTOR_KG;
    assert!((ours.com_m - lib.com_world_m).length() < 1.0e-12);
    assert!((ours.com_vel_m_s * mass - lib.linear_world_kg_m_s).length() < 1.0e-9);
    assert!((ours.angular_nms - lib.angular_about_com_world_nms).length() < 1.0e-11);
    // The lone body's centre of mass is its root origin.
    let lone = lone_body();
    let model = model_of(&lone);
    let (r0, v0) = orbit_start();
    let state = model
        .state(&lone.tree, &base_moving_at(r0, v0), &ForceFn(no_loads), &())
        .unwrap();
    assert!((totals(&model, &state, &lone.parts).com_m - r0).length() < 1.0e-9);
}

#[test]
fn library_angular_momentum_keeps_precision_in_orbit() {
    // Angular momentum about the centre of mass depends only on motion
    // relative to it. Summed from absolute positions (6.8e6 m, ulp ~1e-9 m)
    // and absolute momenta (~8e6 kg·m/s) it carries ~1e-2 N·m·s of rounding
    // on ~120 N·m·s; summed from relative quantities, as the test does, it
    // stays within ~1e-8. The bound is relative 1e-10 of |L|.
    let (r0, v0) = orbit_start();
    let (ours, lib) = both_sums(r0, v0);
    let error = (ours.angular_nms - lib.angular_about_com_world_nms).length();
    let bound = 1.0e-10 * ours.angular_nms.length();
    assert!(
        error < bound,
        "library angular momentum off by {error:e} N·m·s, bound {bound:e}"
    );
}

#[test]
fn non_tumbling_body_follows_the_point_mass_orbit() {
    // Without rotation the root-axis velocity is a fixed rotation of the
    // world velocity, so any formulation reproduces the reference to
    // rounding; this checks the harness and the reference integrator.
    let body = lone_body();
    let (r0, v0) = orbit_start();
    let mut base = base_moving_at(r0, v0);
    base.angular_vel_root_rad_s = DVec3::ZERO;
    let track = run(
        &body,
        &ForceFn(central_gravity(mass_ids(&body))),
        &base,
        STEP_S,
        STEPS,
    );
    let reference = point_mass_orbit(track[0].com_m, track[0].com_vel_m_s, STEP_S, STEPS);
    let error = track
        .iter()
        .zip(&reference)
        .map(|(a, r)| (a.com_m - *r).length())
        .fold(0.0, f64::max);
    assert!(
        error < ORBIT_BOUND_M,
        "non-tumbling body off the orbit by {error:e} m"
    );
}
