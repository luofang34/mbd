//! Newtonian mechanics is Galilean invariant: adding a constant velocity to
//! everything changes no acceleration and moves the trajectory by that
//! velocity times time. With loads that do not depend on absolute velocity,
//! a model evaluated at base velocity `v` and at `v + V` must agree on every
//! acceleration, and two integrations must stay `V t` apart, to rounding,
//! however large `V` is.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
    WrenchAtSite,
};
use mbd_multibody::{
    FloatingBaseState, JointLoadInput, ModelScale, ModelState, MultibodyModel, Rk4, UniformGravity,
    Workspace,
};

/// Orbital speed along a direction no body or world axis is aligned with.
fn boost() -> DVec3 {
    7_700.0 * DVec3::new(0.37, -0.61, 0.70).normalize()
}

const GRAVITY: DVec3 = DVec3::new(1.2, -3.1, -8.4);
const TUMBLE_ROOT_RAD_S: DVec3 = DVec3::new(0.3, -0.4, 0.25);
/// Characteristic length of the model, m.
const LENGTH_M: f64 = 1.0;

fn part(placement: Placement, mass: f64, inertia: DMat3, joint: JointSpec) -> ComponentSpec {
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

/// A body with products of inertia, a hinged arm and a slider on the arm.
fn tree() -> ComponentTree {
    let body_inertia = DMat3::from_cols(
        DVec3::new(120.0, 8.0, -5.0),
        DVec3::new(8.0, 160.0, 12.0),
        DVec3::new(-5.0, 12.0, 200.0),
    );
    let hinge = JointSpec::revolute(DVec3::new(0.2, 0.9, 0.3).normalize(), -1.0e3, 1.0e3);
    let slide = JointSpec::prismatic(DVec3::X, -1.0e3, 1.0e3);
    ComponentTree::new([
        (
            part(Placement::ROOT, 300.0, body_inertia, JointSpec::FIXED),
            posed(JointState::FIXED),
        ),
        (
            part(
                Placement::child_of(ComponentId(0), DVec3::new(0.8, 0.1, -0.2)),
                30.0,
                DMat3::from_diagonal(DVec3::new(2.0, 3.0, 1.5)),
                hinge,
            ),
            posed(JointState::revolute(0.4, 0.3)),
        ),
        (
            part(
                Placement::child_of(ComponentId(1), DVec3::new(0.3, 0.0, 0.1)),
                10.0,
                DMat3::from_diagonal(DVec3::new(0.5, 0.4, 0.6)),
                slide,
            ),
            posed(JointState::prismatic(0.1, -0.05)),
        ),
    ])
    .unwrap()
}

type Elements = (UniformGravity, JointLoadInput, JointLoadInput, WrenchAtSite);

/// Loads that do not depend on velocity: a uniform field, joint efforts
/// and a wrench fixed in the slider's axes.
fn elements() -> Elements {
    (
        UniformGravity(GRAVITY),
        JointLoadInput::scalar(ComponentId(1), 4.0),
        JointLoadInput::scalar(ComponentId(2), -1.5),
        WrenchAtSite {
            site: ComponentId(2),
            point_local_m: DVec3::new(0.1, 0.05, 0.0),
            force_local_n: DVec3::new(3.0, -2.0, 5.0),
            torque_local_nm: DVec3::new(0.4, 0.1, -0.3),
        },
    )
}

fn model() -> MultibodyModel {
    MultibodyModel::new(&tree(), &[], ModelScale::new(LENGTH_M, 340.0, 1.0).unwrap()).unwrap()
}

/// Tied to the present convention that the base linear rate is held in
/// root axes, with [`world_accel`]: the base state whose root origin moves
/// at `velocity_world_m_s`.
fn base_moving_at(velocity_world_m_s: DVec3) -> FloatingBaseState {
    let rotation = DQuat::from_axis_angle(DVec3::new(-0.4, 0.8, 0.45).normalize(), 1.3);
    FloatingBaseState {
        position_world_m: DVec3::new(10.0, -20.0, 5.0),
        rotation_world_from_root: rotation,
        linear_vel_root_m_s: rotation.inverse() * velocity_world_m_s,
        angular_vel_root_rad_s: TUMBLE_ROOT_RAD_S,
    }
}

fn state_moving_at(model: &MultibodyModel, velocity_world_m_s: DVec3) -> ModelState {
    let base = base_moving_at(velocity_world_m_s);
    model.state(&tree(), &base, &elements(), &()).unwrap()
}

/// World-axis accelerations from one derivative evaluation.
#[derive(Debug)]
struct WorldAccel {
    linear_m_s2: DVec3,
    angular_rad_s2: DVec3,
    hinge_rad_s2: f64,
    slide_m_s2: f64,
}

/// Also tied to the root-axis convention: the rate block holds the time
/// derivative of the root-axis components, so the inertial acceleration of
/// the root origin is `R (v̇_root + ω_root × v_root)`; `ω̇` maps by `R`
/// alone since `ω × ω` vanishes.
fn world_accel(model: &MultibodyModel, state: &ModelState) -> WorldAccel {
    let mut rates = vec![0.0; state.as_slice().len()];
    let x = state.as_slice();
    model
        .derivative(
            0.0,
            x,
            state.modes(),
            &elements(),
            &(),
            &mut Workspace::new(),
            &mut rates,
        )
        .unwrap();
    let nq = model.layout().position_len();
    let v = |at: usize| DVec3::new(rates[at], rates[at + 1], rates[at + 2]);
    let base = state.base();
    let r = base.rotation_world_from_root;
    WorldAccel {
        linear_m_s2: r * (v(nq) + base.angular_vel_root_rad_s.cross(base.linear_vel_root_m_s)),
        angular_rad_s2: r * v(nq + 3),
        hinge_rad_s2: rates[nq + 6],
        slide_m_s2: rates[nq + 7],
    }
}

#[test]
fn accelerations_do_not_depend_on_a_uniform_velocity() {
    let model = model();
    let v = DVec3::new(0.5, -0.2, 0.1);
    let rest = world_accel(&model, &state_moving_at(&model, v));
    let moving = world_accel(&model, &state_moving_at(&model, v + boost()));
    // Scales are rest-frame magnitudes, independent of the boost: for
    // translation |g| + |ω|² L, for rotation that over L.
    let w2 = TUMBLE_ROOT_RAD_S.length_squared();
    let linear_scale = GRAVITY.length() + w2 * LENGTH_M;
    let angular_scale = linear_scale / LENGTH_M;
    let rel = |a: f64, s: f64| a / s;
    let errors = [
        rel(
            (moving.linear_m_s2 - rest.linear_m_s2).length(),
            linear_scale,
        ),
        rel(
            (moving.angular_rad_s2 - rest.angular_rad_s2).length(),
            angular_scale,
        ),
        rel(
            (moving.hinge_rad_s2 - rest.hinge_rad_s2).abs(),
            angular_scale,
        ),
        rel((moving.slide_m_s2 - rest.slide_m_s2).abs(), linear_scale),
    ];
    assert!(
        errors.iter().all(|e| *e < 1.0e-12),
        "relative errors [linear, angular, hinge, slide] {errors:?}\nrest {rest:?}\nmoving {moving:?}"
    );
}

/// What two runs are compared on after every step.
struct Sample {
    position_m: DVec3,
    velocity_world_m_s: DVec3,
    attitude: DQuat,
    angular_world_rad_s: DVec3,
    joints: [f64; 4],
}

fn sample(model: &MultibodyModel, state: &ModelState) -> Sample {
    let base = state.base();
    let hinge = model.joint_state(state, ComponentId(1)).unwrap();
    let slide = model.joint_state(state, ComponentId(2)).unwrap();
    Sample {
        position_m: base.position_world_m,
        velocity_world_m_s: base.linear_vel_world_m_s(),
        attitude: base.rotation_world_from_root,
        angular_world_rad_s: base.angular_vel_world_rad_s(),
        joints: [
            hinge.q_scalar,
            hinge.q_vel_scalar,
            slide.q_scalar,
            slide.q_vel_scalar,
        ],
    }
}

fn integrate(
    model: &MultibodyModel,
    velocity_world_m_s: DVec3,
    h: f64,
    steps: usize,
) -> Vec<Sample> {
    let mut state = state_moving_at(model, velocity_world_m_s);
    let (mut rk4, mut work, loads) = (Rk4::new(), Workspace::new(), elements());
    let mut out = vec![sample(model, &state)];
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
        out.push(sample(model, &state));
    }
    out
}

#[test]
fn boosted_trajectory_is_the_rest_trajectory_moved_by_v_t() {
    const STEP_S: f64 = 0.5;
    const STEPS: usize = 100;
    let model = model();
    let v = DVec3::new(0.5, -0.2, 0.1);
    let rest = integrate(&model, v, STEP_S, STEPS);
    let moving = integrate(&model, v + boost(), STEP_S, STEPS);
    let (mut position, mut velocity, mut attitude, mut rates, mut joints) =
        (0.0, 0.0, 0.0, 0.0, 0.0);
    for (i, (a, b)) in rest.iter().zip(&moving).enumerate() {
        let t = i as f64 * STEP_S;
        position = f64::max(
            position,
            (b.position_m - a.position_m - boost() * t).length(),
        );
        velocity = f64::max(
            velocity,
            (b.velocity_world_m_s - a.velocity_world_m_s - boost()).length(),
        );
        let relative = b.attitude * a.attitude.inverse();
        attitude = f64::max(
            attitude,
            DVec3::new(relative.x, relative.y, relative.z).length(),
        );
        rates = f64::max(
            rates,
            (b.angular_world_rad_s - a.angular_world_rad_s).length(),
        );
        for (p, q) in a.joints.iter().zip(&b.joints) {
            joints = f64::max(joints, (p - q).abs());
        }
    }
    // Rounding, plus what the derivative check allows (1e-12 of the
    // ~9 m/s² and ~9 rad/s² scales) carried over T = 50 s. The boosted
    // position reaches |V| T = 3.9e5 m, whose ulp is 6e-11 m: a few ulp per
    // step over 100 steps is ~1e-8 m, and 1e-11 m/s² over T adds ~1e-8 m.
    // The boosted velocity's ulp is 9e-13 m/s, a few hundred of which over
    // 100 steps of rotating it between axes approach 1e-9 m/s, and
    // 1e-11 m/s² over T adds 5e-10 m/s. Angular rate (and rate-like joint
    // states) take 1e-11 per second over T, 5e-10, plus rounding of the
    // same order; attitude and joint coordinates its integral, ~1e-8.
    let ok = position < 1.0e-7
        && velocity < 1.0e-8
        && attitude < 1.0e-8
        && rates < 2.0e-9
        && joints < 1.0e-8;
    assert!(
        ok,
        "beyond V t: position {position:e} m, world velocity {velocity:e} m/s, \
         attitude sin(θ/2) {attitude:e}, angular rate {rates:e} rad/s, joint states {joints:e}"
    );
}
