//! The base's translation: held in world axes in the state, integrated as
//! the centre of mass's, and reported as an inertial acceleration. Also the
//! one force that depends on a velocity, that of added inertia, which
//! follows the motion relative to its medium.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
};
use mbd_multibody::{
    ElementContext, FloatingBaseState, ForceFn, Loads, MediumMotion, ModelScale, ModelState,
    MultiBodyDynamics, MultibodyError, MultibodyModel, SpatialMat6, UniformGravity, Workspace,
};

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

/// A body with an arm on a moving hinge.
fn arm_tree() -> ComponentTree {
    let hinge = JointSpec::revolute(DVec3::new(0.0, 0.6, 0.8), -1.0e3, 1.0e3);
    let swinging = ComponentState {
        joint: JointState::revolute(0.4, -0.7),
        ..ComponentState::at_temperature(293.15)
    };
    ComponentTree::new([
        (
            part(
                Placement::ROOT,
                50.0,
                DMat3::from_diagonal(DVec3::new(4.0, 5.0, 6.0)),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            part(
                Placement::child_of(ComponentId(0), DVec3::new(0.7, -0.2, 0.3)),
                12.0,
                DMat3::from_diagonal(DVec3::new(0.3, 0.5, 0.4)),
                hinge,
            ),
            swinging,
        ),
    ])
    .unwrap()
}

fn tumbling_base() -> FloatingBaseState {
    FloatingBaseState {
        position_world_m: DVec3::new(6.0e6, -2.0e6, 1.0e6),
        rotation_world_from_root: DQuat::from_axis_angle(
            DVec3::new(0.2, 0.5, -0.8).normalize(),
            1.1,
        ),
        linear_vel_world_m_s: DVec3::new(-2_000.0, 6_500.0, 3_000.0),
        angular_vel_root_rad_s: DVec3::new(0.3, -0.2, 0.4),
    }
}

fn arm_model() -> (MultibodyModel, ModelState) {
    let tree = arm_tree();
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, 62.0, 1.0).unwrap()).unwrap();
    let state = model.state(&tree, &tumbling_base(), &(), &()).unwrap();
    (model, state)
}

#[test]
fn centre_of_mass_agrees_with_the_momentum_sum_and_the_link_masses() {
    let (model, state) = arm_model();
    let mut work = Workspace::new();
    let momentum = model.system_momentum(&state, &mut work).unwrap();
    let kin = model.kinematics(&state, &mut work).unwrap();
    let com = kin.centre_of_mass().unwrap();
    assert_eq!(com.mass_kg, momentum.total_mass_kg);
    assert_eq!(com.position_world_m, momentum.com_world_m);
    assert_eq!(
        com.velocity_world_m_s * com.mass_kg,
        momentum.linear_world_kg_m_s
    );
    assert_eq!(kin.link_mass(ComponentId(0)).unwrap(), (50.0, DVec3::ZERO));
    assert_eq!(kin.link_mass(ComponentId(1)).unwrap(), (12.0, DVec3::ZERO));
    // The same centre from the links' own points.
    let (body, arm) = (
        kin.point(ComponentId(0), DVec3::ZERO).unwrap(),
        kin.point(ComponentId(1), DVec3::ZERO).unwrap(),
    );
    let expected = (body.position_world_m * 50.0 + arm.position_world_m * 12.0) / 62.0;
    assert!((com.position_world_m - expected).length() < 1.0e-8);
    assert_eq!(
        com.position_world_m,
        state.base().position_world_m + com.offset_from_root_world_m
    );
    assert_eq!(
        com.velocity_world_m_s,
        state.base().linear_vel_world_m_s + com.velocity_from_root_world_m_s
    );
}

#[test]
fn centre_of_mass_coordinates_round_trip() {
    let (model, state) = arm_model();
    let mut work = Workspace::new();
    let com = model
        .kinematics(&state, &mut work)
        .unwrap()
        .centre_of_mass()
        .unwrap();
    let nq = model.layout().position_len();
    let mut x = state.as_slice().to_vec();
    model.root_to_centre_of_mass(&mut x, &mut work).unwrap();
    assert_eq!(DVec3::from_slice(&x[..3]), com.position_world_m);
    assert_eq!(DVec3::from_slice(&x[nq..nq + 3]), com.velocity_world_m_s);
    let base = model.centre_of_mass_to_root(&mut x, &mut work).unwrap();
    // One rounding of a 6e6 m position and of a 7 km/s velocity each way.
    let back = state.base();
    assert!((base.position_world_m - back.position_world_m).length() < 4.0e-9);
    assert!((base.linear_vel_world_m_s - back.linear_vel_world_m_s).length() < 4.0e-12);
    for (k, (a, b)) in x.iter().zip(state.as_slice()).enumerate() {
        let translation = k < 3 || (nq..nq + 3).contains(&k);
        assert!(translation || a == b, "component {k} changed");
    }
}

#[test]
fn zero_length_step_leaves_every_bit_of_the_state() {
    let (model, mut state) = arm_model();
    let before = state.clone();
    let report = model
        .advance(
            &mut mbd_multibody::Rk4::new(),
            &(),
            &(),
            &mut state,
            3.0,
            0.0,
            &mut Workspace::new(),
        )
        .unwrap();
    assert_eq!(report.end_time_s, 3.0);
    assert_eq!(state, before);
}

#[test]
fn linear_rate_block_is_the_inertial_acceleration_of_the_root_origin() {
    let (model, state) = arm_model();
    let gravity = DVec3::new(0.4, -9.0, 2.0);
    let (mut work, mut rates) = (Workspace::new(), vec![0.0; state.as_slice().len()]);
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            &UniformGravity(gravity),
            &(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    let nq = model.layout().position_len();
    let base = state.base();
    assert_eq!(DVec3::from_slice(&rates[..3]), base.linear_vel_world_m_s);
    let accel = work.base_linear_accel_world_m_s2();
    assert_eq!(DVec3::from_slice(&rates[nq..nq + 3]), accel);
    assert_eq!(
        DVec3::from_slice(&rates[nq + 3..nq + 6]),
        work.base_angular_accel_root_rad_s2()
    );
    assert!(!work.used_rigid_fast_path());
    let one_shot =
        MultiBodyDynamics::forward_dynamics(&arm_tree(), &[], &base, gravity, &[], &[]).unwrap();
    assert_eq!(one_shot.base_linear_accel_world_m_s2, accel);
    // Free fall: the centre of mass accelerates at exactly the field. Its
    // acceleration is the root origin's plus the second derivative of the
    // offset, differenced here from the first.
    let h = 1.0e-4;
    let offset_rate = |sign: f64| {
        let mut x = state.as_slice().to_vec();
        for (xi, ri) in x.iter_mut().zip(&rates) {
            *xi += sign * h * ri;
        }
        model.layout().retract(&mut x);
        let mut moved = state.clone();
        moved.as_mut_slice().copy_from_slice(&x);
        let mut work = Workspace::new();
        let kin = model.kinematics(&moved, &mut work).unwrap();
        kin.centre_of_mass().unwrap().velocity_from_root_world_m_s
    };
    let offset_accel = (offset_rate(1.0) - offset_rate(-1.0)) / (2.0 * h);
    assert!((accel + offset_accel - gravity).length() < 1.0e-6);
}

/// Entrained fluid that resists motion along the body's axes unequally.
const ADDED_KG: DVec3 = DVec3::new(3.0, 9.0, 20.0);

fn entrained(_: &ElementContext<'_, ()>, loads: &mut Loads<'_>) -> Result<(), MultibodyError> {
    let added = SpatialMat6 {
        ll: DMat3::from_diagonal(ADDED_KG),
        ..SpatialMat6::ZERO
    };
    loads.add_inertia(ComponentId(0), added, &MediumMotion::AT_REST)
}

#[test]
fn added_inertia_in_a_medium_at_rest_obeys_kirchhoff() {
    // Kirchhoff's equations for a body in a fluid at rest, body axes:
    // (m + A) v̇ + ω × (m + A) v = 0 and J ω̇ + ω × J ω + v × A v = 0.
    let (mass, inertia) = (5.0, DMat3::from_diagonal(DVec3::new(2.0, 3.0, 4.0)));
    let spec = part(Placement::ROOT, mass, inertia, JointSpec::FIXED);
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, mass, 1.0).unwrap()).unwrap();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(1.0, 2.0, 3.0),
        rotation_world_from_root: DQuat::from_axis_angle(
            DVec3::new(0.6, -0.3, 0.74).normalize(),
            0.8,
        ),
        linear_vel_world_m_s: DVec3::new(4.0, -7.0, 2.5),
        angular_vel_root_rad_s: DVec3::new(0.5, 0.2, -0.3),
    };
    let fluid = ForceFn(entrained);
    let state = model.state(&tree, &base, &fluid, &()).unwrap();
    let (mut work, mut rates) = (Workspace::new(), vec![0.0; state.as_slice().len()]);
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            &fluid,
            &(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    let (v, w) = (base.linear_vel_root_m_s(), base.angular_vel_root_rad_s);
    let total = DVec3::splat(mass) + ADDED_KG;
    let v_dot = -w.cross(total * v) / total;
    let w_dot = -inertia.inverse() * (w.cross(inertia * w) + v.cross(ADDED_KG * v));
    // The rate of the root-axis components and the inertial acceleration
    // differ by the turning of the axes.
    let expected = base.rotation_world_from_root * (v_dot + w.cross(v));
    assert!((work.base_linear_accel_world_m_s2() - expected).length() < 1.0e-12);
    assert!((work.base_angular_accel_root_rad_s2() - w_dot).length() < 1.0e-12);
}

/// Entrained fluid with coupled blocks, belonging to a medium that turns
/// at `spin` about the world origin and drifts at `drift`.
fn entrained_in(
    spin: DVec3,
    drift: DVec3,
) -> impl Fn(&ElementContext<'_, ()>, &mut Loads<'_>) -> Result<(), MultibodyError> {
    move |ctx, loads| {
        let at = ctx.kinematics.point(ComponentId(0), DVec3::ZERO)?;
        let added = SpatialMat6 {
            aa: DMat3::from_diagonal(DVec3::new(0.4, 0.1, 0.7)),
            ll: DMat3::from_diagonal(ADDED_KG),
            ..SpatialMat6::ZERO
        };
        let medium = MediumMotion {
            velocity_world_m_s: drift + spin.cross(at.position_world_m),
            angular_velocity_world_rad_s: spin,
        };
        loads.add_inertia(ComponentId(0), added, &medium)
    }
}

/// Base accelerations `[linear world, angular root]` of a 5 kg body with
/// spherical inertia under `elements`.
fn ball_accel<E: mbd_multibody::ForceElement<()>>(
    base: &FloatingBaseState,
    elements: &E,
) -> [DVec3; 2] {
    let inertia = DMat3::from_diagonal(DVec3::splat(2.0));
    let spec = part(Placement::ROOT, 5.0, inertia, JointSpec::FIXED);
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, 5.0, 1.0).unwrap()).unwrap();
    let state = model.state(&tree, base, elements, &()).unwrap();
    let (mut work, mut rates) = (Workspace::new(), vec![0.0; state.as_slice().len()]);
    let x = state.as_slice();
    model
        .derivative(0.0, x, state.modes(), elements, &(), &mut work, &mut rates)
        .unwrap();
    [
        work.base_linear_accel_world_m_s2(),
        work.base_angular_accel_root_rad_s2(),
    ]
}

#[test]
fn body_and_medium_boosted_together_accelerate_as_at_rest() {
    let rotation = DQuat::from_axis_angle(DVec3::new(0.6, -0.3, 0.74).normalize(), 0.8);
    let moving_at = |velocity| FloatingBaseState {
        position_world_m: DVec3::new(1.0, 2.0, 3.0),
        rotation_world_from_root: rotation,
        linear_vel_world_m_s: velocity,
        angular_vel_root_rad_s: DVec3::new(0.5, 0.2, -0.3),
    };
    let through = DVec3::new(4.0, -7.0, 2.5);
    let boost = 7_700.0 * DVec3::new(0.37, -0.61, 0.70).normalize();
    let rest = ball_accel(
        &moving_at(through),
        &ForceFn(entrained_in(DVec3::ZERO, DVec3::ZERO)),
    );
    let boosted = ball_accel(
        &moving_at(through + boost),
        &ForceFn(entrained_in(DVec3::ZERO, boost)),
    );
    // The fluid does act, and the two cases differ only by the rounding of
    // subtracting the boost from a 7.7 km/s velocity (1e-12 m/s of 8 m/s).
    assert!(rest[0].length() > 1.0 && rest[1].length() > 1.0);
    for (a, b) in rest.iter().zip(&boosted) {
        assert!((*a - *b).length() < 1.0e-10 * a.length(), "{a:?} vs {b:?}");
    }
}

#[test]
fn body_turning_with_its_medium_feels_no_added_inertia() {
    let spin = DVec3::new(0.02, -0.03, 0.05);
    let position = DVec3::new(2_000.0, -500.0, 300.0);
    let rotation = DQuat::from_axis_angle(DVec3::new(0.3, -0.5, 0.8).normalize(), 0.7);
    let base = FloatingBaseState {
        position_world_m: position,
        rotation_world_from_root: rotation,
        linear_vel_world_m_s: spin.cross(position),
        angular_vel_root_rad_s: rotation.conjugate() * spin,
    };
    // The force that holds the body on its circle with the medium.
    let centripetal = spin.cross(spin.cross(position));
    let support = mbd_core::WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: rotation.conjugate() * centripetal * 5.0,
        torque_local_nm: DVec3::ZERO,
    };
    let alone = ball_accel(&base, &support);
    assert!((alone[0] - centripetal).length() < 1.0e-13 * centripetal.length());
    let turning = ball_accel(&base, &(support, ForceFn(entrained_in(spin, DVec3::ZERO))));
    assert!((turning[0] - alone[0]).length() < 1.0e-12 * centripetal.length());
    assert!((turning[1] - alone[1]).length() < 1.0e-12 * spin.length_squared());
    // In a medium at rest the same motion is resisted.
    let resting = ball_accel(
        &base,
        &(support, ForceFn(entrained_in(DVec3::ZERO, DVec3::ZERO))),
    );
    assert!((resting[0] - alone[0]).length() > 1.0e-2 * centripetal.length());
}
