//! Classical RK4 trajectories pinned to recorded bit patterns
//! (`rk4_reference_bits.txt`): every joint kind, a driven floating base,
//! actuator limits, travel stops, joint loads, external wrenches and
//! distributed masses. The integrator and the derivative it drives must keep
//! producing these states; all arithmetic runs through `libm`, so the bits do
//! not depend on the platform.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, JointActuatorSpec, JointSpec, JointState,
    Placement, WrenchAtSite,
};

use super::rigs::{
    OPEN_CONE_RAD, WIDE_RAD, diag, ground, hinge_z, idle, part, point_mass, posed, run_on_ground,
};
use super::stepping::{StageForces, step_rk4, step_rk4_with_forces, step_rk4_with_loads};
use crate::{FloatingBaseState, JointDriveInput, JointLoadInput, MultibodyError};

const RECORDED: &str = include_str!("rk4_reference_bits.txt");

/// Base state followed by every component's joint state, in id order.
fn flatten(tree: &ComponentTree, base: &FloatingBaseState) -> Vec<f64> {
    let mut out = Vec::new();
    out.extend(base.position_world_m.to_array());
    out.extend(base.rotation_world_from_root.to_array());
    out.extend(base.linear_vel_root_m_s.to_array());
    out.extend(base.angular_vel_root_rad_s.to_array());
    for st in tree.states() {
        let j = st.joint;
        out.extend([j.q_scalar, j.q_vel_scalar]);
        out.extend(j.translation_m.to_array());
        out.extend(j.linear_vel_m_s.to_array());
        out.extend(j.rotation.to_array());
        out.extend(j.angular_vel_rad_s.to_array());
    }
    out
}

fn pendulum() -> Vec<f64> {
    let mut tree = ComponentTree::new([
        ground(),
        hinge_z(0, DVec3::ZERO, 1.0, 0.0),
        point_mass(1, DVec3::new(0.0, -1.0, 0.0), 1.0),
    ])
    .unwrap();
    let g = DVec3::new(0.0, -9.81, 0.0);
    let base = run_on_ground(&mut tree, FloatingBaseState::AT_REST, g, &[], 0.01, 300);
    flatten(&tree, &base)
}

fn double_pendulum() -> Vec<f64> {
    let mut tree = ComponentTree::new([
        ground(),
        hinge_z(0, DVec3::ZERO, 2.0, 0.0),
        point_mass(1, DVec3::new(0.0, -1.1, 0.0), 1.5),
        hinge_z(2, DVec3::ZERO, 1.0, 0.0),
        point_mass(3, DVec3::new(0.0, -0.7, 0.0), 0.9),
    ])
    .unwrap();
    let g = DVec3::new(0.0, -9.81, 0.0);
    let base = run_on_ground(&mut tree, FloatingBaseState::AT_REST, g, &[], 2.0e-3, 1500);
    flatten(&tree, &base)
}

fn solo_body(inertia: DMat3) -> ComponentTree {
    ComponentTree::new([(
        part("body", Placement::ROOT, 1.0, inertia, JointSpec::FIXED),
        idle(),
    )])
    .unwrap()
}

fn kepler() -> Vec<f64> {
    let mut tree = solo_body(diag(1.0, 1.0, 1.0));
    let mut base = FloatingBaseState {
        position_world_m: DVec3::X,
        linear_vel_root_m_s: DVec3::Y,
        ..FloatingBaseState::AT_REST
    };
    for _ in 0..40 {
        base =
            step_rk4_with_forces::<MultibodyError, _>(&mut tree, &[], &base, 0.05, |_, st, _| {
                let r = st.position_world_m;
                Ok((
                    -r / (r.length_squared() * r.length()),
                    Vec::new(),
                    Vec::new(),
                ))
            })
            .unwrap();
    }
    flatten(&tree, &base)
}

fn tumbling_body() -> Vec<f64> {
    let mut tree = solo_body(diag(1.0, 2.0, 3.0));
    let mut base = FloatingBaseState {
        angular_vel_root_rad_s: DVec3::new(0.8, 0.3, -0.6),
        ..FloatingBaseState::AT_REST
    };
    for _ in 0..40 {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &[], &[], 0.05).unwrap();
    }
    flatten(&tree, &base)
}

/// Ball joint that starts inside a narrow cone and runs into its stop.
fn ball_joint_into_cone_stop() -> Vec<f64> {
    let mut tree = ComponentTree::new([
        ground(),
        (
            part(
                "ball",
                Placement::child_of(ComponentId(0), DVec3::new(0.2, 0.0, 0.1)),
                3.0,
                diag(1.0, 2.0, 3.0),
                JointSpec::spherical(0.3),
            ),
            posed(JointState::spherical(
                DQuat::from_rotation_x(0.1),
                DVec3::new(0.8, 0.3, -0.6),
            )),
        ),
    ])
    .unwrap();
    let base = FloatingBaseState::AT_REST;
    let g = DVec3::new(0.0, 0.0, -1.0);
    let base = run_on_ground(&mut tree, base, g, &[], 2.0e-3, 400);
    flatten(&tree, &base)
}

/// Free-floating body driving a hinged panel, with a ball-jointed tip.
fn driven_floating_arm() -> Vec<f64> {
    let mut tree = ComponentTree::new([
        (
            part(
                "base",
                Placement::ROOT,
                6.0,
                diag(2.0, 2.5, 3.0),
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "arm",
                Placement {
                    parent: Some(ComponentId(0)),
                    translation_m: DVec3::new(1.0, 0.2, -0.1),
                    rotation: DQuat::from_rotation_z(0.3),
                },
                2.0,
                diag(0.2, 0.3, 0.4),
                JointSpec::revolute(DVec3::Y, -WIDE_RAD, WIDE_RAD),
            ),
            posed(JointState::revolute(0.3, 0.6)),
        ),
        (
            part(
                "tip",
                Placement::child_of(ComponentId(1), DVec3::new(0.7, 0.2, 0.1)),
                1.0,
                diag(0.01, 0.02, 0.03),
                JointSpec::spherical(OPEN_CONE_RAD),
            ),
            posed(JointState::spherical(
                DQuat::from_euler(glam::EulerRot::XYZ, 0.3, 0.2, -0.4),
                DVec3::new(0.4, -0.3, 0.5),
            )),
        ),
    ])
    .unwrap();
    let mut base = FloatingBaseState {
        position_world_m: DVec3::new(3.0, -1.0, 2.0),
        rotation_world_from_root: DQuat::from_rotation_x(0.3),
        linear_vel_root_m_s: DVec3::new(0.5, -0.2, 0.1),
        angular_vel_root_rad_s: DVec3::new(0.1, -0.2, 0.15),
    };
    let drives = [
        JointDriveInput::scalar(ComponentId(1), 0.4),
        JointDriveInput {
            component: ComponentId(2),
            effort_scalar: 0.0,
            torque_3d_nm: DVec3::new(0.02, -0.01, 0.03),
            force_3d_n: DVec3::ZERO,
        },
    ];
    let g = DVec3::new(0.3, 0.0, -1.2);
    for _ in 0..300 {
        base = step_rk4(&mut tree, &[], &base, g, &drives, &[], 2.0e-3).unwrap();
    }
    flatten(&tree, &base)
}

fn compliant_dock() -> Vec<f64> {
    let mut tree = ComponentTree::new([
        (
            part(
                "chaser",
                Placement::ROOT,
                300.0,
                diag(50.0, 50.0, 50.0),
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "target",
                Placement::child_of(ComponentId(0), DVec3::X * 2.0),
                600.0,
                diag(80.0, 90.0, 70.0),
                JointSpec::compliant_6dof(
                    DVec3::splat(200.0),
                    DVec3::splat(40.0),
                    DVec3::splat(100.0),
                    DVec3::splat(20.0),
                ),
            ),
            posed(JointState::compliant_6dof(
                DVec3::new(0.2, -0.1, 0.05),
                DVec3::new(0.0, 0.02, 0.0),
                DQuat::from_rotation_x(0.1),
                DVec3::new(0.01, 0.0, -0.02),
            )),
        ),
    ])
    .unwrap();
    let mut base = FloatingBaseState {
        angular_vel_root_rad_s: DVec3::new(0.0, 0.02, 0.01),
        ..FloatingBaseState::AT_REST
    };
    for _ in 0..250 {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &[], &[], 0.004).unwrap();
    }
    flatten(&tree, &base)
}

/// Sprung, effort-limited hinge driven past its travel stop.
fn limited_hinge_into_stop() -> Vec<f64> {
    let actuator = JointActuatorSpec {
        max_effort: 2.0,
        max_velocity: 10.0,
        damping: 0.3,
        stiffness: 1.5,
        coulomb_friction: 0.0,
        backdrivable: true,
    };
    let hinge = JointSpec::revolute(DVec3::Z, -0.2, 0.2).with_actuator(actuator);
    let mut tree = ComponentTree::new([
        ground(),
        (
            part(
                "hinge",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                0.0,
                DMat3::ZERO,
                hinge,
            ),
            posed(JointState::revolute(0.1, 1.0)),
        ),
        point_mass(1, DVec3::X, 2.0),
    ])
    .unwrap();
    let drives = [
        JointDriveInput::scalar(ComponentId(1), 1.5),
        JointDriveInput::scalar(ComponentId(1), 4.0),
    ];
    let base = run_on_ground(
        &mut tree,
        FloatingBaseState::AT_REST,
        DVec3::ZERO,
        &drives,
        1.0e-3,
        400,
    );
    flatten(&tree, &base)
}

/// Prismatic seat on a capsule: drive, friction load, an off-axis external
/// wrench on the seat and a distributed mass riding on it.
fn loaded_rail_with_distributed_mass() -> Vec<f64> {
    let mut tree = ComponentTree::new([
        (
            part(
                "capsule",
                Placement::ROOT,
                400.0,
                diag(100.0, 110.0, 120.0),
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "seat",
                Placement::child_of(ComponentId(0), DVec3::new(0.0, 0.3, 0.0)),
                100.0,
                diag(10.0, 11.0, 12.0),
                JointSpec::prismatic(DVec3::X, -1.0, 5.0),
            ),
            idle(),
        ),
    ])
    .unwrap();
    let masses = [DynamicMassContribution {
        component: ComponentId(1),
        mass_kg: 30.0,
        cg_local_m: DVec3::new(0.3, 0.2, -0.4),
        inertia_cg_local: diag(2.0, 3.0, 4.0),
    }];
    let forces = StageForces {
        gravity_world_m_s2: DVec3::new(0.0, 0.0, -1.62),
        joint_drives: vec![JointDriveInput::scalar(ComponentId(1), 1_000.0)],
        joint_loads: vec![
            JointLoadInput::scalar(ComponentId(1), -200.0),
            JointLoadInput::scalar(ComponentId(1), 15.0),
        ],
        external_wrenches: vec![WrenchAtSite {
            site: ComponentId(1),
            point_local_m: DVec3::new(0.5, 0.0, 0.2),
            force_local_n: DVec3::new(12.0, -8.0, 5.0),
            torque_local_nm: DVec3::new(1.0, 2.0, -1.5),
        }],
    };
    let mut base = FloatingBaseState::AT_REST;
    for _ in 0..50 {
        base =
            step_rk4_with_loads::<MultibodyError, _>(&mut tree, &masses, &base, 0.01, |_, _, _| {
                Ok(forces.clone())
            })
            .unwrap();
    }
    flatten(&tree, &base)
}

type Scenario = (&'static str, fn() -> Vec<f64>);

const SCENARIOS: [Scenario; 9] = [
    ("pendulum", pendulum),
    ("double_pendulum", double_pendulum),
    ("kepler", kepler),
    ("tumbling_body", tumbling_body),
    ("ball_joint_into_cone_stop", ball_joint_into_cone_stop),
    ("driven_floating_arm", driven_floating_arm),
    ("compliant_dock", compliant_dock),
    ("limited_hinge_into_stop", limited_hinge_into_stop),
    (
        "loaded_rail_with_distributed_mass",
        loaded_rail_with_distributed_mass,
    ),
];

fn recorded(name: &str) -> Vec<f64> {
    let line = RECORDED
        .lines()
        .find_map(|l| l.strip_prefix(name)?.strip_prefix(':'))
        .unwrap_or_else(|| panic!("no recorded trajectory named {name}"));
    line.split_whitespace()
        .map(|hex| f64::from_bits(u64::from_str_radix(hex, 16).unwrap()))
        .collect()
}

#[test]
fn rk4_trajectories_reproduce_recorded_bits() {
    for (name, run) in SCENARIOS {
        let (now, then) = (run(), recorded(name));
        assert_eq!(now.len(), then.len(), "{name}: state length changed");
        for (k, (a, b)) in now.iter().zip(&then).enumerate() {
            assert_eq!(
                a.to_bits(),
                b.to_bits(),
                "{name}[{k}]: {a:e} differs from recorded {b:e} by {:e}",
                a - b
            );
        }
    }
}
