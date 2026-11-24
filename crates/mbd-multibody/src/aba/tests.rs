use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointSpec,
    JointState, Placement, WrenchAtSite,
};

use self::stepping::{step_rk4, step_rk4_with_forces};
use super::*;

mod compliant_joint;
mod convergence;
mod double_pendulum;
mod effort_sources;
mod free_floating;
mod joint_drives;
mod joint_stops;
mod pendulum;
mod rigid_fast_path;
mod rigs;
mod rk4_reference;
mod scale_invariance;
mod small_links;
mod spherical_joint;
mod spherical_pendulum;
pub(crate) mod stepping;
mod stop_response;

fn part_spec(
    name: &str,
    placement: Placement,
    mass_kg: f64,
    diag: DVec3,
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg: mass_kg,
        dry_inertia_body: DMat3::from_diagonal(diag),
        thermal_capacity_j_k: 100.0,
        nominal_power_w: 0.0,
        joint,
    }
}

fn free_floating_7dof_tree() -> ComponentTree {
    let base = (
        part_spec(
            "spacecraft",
            Placement::ROOT,
            500.0,
            DVec3::new(180.0, 220.0, 200.0),
            JointSpec::FIXED,
        ),
        ComponentState::at_temperature(293.15),
    );
    let axes = [
        DVec3::Z,
        DVec3::Y,
        DVec3::X,
        DVec3::Y,
        DVec3::X,
        DVec3::Y,
        DVec3::Z,
    ];
    let mut items = vec![base];
    for (idx, axis) in axes.into_iter().enumerate() {
        let offset = if idx == 0 {
            DVec3::new(1.5, 0.3, -0.4)
        } else {
            DVec3::new(0.8, 0.0, 0.0)
        };
        items.push((
            part_spec(
                &format!("link_{idx}"),
                Placement {
                    parent: Some(ComponentId(idx as u32)),
                    translation_m: offset,
                    rotation: DQuat::from_axis_angle(DVec3::Y, 0.05 * (idx as f64)),
                },
                12.0,
                DVec3::new(0.4, 1.5, 1.5),
                JointSpec::revolute(axis, -std::f64::consts::PI, std::f64::consts::PI),
            ),
            ComponentState::at_temperature(293.15),
        ));
    }
    ComponentTree::new(items).unwrap()
}

fn rigid_fixture_tree() -> ComponentTree {
    ComponentTree::new([
        (
            part_spec(
                "core",
                Placement::ROOT,
                200.0,
                DVec3::new(40.0, 60.0, 70.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            part_spec(
                "pod",
                Placement {
                    parent: Some(ComponentId(0)),
                    translation_m: DVec3::new(2.0, -1.0, 0.5),
                    rotation: DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.4),
                },
                50.0,
                DVec3::new(8.0, 12.0, 15.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap()
}

#[test]
fn rigid_fast_path_matches_unlocked_aba_to_machine_precision() {
    let rigid_tree = rigid_fixture_tree();

    // Equivalent tree with zero-motion locked-like compliant/fixed check via 1e9 stiffness?
    // Even cleaner: verify RigidFastPath directly against single-lump MassProperties Newton-Euler!
    let extra = DynamicMassContribution {
        component: ComponentId(1),
        mass_kg: 30.0,
        cg_local_m: DVec3::new(0.3, 0.2, -0.4),
        inertia_cg_local: DMat3::from_diagonal(DVec3::new(2.0, 3.0, 4.0)),
    };
    let mp = rigid_tree
        .mass_properties_with_contributions(&[extra])
        .unwrap();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(100.0, 0.0, 0.0),
        rotation_world_from_root: DQuat::from_rotation_y(0.4),
        linear_vel_world_m_s: DQuat::from_rotation_y(0.4) * DVec3::new(20.0, -5.0, 3.0),
        angular_vel_root_rad_s: DVec3::new(0.2, -0.4, 0.3),
    };
    let ext = WrenchAtSite {
        site: ComponentId(1),
        point_local_m: DVec3::new(0.5, 0.0, 0.0),
        force_local_n: DVec3::new(120.0, -80.0, 50.0),
        torque_local_nm: DVec3::new(10.0, 20.0, -15.0),
    };
    let accel = MultiBodyDynamics::forward_dynamics(
        &rigid_tree,
        &[extra],
        &base,
        DVec3::new(0.0, 0.0, -9.81),
        &[],
        &[ext],
    )
    .unwrap();
    assert!(accel.used_rigid_fast_path);

    let pose1 = rigid_tree
        .evaluate_pose_table()
        .unwrap()
        .get(ComponentId(1))
        .copied()
        .unwrap();
    let (f_root, m_cg) = ext.to_root_wrench(&pose1, mp.cg_body_m);
    let w = base.angular_vel_root_rad_s;
    let expected_alpha = mp.inertia_cg_body.inverse() * (m_cg - w.cross(mp.inertia_cg_body * w));
    let g_root = base.rotation_world_from_root.conjugate() * DVec3::new(0.0, 0.0, -9.81);
    // Inertial acceleration of the centre of mass, then of the root origin
    // a rigid arm away from it, in root axes.
    let a_cg = f_root / mp.total_mass_kg + g_root;
    let expected_a0_lin =
        a_cg - expected_alpha.cross(mp.cg_body_m) - w.cross(w.cross(mp.cg_body_m));
    assert!((accel.base_angular_accel_root_rad_s2 - expected_alpha).length() < 1e-12);
    let a0_lin = base.rotation_world_from_root.conjugate() * accel.base_linear_accel_world_m_s2;
    assert!(
        (a0_lin - expected_a0_lin).length() < 1e-12,
        "lin: {a0_lin:?} vs {expected_a0_lin:?}"
    );
}

#[test]
fn free_floating_7dof_manipulator_conserves_momentum_under_motor_torques() {
    let mut tree = free_floating_7dof_tree();
    let mut base = FloatingBaseState {
        position_world_m: DVec3::new(10.0, -5.0, 2.0),
        rotation_world_from_root: DQuat::from_rotation_x(0.3),
        linear_vel_world_m_s: DQuat::from_rotation_x(0.3) * DVec3::new(1.5, -0.5, 0.2),
        angular_vel_root_rad_s: DVec3::new(0.02, -0.01, 0.03),
    };
    let initial = MultiBodyDynamics::system_momentum(&tree, &[], &base).unwrap();
    let drives: Vec<_> = (1..=7)
        .map(|i| JointDriveInput::scalar(ComponentId(i), if i % 2 == 0 { 1.2 } else { -0.9 }))
        .collect();

    for _ in 0..200 {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &drives, &[], 0.005).unwrap();
    }
    let after = MultiBodyDynamics::system_momentum(&tree, &[], &base).unwrap();
    let lin_err = (after.linear_world_kg_m_s - initial.linear_world_kg_m_s).length();
    let ang_err =
        (after.angular_about_com_world_nms - initial.angular_about_com_world_nms).length();
    assert!(
        lin_err < 1e-8 * initial.linear_world_kg_m_s.length().max(1.0),
        "7dof lin_err = {lin_err}"
    );
    assert!(
        ang_err < 1e-8 * initial.angular_about_com_world_nms.length().max(1.0),
        "7dof ang_err = {ang_err}"
    );
    // Base recoils noticeably opposite to the driving arm motion!
    assert!((base.angular_vel_root_rad_s - DVec3::new(0.02, -0.01, 0.03)).length() > 1e-3);
}

#[test]
fn prismatic_rail_acceleration_matches_exact_reduced_mass() {
    let mut tree = ComponentTree::new([
        (
            part_spec(
                "capsule",
                Placement::ROOT,
                400.0,
                DVec3::splat(100.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            part_spec(
                "seat",
                Placement::child_of(ComponentId(0), DVec3::ZERO),
                100.0,
                DVec3::splat(10.0),
                JointSpec::prismatic(DVec3::X, -1.0, 5.0),
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();

    let mut base = FloatingBaseState::AT_REST;
    let init_mom = MultiBodyDynamics::system_momentum(&tree, &[], &base).unwrap();
    let drive = [JointDriveInput::scalar(ComponentId(1), 1_000.0)];
    let accel =
        MultiBodyDynamics::forward_dynamics(&tree, &[], &base, DVec3::ZERO, &drive, &[]).unwrap();
    assert!((accel.joint_accels[0].q_accel_scalar - 12.5).abs() < 1e-12);
    assert!((accel.base_linear_accel_world_m_s2.x - (-2.5)).abs() < 1e-12);

    for _ in 0..50 {
        base = step_rk4(&mut tree, &[], &base, DVec3::ZERO, &drive, &[], 0.01).unwrap();
    }
    let after = MultiBodyDynamics::system_momentum(&tree, &[], &base).unwrap();
    assert!((after.linear_world_kg_m_s - init_mom.linear_world_kg_m_s).length() < 1e-11);
}

#[test]
fn compliant_6dof_dock_settles_and_conserves_momentum() {
    let mut ctree = ComponentTree::new([
        (
            part_spec(
                "chaser",
                Placement::ROOT,
                300.0,
                DVec3::splat(50.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            part_spec(
                "target",
                Placement::child_of(ComponentId(0), DVec3::X * 2.0),
                600.0,
                DVec3::splat(80.0),
                JointSpec::compliant_6dof(
                    DVec3::splat(200.0),
                    DVec3::splat(40.0),
                    DVec3::splat(100.0),
                    DVec3::splat(20.0),
                ),
            ),
            ComponentState {
                joint: JointState::compliant_6dof(
                    DVec3::new(0.2, -0.1, 0.05),
                    DVec3::ZERO,
                    DQuat::from_rotation_x(0.1),
                    DVec3::ZERO,
                ),
                ..ComponentState::at_temperature(293.15)
            },
        ),
    ])
    .unwrap();
    let mut cbase = FloatingBaseState::AT_REST;
    let cinit = MultiBodyDynamics::system_momentum(&ctree, &[], &cbase).unwrap();
    for _ in 0..250 {
        cbase = step_rk4(&mut ctree, &[], &cbase, DVec3::ZERO, &[], &[], 0.004).unwrap();
    }
    let cafter = MultiBodyDynamics::system_momentum(&ctree, &[], &cbase).unwrap();
    assert!(
        (cafter.linear_world_kg_m_s - cinit.linear_world_kg_m_s).length() < 1e-7,
        "c_lin: {:?} vs {:?}",
        cafter.linear_world_kg_m_s,
        cinit.linear_world_kg_m_s
    );
    assert!(
        (cafter.angular_about_com_world_nms - cinit.angular_about_com_world_nms).length() < 1e-6,
        "c_ang: {:?} vs {:?}",
        cafter.angular_about_com_world_nms,
        cinit.angular_about_com_world_nms
    );
    assert!(
        ctree
            .state(ComponentId(1))
            .unwrap()
            .joint
            .translation_m
            .length()
            < 0.2
    );
}

#[test]
fn step_rk4_with_forces_achieves_fourth_order_central_orbit_accuracy() {
    let mu = 3.986_004_418e14_f64;
    let r0 = 6_778_137.0_f64;
    let v_circ = (mu / r0).sqrt();
    let mut tree = ComponentTree::new([(
        part_spec(
            "sat",
            Placement::ROOT,
            500.0,
            DVec3::splat(200.0),
            JointSpec::FIXED,
        ),
        ComponentState::at_temperature(293.15),
    )])
    .unwrap();
    let mut base = FloatingBaseState {
        position_world_m: DVec3::new(r0, 0.0, 0.0),
        rotation_world_from_root: DQuat::IDENTITY,
        linear_vel_world_m_s: DVec3::new(0.0, v_circ, 0.0),
        angular_vel_root_rad_s: DVec3::ZERO,
    };
    for _ in 0..100 {
        base = step_rk4_with_forces::<crate::MultibodyError, _>(
            &mut tree,
            &[],
            &base,
            0.1,
            |_, st, _| {
                let r = st.position_world_m;
                let r3 = r.length_squared() * r.length();
                Ok((-mu * r / r3, Vec::new(), Vec::new()))
            },
        )
        .unwrap();
    }
    let rad_err = (base.position_world_m.length() - r0).abs();
    assert!(rad_err < 1e-4, "4th-order LEO radius error = {rad_err} m");
}
