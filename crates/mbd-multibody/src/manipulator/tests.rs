use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};

use super::*;
use crate::aba::FloatingBaseState;

fn arm_spec(
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

fn hst_servicing_7dof() -> (ComponentTree, ManipulatorChain) {
    let base = (
        arm_spec(
            "servicer_bus",
            Placement::ROOT,
            600.0,
            DVec3::new(220.0, 260.0, 240.0),
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
    let mut joint_ids = Vec::new();
    for (i, axis) in axes.into_iter().enumerate() {
        let jid = ComponentId((i + 1) as u32);
        joint_ids.push(jid);
        let trans = if i == 0 {
            DVec3::new(1.2, 0.2, -0.3)
        } else {
            DVec3::new(0.75, 0.0, 0.0)
        };
        items.push((
            arm_spec(
                &format!("arm_{i}"),
                Placement {
                    parent: Some(ComponentId(i as u32)),
                    translation_m: trans,
                    rotation: DQuat::from_rotation_y(0.1 * (i as f64)),
                },
                15.0,
                DVec3::new(0.5, 2.0, 2.0),
                JointSpec::revolute(axis, -2.6, 2.6),
            ),
            ComponentState::at_temperature(293.15),
        ));
    }
    let tree = ComponentTree::new(items).unwrap();
    let chain =
        ManipulatorChain::new(&tree, joint_ids, ComponentId(7), DVec3::new(0.5, 0.0, 0.0)).unwrap();
    (tree, chain)
}

#[test]
fn generalized_jacobian_matches_exact_free_floating_momentum_recoil() {
    let (mut tree, chain) = hst_servicing_7dof();
    // Bend joint 2 to create an active 3D arm configuration.
    let mut st = *tree.state(ComponentId(2)).unwrap();
    st.joint.q_scalar = 0.6;
    tree.set_state(ComponentId(2), st).unwrap();

    let rates = [0.2, -0.3, 0.15, 0.4, -0.25, 0.35, -0.1];
    let j_star = chain.generalized_jacobian(&tree, &[]).unwrap();
    let mut twist_j_star = SpatialMotion::ZERO;
    for (col, &qd) in j_star.iter().zip(&rates) {
        twist_j_star += *col * qd;
    }

    // Solve true base velocity v_0 such that total system momentum == 0 when joints move at `rates`:
    for (&jid, &qd) in chain.joint_ids().iter().zip(&rates) {
        let mut js = *tree.state(jid).unwrap();
        js.joint.q_vel_scalar = qd;
        tree.set_state(jid, js).unwrap();
    }
    let h_base = MultiBodyDynamics::locked_root_spatial_inertia(&tree, &[]).unwrap();
    let p_uncompensated =
        MultiBodyDynamics::system_momentum(&tree, &[], &FloatingBaseState::AT_REST).unwrap();
    let mom_root = SpatialForce::new(
        p_uncompensated.angular_about_com_world_nms
            + p_uncompensated
                .com_world_m
                .cross(p_uncompensated.linear_world_kg_m_s),
        p_uncompensated.linear_world_kg_m_s,
    );
    let v0_comp = -h_base.solve(mom_root).unwrap();

    let poses = tree.evaluate_pose_table().unwrap();
    let ee_pose = poses.get(ComponentId(7)).unwrap();
    let p_ee =
        ee_pose.position_root_m + ee_pose.rotation_root_from_link * DVec3::new(0.5, 0.0, 0.0);
    let ee_lin_vel = v0_comp.lin
        + v0_comp.ang.cross(p_ee)
        + ee_pose.linear_vel_root_m_s
        + ee_pose
            .angular_vel_root_rad_s
            .cross(ee_pose.rotation_root_from_link * DVec3::new(0.5, 0.0, 0.0));
    let ee_ang_vel = v0_comp.ang + ee_pose.angular_vel_root_rad_s;

    assert!((twist_j_star.lin - ee_lin_vel).length() < 1e-11);
    assert!((twist_j_star.ang - ee_ang_vel).length() < 1e-11);
}

#[test]
fn dls_ik_tracks_target_and_null_space_pushes_away_from_joint_stop() {
    let (mut tree, chain) = hst_servicing_7dof();
    // Non-singular bent arm posture with shoulder roll (joint 3) near its +2.6 rad limit:
    for (id, q) in [(1, 0.3), (2, 0.5), (3, 2.4), (4, -0.9), (5, 0.4), (6, 0.5)] {
        let mut s = *tree.state(ComponentId(id)).unwrap();
        s.joint.q_scalar = q;
        tree.set_state(ComponentId(id), s).unwrap();
    }
    let (p_start, _) = chain.end_effector_pose(&tree).unwrap();

    // First verify null-space self-motion pushes joint 3 away from stop while keeping EE fixed:
    for _ in 0..220 {
        let null_v = chain.joint_limit_avoidance_null_vel(&tree, 1.5).unwrap();
        let q_dot = chain
            .solve_dls_ik(&tree, &[], SpatialMotion::ZERO, 1e-4, false, Some(&null_v))
            .unwrap();
        for (&jid, &qd) in chain.joint_ids().iter().zip(&q_dot) {
            let mut st = *tree.state(jid).unwrap();
            st.joint.q_scalar += qd * 0.01;
            tree.set_state(jid, st).unwrap();
        }
    }
    let (p_end, _) = chain.end_effector_pose(&tree).unwrap();
    let q3_final = tree.state(ComponentId(3)).unwrap().joint.q_scalar;
    assert!(
        (p_end - p_start).length() < 8e-3,
        "null space keeps EE fixed: {p_end:?} vs {p_start:?}"
    );
    assert!(q3_final < 2.1, "q3 swiveled away from stop: {q3_final}");

    // Also verify nonzero free-floating DLS-IK reproduces commanded end-effector twist:
    let cmd_twist = SpatialMotion::new(DVec3::new(0.02, -0.01, 0.03), DVec3::new(0.1, -0.05, 0.08));
    let q_dot = chain
        .solve_dls_ik(&tree, &[], cmd_twist, 1e-6, true, None)
        .unwrap();
    let jac = chain.generalized_jacobian(&tree, &[]).unwrap();
    let mut achieved = SpatialMotion::ZERO;
    for (col, &qd) in jac.iter().zip(&q_dot) {
        achieved += *col * qd;
    }
    assert!((achieved.lin - cmd_twist.lin).length() < 1e-7);
    assert!((achieved.ang - cmd_twist.ang).length() < 1e-7);
}

#[test]
fn capsule_clearance_and_admittance_controller_work_accurately() {
    let (tree, _) = hst_servicing_7dof();
    let poses = tree.evaluate_pose_table().unwrap();
    let capsules = [
        LinkCapsule {
            component: ComponentId(0),
            start_local_m: DVec3::new(-1.0, 0.0, 0.0),
            end_local_m: DVec3::new(1.0, 0.0, 0.0),
            radius_m: 0.6,
        },
        LinkCapsule {
            component: ComponentId(7),
            start_local_m: DVec3::ZERO,
            end_local_m: DVec3::new(0.5, 0.0, 0.0),
            radius_m: 0.1,
        },
    ];
    let clear = minimum_capsule_clearance(&poses, &capsules, &[]).unwrap();
    assert!(clear > 2.0);

    let overlap = [
        capsules[0],
        LinkCapsule {
            component: ComponentId(0),
            start_local_m: DVec3::new(0.0, 0.3, 0.0),
            end_local_m: DVec3::new(0.5, 0.3, 0.0),
            radius_m: 0.2,
        },
    ];
    assert!(minimum_capsule_clearance(&poses, &overlap, &[]).unwrap() < 0.0);

    let mut adm = AdmittanceController::new(20.0, 5.0, 80.0, 20.0, 40.0, 10.0).unwrap();
    for _ in 0..400 {
        let _ = adm.step(
            SpatialForce::new(DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0)),
            SpatialForce::ZERO,
            0.02,
        );
    }
    let (off_lin, _) = adm.offset();
    assert!(
        (off_lin.x - 0.25).abs() < 5e-3,
        "steady deflection = F/K = 10/40 = 0.25, got {off_lin:?}"
    );
}
