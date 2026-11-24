use std::f64::consts::FRAC_PI_2;

use glam::{DQuat, DVec3};

use super::*;
use crate::{DynamicMassContribution, JointSpec, JointState};

fn pose_vec_close(actual: DVec3, expected: DVec3) {
    assert!(
        (actual - expected).length() < 1e-10,
        "{actual} vs {expected}"
    );
}

fn pose_quat_close(actual: DQuat, expected: DQuat) {
    let dot = actual.dot(expected).abs();
    assert!((1.0 - dot) < 1e-10, "{actual} vs {expected}");
}

#[test]
fn pose_table_and_dynamic_mass_contributions_match_chain_poses() {
    let mut hinge = spec(
        "arm_link",
        Placement::child_of(CORE, DVec3::X * 2.0),
        10.0,
        isotropic(2.0),
    );
    hinge.joint = JointSpec::revolute(DVec3::Z, -3.0, 3.0);
    let tip = spec(
        "tip",
        Placement::child_of(ComponentId(1), DVec3::X),
        5.0,
        isotropic(1.0),
    );
    let tree = ComponentTree::new([
        (
            spec("core", Placement::ROOT, 100.0, isotropic(10.0)),
            state(),
        ),
        (
            hinge,
            ComponentState {
                joint: JointState::revolute(FRAC_PI_2, 0.5),
                ..state()
            },
        ),
        (tip, state()),
    ])
    .unwrap();

    let poses = tree.evaluate_pose_table().unwrap();
    let tip_pose = poses.get(ComponentId(2)).unwrap();
    let (chain_pos, chain_rot) = tree.world_transform_in_root(ComponentId(2)).unwrap();
    pose_vec_close(tip_pose.position_root_m, chain_pos);
    pose_quat_close(tip_pose.rotation_root_from_link, chain_rot);
    pose_vec_close(tip_pose.position_root_m, DVec3::new(2.0, 1.0, 0.0));
    pose_vec_close(tip_pose.linear_vel_root_m_s, DVec3::new(-0.5, 0.0, 0.0));
    pose_vec_close(tip_pose.angular_vel_root_rad_s, DVec3::Z * 0.5);

    let extra = DynamicMassContribution {
        component: CORE,
        mass_kg: 20.0,
        cg_local_m: DVec3::new(0.0, 0.0, -1.5),
        inertia_cg_local: isotropic(3.0),
    };
    let with_extra = tree.mass_properties_with_contributions(&[extra]).unwrap();
    let base = tree.mass_properties().unwrap();
    assert!((with_extra.total_mass_kg - (base.total_mass_kg + 20.0)).abs() < 1e-12);
    assert!(with_extra.cg_body_m.z < 0.0);
}

fn build_sivb_and_csm() -> (ComponentTree, ComponentTree) {
    let mut sla = spec(
        "sla",
        Placement::child_of(ComponentId(0), DVec3::X * 5.0),
        500.0,
        isotropic(50.0),
    );
    sla.placement.rotation = DQuat::from_rotation_y(0.2);
    let lm = spec(
        "lm",
        Placement {
            parent: Some(ComponentId(1)),
            translation_m: DVec3::X * 2.0,
            rotation: DQuat::from_rotation_x(0.4),
        },
        4_500.0,
        isotropic(500.0),
    );
    let sivb_stack = ComponentTree::new([
        (
            spec("s_ivb", Placement::ROOT, 10_000.0, isotropic(2_000.0)),
            state(),
        ),
        (sla, state()),
        (lm, state()),
    ])
    .unwrap();

    let csm = ComponentTree::new([
        (
            spec("csm_cm", Placement::ROOT, 5_500.0, isotropic(800.0)),
            state(),
        ),
        (
            spec(
                "docking_probe",
                Placement::child_of(ComponentId(0), DVec3::X * 3.0),
                100.0,
                isotropic(10.0),
            ),
            state(),
        ),
    ])
    .unwrap();
    (sivb_stack, csm)
}

#[test]
fn apollo_tde_graft_reroot_and_sivb_split_preserve_world_transforms() {
    let (sivb_stack, csm) = build_sivb_and_csm();
    let docked = sivb_stack
        .graft_subtree(
            &csm,
            ComponentId(2),
            Placement {
                parent: None,
                translation_m: DVec3::X * 4.0,
                rotation: DQuat::from_rotation_z(0.5),
            },
            JointSpec::FIXED,
            JointState::FIXED,
        )
        .unwrap();

    let csm_id = docked.grafted_id_map[0].1;
    let rerooted = docked.tree.reroot(csm_id).unwrap();
    let (p_new, r_new) = rerooted.new_root_pose_in_old_root;
    let inv_rot = r_new.conjugate();
    for id in docked.tree.ids() {
        let (p_old_i, r_old_i) = docked.tree.world_transform_in_root(id).unwrap();
        let (p_re_i, r_re_i) = rerooted.tree.world_transform_in_root(id).unwrap();
        pose_vec_close(p_re_i, inv_rot * (p_old_i - p_new));
        pose_quat_close(r_re_i, inv_rot * r_old_i);
    }

    let sla_id = docked.base_id_map[1].1;
    let extracted = rerooted.tree.split_subtree(sla_id).unwrap();
    assert_eq!(extracted.parent.len(), 3);
    assert_eq!(extracted.child.len(), 2);
    let lm_in_extracted = extracted
        .parent_id_map
        .iter()
        .find(|&&(old, _)| old == docked.base_id_map[2].1)
        .unwrap()
        .1;
    let (lm_pos, _) = extracted
        .parent
        .world_transform_in_root(lm_in_extracted)
        .unwrap();
    pose_vec_close(
        lm_pos,
        DQuat::from_rotation_z(-0.5) * DVec3::new(-4.0, 0.0, 0.0),
    );
}

#[test]
fn reroot_across_prismatic_and_zero_offset_revolute_preserves_kinematics() {
    let mut rail = spec(
        "rail",
        Placement {
            parent: Some(CORE),
            translation_m: DVec3::new(1.0, 0.5, 0.0),
            rotation: DQuat::from_rotation_y(0.3),
        },
        20.0,
        isotropic(2.0),
    );
    rail.joint = JointSpec::prismatic(DVec3::X, -2.0, 5.0);

    let mut hinge = spec(
        "hinge",
        Placement {
            parent: Some(ComponentId(1)),
            translation_m: DVec3::ZERO,
            rotation: DQuat::from_rotation_z(0.6),
        },
        30.0,
        isotropic(3.0),
    );
    hinge.joint = JointSpec::revolute(DVec3::Y, -std::f64::consts::PI, std::f64::consts::PI);

    let tree = ComponentTree::new([
        (
            spec("core", Placement::ROOT, 100.0, isotropic(10.0)),
            state(),
        ),
        (
            rail,
            ComponentState {
                joint: JointState::prismatic(1.25, 0.8),
                ..state()
            },
        ),
        (
            hinge,
            ComponentState {
                joint: JointState::revolute(0.7, -0.4),
                ..state()
            },
        ),
    ])
    .unwrap();

    let rerooted = tree.reroot(ComponentId(2)).unwrap();
    let old_poses = tree.evaluate_pose_table().unwrap();
    let new_poses = rerooted.tree.evaluate_pose_table().unwrap();
    let (p_h, r_h) = rerooted.new_root_pose_in_old_root;
    let h_old = old_poses.get(ComponentId(2)).unwrap();
    let inv_r = r_h.conjugate();
    for id in tree.ids() {
        let p_o = old_poses.get(id).unwrap();
        let p_n = new_poses.get(id).unwrap();
        pose_vec_close(p_n.position_root_m, inv_r * (p_o.position_root_m - p_h));
        pose_quat_close(
            p_n.rotation_root_from_link,
            inv_r * p_o.rotation_root_from_link,
        );
        let rel_w = inv_r * (p_o.angular_vel_root_rad_s - h_old.angular_vel_root_rad_s);
        let rel_v = inv_r
            * (p_o.linear_vel_root_m_s
                - h_old.linear_vel_root_m_s
                - h_old
                    .angular_vel_root_rad_s
                    .cross(p_o.position_root_m - p_h));
        pose_vec_close(p_n.angular_vel_root_rad_s, rel_w);
        pose_vec_close(p_n.linear_vel_root_m_s, rel_v);
    }
}
