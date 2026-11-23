use alloc::vec;
use alloc::vec::Vec;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentTree, DynamicMassContribution, JointKind, WrenchAtSite};

use super::articulated::{assign_slot, solve_aba};
use super::chain::{JointEfforts, LinkData, build_link_chain, validate_inputs};
use super::{FloatingBaseState, JointDriveInput, JointLoadInput, MultiBodyAccel, SystemMomentum};
use crate::MultibodyError;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion};

/// Floating-base Featherstone Articulated-Body Algorithm (ABA) with automatic
/// zero-DoF [`RigidFastPath`](MultiBodyAccel::used_rigid_fast_path) dispatch.
pub struct MultiBodyDynamics;

impl MultiBodyDynamics {
    /// Solves exact forward dynamics $\ddot{\mathbf{q}}$ and floating-base
    /// acceleration for `tree` at `base_state`, with actuator commands only.
    pub fn forward_dynamics(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
        gravity_world_m_s2: DVec3,
        joint_drives: &[JointDriveInput],
        external_wrenches: &[WrenchAtSite],
    ) -> Result<MultiBodyAccel, MultibodyError> {
        Self::forward_dynamics_with_loads(
            tree,
            dynamic_masses,
            base_state,
            gravity_world_m_s2,
            joint_drives,
            &[],
            external_wrenches,
        )
    }

    /// [`Self::forward_dynamics`] with additional unlimited joint loads
    /// (constraints, springs, mechanisms) superposed on the actuator commands.
    pub fn forward_dynamics_with_loads(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
        gravity_world_m_s2: DVec3,
        joint_drives: &[JointDriveInput],
        joint_loads: &[JointLoadInput],
        external_wrenches: &[WrenchAtSite],
    ) -> Result<MultiBodyAccel, MultibodyError> {
        validate_inputs(base_state, gravity_world_m_s2, dynamic_masses)?;
        let efforts = JointEfforts {
            drives: joint_drives,
            loads: joint_loads,
        };
        let links = build_link_chain(tree, dynamic_masses, efforts, external_wrenches)?;
        if links.iter().all(|l| matches!(l.kind, JointKind::Fixed)) {
            return solve_rigid_fast_path(&links, base_state, gravity_world_m_s2);
        }
        solve_aba(&links, base_state, gravity_world_m_s2)
    }

    /// Computes total world mass, CoM, linear momentum, and angular momentum
    /// about the instantaneous system CoM across all attached links.
    pub fn system_momentum(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
    ) -> Result<SystemMomentum, MultibodyError> {
        validate_inputs(base_state, DVec3::ZERO, dynamic_masses)?;
        let links = build_link_chain(tree, dynamic_masses, JointEfforts::NONE, &[])?;
        let n = links.len();
        let mut r_world_list = vec![DMat3::IDENTITY; n];
        let mut p_world_list = vec![DVec3::ZERO; n];
        let mut v_local_list = vec![SpatialMotion::ZERO; n];

        let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
        let p0 = base_state.position_world_m;
        let v0 = base_state.spatial_velocity_root();

        for idx in 0..n {
            let Some(link) = links.get(idx) else {
                break;
            };
            if let Some(p_idx) = link.parent_order {
                let (Some(&rp), Some(&pp), Some(&vp)) = (
                    r_world_list.get(p_idx),
                    p_world_list.get(p_idx),
                    v_local_list.get(p_idx),
                ) else {
                    break;
                };
                let r_i = rp * link.x_parent_from_child.r_parent_from_child;
                let p_i = pp + rp * link.x_parent_from_child.p_child_in_parent;
                let v_i = link.x_parent_from_child.apply_motion_to_child(vp) + link.v_j_child;
                assign_slot(&mut r_world_list, idx, r_i);
                assign_slot(&mut p_world_list, idx, p_i);
                assign_slot(&mut v_local_list, idx, v_i);
            } else {
                assign_slot(&mut r_world_list, idx, r0);
                assign_slot(&mut p_world_list, idx, p0);
                assign_slot(&mut v_local_list, idx, v0);
            }
        }
        accumulate_world_momentum(&links, &r_world_list, &p_world_list, &v_local_list)
    }

    /// Computes total rigid-body spatial inertia in root axes assuming
    /// joints are momentarily frozen, used by mechanisms and Generalized Jacobians.
    pub fn locked_root_spatial_inertia(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
    ) -> Result<SpatialMat6, MultibodyError> {
        let links = build_link_chain(tree, dynamic_masses, JointEfforts::NONE, &[])?;
        let mut total = vec![SpatialMat6::ZERO; links.len()];
        for idx in 0..links.len() {
            if let (Some(slot), Some(link)) = (total.get_mut(idx), links.get(idx)) {
                *slot = link.i_local;
            }
        }
        for idx in (1..links.len()).rev() {
            let (Some(child), Some(&child_i)) = (links.get(idx), total.get(idx)) else {
                continue;
            };
            let Some(p_idx) = child.parent_order else {
                continue;
            };
            let shifted = child_i.transform_to_parent(&child.x_parent_from_child);
            if let Some(p_slot) = total.get_mut(p_idx) {
                *p_slot += shifted;
            }
        }
        total
            .first()
            .copied()
            .ok_or(MultibodyError::SingularMassMatrix)
    }
}

fn accumulate_world_momentum(
    links: &[LinkData],
    r_world: &[DMat3],
    p_world: &[DVec3],
    v_local: &[SpatialMotion],
) -> Result<SystemMomentum, MultibodyError> {
    let mut total_mass = 0.0;
    let mut first_moment = DVec3::ZERO;
    for idx in 0..links.len() {
        let (Some(link), Some(&r), Some(&p)) = (links.get(idx), r_world.get(idx), p_world.get(idx))
        else {
            continue;
        };
        let cm_world = p + r * link.cg_local_m;
        total_mass += link.mass_kg;
        first_moment += cm_world * link.mass_kg;
    }
    if total_mass <= 0.0 {
        return Err(MultibodyError::SingularMassMatrix);
    }
    let com_world = first_moment / total_mass;
    let mut lin_mom = DVec3::ZERO;
    let mut ang_mom = DVec3::ZERO;
    for idx in 0..links.len() {
        let (Some(link), Some(&r), Some(&p), Some(&v)) = (
            links.get(idx),
            r_world.get(idx),
            p_world.get(idx),
            v_local.get(idx),
        ) else {
            continue;
        };
        let cm_world = p + r * link.cg_local_m;
        let w_world = r * v.ang;
        let v_cm_world = r * (v.lin + v.ang.cross(link.cg_local_m));
        let p_i = v_cm_world * link.mass_kg;
        lin_mom += p_i;
        let i_cm_world = r * link.inertia_cg_local * r.transpose();
        ang_mom += i_cm_world * w_world + (cm_world - com_world).cross(p_i);
    }
    Ok(SystemMomentum {
        total_mass_kg: total_mass,
        com_world_m: com_world,
        linear_world_kg_m_s: lin_mom,
        angular_about_com_world_nms: ang_mom,
    })
}

fn solve_rigid_fast_path(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
) -> Result<MultiBodyAccel, MultibodyError> {
    let n = links.len();
    let mut inertias = vec![SpatialMat6::ZERO; n];
    let mut wrenches = vec![SpatialForce::ZERO; n];
    let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
    let g_root = r0.transpose() * gravity_world;

    for idx in 0..n {
        if let Some(link) = links.get(idx) {
            assign_slot(&mut inertias, idx, link.i_local);
            assign_slot(&mut wrenches, idx, link.ext_force_local);
        }
    }
    for idx in (1..n).rev() {
        let (Some(link), Some(&i_c), Some(&f_c)) =
            (links.get(idx), inertias.get(idx), wrenches.get(idx))
        else {
            continue;
        };
        let Some(p_idx) = link.parent_order else {
            continue;
        };
        let i_shift = i_c.transform_to_parent(&link.x_parent_from_child);
        let f_shift = link.x_parent_from_child.apply_force_to_parent(f_c);
        if let Some(i_p) = inertias.get_mut(p_idx) {
            *i_p += i_shift;
        }
        if let Some(f_p) = wrenches.get_mut(p_idx) {
            *f_p += f_shift;
        }
    }

    let i_total = inertias
        .first()
        .copied()
        .ok_or(MultibodyError::SingularMassMatrix)?;
    let f_ext = wrenches.first().copied().unwrap_or(SpatialForce::ZERO);
    let v0 = base_state.spatial_velocity_root();
    let f_grav = i_total.mul_motion(SpatialMotion::new(DVec3::ZERO, g_root));
    let bias = v0.cross_force(i_total.mul_motion(v0));
    let accel = i_total
        .solve(f_ext + f_grav - bias)
        .ok_or(MultibodyError::SingularMassMatrix)?;

    Ok(MultiBodyAccel {
        base_spatial_accel_root: accel,
        joint_accels: Vec::new(),
        used_rigid_fast_path: true,
    })
}
