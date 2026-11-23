use alloc::vec;
use alloc::vec::Vec;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointKind,
    MbdError, WrenchAtSite,
};

use super::{FloatingBaseState, JointDriveInput};
use crate::MultibodyError;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion, SpatialTransform, outer3};

const JOINT_LIMIT_STIFFNESS: f64 = 2.0e3;
const JOINT_LIMIT_DAMPING: f64 = 80.0;

pub(super) struct LinkData {
    pub(super) id: ComponentId,
    pub(super) parent_order: Option<usize>,
    pub(super) x_parent_from_child: SpatialTransform,
    pub(super) i_local: SpatialMat6,
    pub(super) v_j_child: SpatialMotion,
    pub(super) s_1dof: SpatialMotion,
    pub(super) q_j_rot: DQuat,
    pub(super) tau_1dof: f64,
    pub(super) tau_3d_child: DVec3,
    pub(super) force_3d_child: DVec3,
    pub(super) ext_force_local: SpatialForce,
    pub(super) mass_kg: f64,
    pub(super) cg_local_m: DVec3,
    pub(super) inertia_cg_local: DMat3,
    pub(super) kind: JointKind,
}

pub(super) fn validate_inputs(
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
    dynamic_masses: &[DynamicMassContribution],
) -> Result<(), MultibodyError> {
    if !base_state.position_world_m.is_finite() {
        return Err(MultibodyError::InvalidParameter {
            field: "position_world_m",
            value: base_state.position_world_m.length(),
        });
    }
    if !base_state.rotation_world_from_root.is_finite()
        || (base_state.rotation_world_from_root.length_squared() - 1.0).abs() > 1e-6
    {
        return Err(MultibodyError::InvalidParameter {
            field: "rotation_world_from_root",
            value: base_state.rotation_world_from_root.length(),
        });
    }
    if !base_state.linear_vel_root_m_s.is_finite() || !base_state.angular_vel_root_rad_s.is_finite()
    {
        return Err(MultibodyError::InvalidParameter {
            field: "base_velocity",
            value: 0.0,
        });
    }
    if !gravity_world.is_finite() {
        return Err(MultibodyError::InvalidParameter {
            field: "gravity_world_m_s2",
            value: gravity_world.length(),
        });
    }
    for m in dynamic_masses {
        m.validate()?;
    }
    Ok(())
}

pub(super) fn build_link_chain(
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    drives: &[JointDriveInput],
    wrenches: &[WrenchAtSite],
) -> Result<Vec<LinkData>, MultibodyError> {
    let poses = tree.evaluate_pose_table()?;
    let order = topological_attached_ids(tree, &poses)?;
    if order.is_empty() {
        return Err(MultibodyError::Mbd(MbdError::NoAttachedMass));
    }
    let mut id_to_order = vec![None; tree.len()];
    for (pos, &id) in order.iter().enumerate() {
        if let Some(slot) = id_to_order.get_mut(id.0 as usize) {
            *slot = Some(pos);
        }
    }
    let mut links = Vec::with_capacity(order.len());
    for (pos, &id) in order.iter().enumerate() {
        let spec = tree.spec(id)?;
        let state = tree.state(id)?;
        let parent_order = spec
            .placement
            .parent
            .and_then(|pid| id_to_order.get(pid.0 as usize).and_then(|v| *v));
        if pos > 0 && parent_order.is_none() {
            return Err(MultibodyError::InvalidChain {
                detail: alloc::format!("multiple root bodies unsupported above {id}"),
            });
        }
        links.push(build_one_link(
            id,
            parent_order,
            spec,
            state,
            dynamic_masses,
            drives,
            wrenches,
        )?);
    }
    for w in wrenches {
        tree.spec(w.site)?;
    }
    Ok(links)
}

fn topological_attached_ids(
    tree: &ComponentTree,
    poses: &mbd_core::PoseTable,
) -> Result<Vec<ComponentId>, MultibodyError> {
    let mut visited = vec![false; tree.len()];
    let mut order = Vec::with_capacity(tree.len());
    let mut stack = Vec::new();
    for id in tree.ids() {
        if !poses.get(id)?.attached || visited.get(id.0 as usize).copied().unwrap_or(false) {
            continue;
        }
        stack.clear();
        let mut cursor = Some(id);
        while let Some(cur) = cursor {
            if visited.get(cur.0 as usize).copied().unwrap_or(false) {
                break;
            }
            stack.push(cur);
            cursor = tree.spec(cur)?.placement.parent;
        }
        while let Some(node) = stack.pop() {
            if poses.get(node)?.attached {
                if let Some(flag) = visited.get_mut(node.0 as usize) {
                    *flag = true;
                }
                order.push(node);
            }
        }
    }
    Ok(order)
}

fn build_one_link(
    id: ComponentId,
    parent_order: Option<usize>,
    spec: &ComponentSpec,
    state: &ComponentState,
    dynamic_masses: &[DynamicMassContribution],
    drives: &[JointDriveInput],
    wrenches: &[WrenchAtSite],
) -> Result<LinkData, MultibodyError> {
    let (j_trans, j_rot) = spec.joint.local_transform(&state.joint)?;
    let (j_vel, j_omega) = spec.joint.local_velocity(&state.joint);
    let q_p_from_c = (spec.placement.rotation * j_rot).normalize();
    let p_c_in_p = spec.placement.translation_m + spec.placement.rotation * j_trans;
    let x_parent_from_child = SpatialTransform::from_pos_quat(p_c_in_p, q_p_from_c);
    let j_rot_inv = j_rot.conjugate();
    let v_j_child = SpatialMotion::new(j_rot_inv * j_omega, j_rot_inv * j_vel);

    let (mass_kg, cg_local_m, inertia_cg_local) =
        combine_link_mass(id, spec, state, dynamic_masses);
    let i_local = SpatialMat6::from_rigid_inertia(mass_kg, cg_local_m, inertia_cg_local);
    let ext_force_local = sum_link_external_wrenches(id, wrenches);
    let drive = drives.iter().find(|d| d.component == id).copied();
    let (s_1dof, tau_1dof, tau_3d_child, force_3d_child) =
        compute_joint_efforts(spec, state, j_rot, j_rot_inv, drive);

    Ok(LinkData {
        id,
        parent_order,
        x_parent_from_child,
        i_local,
        v_j_child,
        s_1dof,
        q_j_rot: j_rot,
        tau_1dof,
        tau_3d_child,
        force_3d_child,
        ext_force_local,
        mass_kg,
        cg_local_m,
        inertia_cg_local,
        kind: spec.joint.kind,
    })
}

fn combine_link_mass(
    id: ComponentId,
    spec: &ComponentSpec,
    state: &ComponentState,
    dynamic_masses: &[DynamicMassContribution],
) -> (f64, DVec3, DMat3) {
    let base_mass = (spec.dry_mass_kg + state.dry_mass_delta_kg).max(0.0)
        + state.fluid_mass_kg
        + state.payload_mass_kg;
    let mut total_m = base_mass;
    let mut first = DVec3::ZERO;
    let matching = dynamic_masses.iter().filter(|d| d.component == id);
    for extra in matching.clone() {
        total_m += extra.mass_kg;
        first += extra.cg_local_m * extra.mass_kg;
    }
    if total_m <= 0.0 {
        return (0.0, DVec3::ZERO, spec.dry_inertia_body);
    }
    let cg = first / total_m;
    let shift_base = DMat3::from_diagonal(DVec3::splat(cg.length_squared())) - outer3(cg, cg);
    let mut total_i = spec.dry_inertia_body + shift_base * base_mass;
    for extra in matching {
        let d = extra.cg_local_m - cg;
        let transfer = DMat3::from_diagonal(DVec3::splat(d.length_squared())) - outer3(d, d);
        total_i += extra.inertia_cg_local + transfer * extra.mass_kg;
    }
    (total_m, cg, total_i)
}

fn compute_joint_efforts(
    spec: &ComponentSpec,
    state: &ComponentState,
    j_rot: DQuat,
    j_rot_inv: DQuat,
    drive: Option<JointDriveInput>,
) -> (SpatialMotion, f64, DVec3, DVec3) {
    let raw_scalar = drive.map_or(0.0, |d| d.effort_scalar);
    let act = spec.joint.actuator;
    let clamped = act.map_or(raw_scalar, |a| {
        raw_scalar.clamp(-a.max_effort, a.max_effort)
    });
    let passive = act.map_or(0.0, |a| {
        -a.stiffness * state.joint.q_scalar - a.damping * state.joint.q_vel_scalar
    });
    match spec.joint.kind {
        JointKind::Fixed => (SpatialMotion::ZERO, 0.0, DVec3::ZERO, DVec3::ZERO),
        JointKind::Revolute {
            axis,
            min_rad,
            max_rad,
        } => (
            SpatialMotion::new(axis, DVec3::ZERO),
            clamped
                + passive
                + scalar_stop(
                    state.joint.q_scalar,
                    state.joint.q_vel_scalar,
                    min_rad,
                    max_rad,
                ),
            DVec3::ZERO,
            DVec3::ZERO,
        ),
        JointKind::Prismatic { axis, min_m, max_m } => (
            SpatialMotion::new(DVec3::ZERO, axis),
            clamped
                + passive
                + scalar_stop(state.joint.q_scalar, state.joint.q_vel_scalar, min_m, max_m),
            DVec3::ZERO,
            DVec3::ZERO,
        ),
        JointKind::Spherical { .. } | JointKind::Compliant6Dof { .. } => {
            multi_dof_efforts(spec.joint.kind, state, j_rot, j_rot_inv, drive)
        }
    }
}

fn multi_dof_efforts(
    kind: JointKind,
    state: &ComponentState,
    j_rot: DQuat,
    j_rot_inv: DQuat,
    drive: Option<JointDriveInput>,
) -> (SpatialMotion, f64, DVec3, DVec3) {
    let raw_t3 = drive.map_or(DVec3::ZERO, |d| d.torque_3d_nm);
    let raw_f3 = drive.map_or(DVec3::ZERO, |d| d.force_3d_n);
    match kind {
        JointKind::Spherical {
            cone_half_angle_rad,
        } => {
            let (ax, angle) = j_rot.to_axis_angle();
            let cone_stop = if angle > cone_half_angle_rad {
                -ax * ((angle - cone_half_angle_rad) * JOINT_LIMIT_STIFFNESS)
                    - state.joint.angular_vel_rad_s * JOINT_LIMIT_DAMPING
            } else {
                DVec3::ZERO
            };
            (
                SpatialMotion::ZERO,
                0.0,
                j_rot_inv * (raw_t3 + cone_stop),
                DVec3::ZERO,
            )
        }
        JointKind::Compliant6Dof {
            linear_stiffness_n_m,
            linear_damping_n_s_m,
            angular_stiffness_nm_rad,
            angular_damping_nm_s_rad,
        } => {
            let spring_t = -j_rot.to_scaled_axis() * angular_stiffness_nm_rad
                - state.joint.angular_vel_rad_s * angular_damping_nm_s_rad;
            let spring_f = -state.joint.translation_m * linear_stiffness_n_m
                - state.joint.linear_vel_m_s * linear_damping_n_s_m;
            (
                SpatialMotion::ZERO,
                0.0,
                j_rot_inv * (raw_t3 + spring_t),
                j_rot_inv * (raw_f3 + spring_f),
            )
        }
        _ => (SpatialMotion::ZERO, 0.0, DVec3::ZERO, DVec3::ZERO),
    }
}

fn scalar_stop(q: f64, q_dot: f64, min: f64, max: f64) -> f64 {
    if q < min {
        (min - q) * JOINT_LIMIT_STIFFNESS - q_dot * JOINT_LIMIT_DAMPING
    } else if q > max {
        (max - q) * JOINT_LIMIT_STIFFNESS - q_dot * JOINT_LIMIT_DAMPING
    } else {
        0.0
    }
}

fn sum_link_external_wrenches(id: ComponentId, wrenches: &[WrenchAtSite]) -> SpatialForce {
    let mut total = SpatialForce::ZERO;
    for w in wrenches {
        if w.site == id {
            total += SpatialForce::new(
                w.torque_local_nm + w.point_local_m.cross(w.force_local_n),
                w.force_local_n,
            );
        }
    }
    total
}
