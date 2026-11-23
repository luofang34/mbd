use alloc::vec;
use alloc::vec::Vec;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointKind,
    MbdError, WrenchAtSite,
};

use super::{FloatingBaseState, JointDriveInput, JointLoadInput};
use crate::MultibodyError;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion, SpatialTransform, outer3};

/// Every joint-space effort source handed to one forward-dynamics call.
#[derive(Clone, Copy)]
pub(super) struct JointEfforts<'a> {
    /// Actuator commands: summed per joint, then limited by the actuator.
    pub(super) drives: &'a [JointDriveInput],
    /// Constraint, spring and mechanism loads: summed and never limited.
    pub(super) loads: &'a [JointLoadInput],
}

/// Net applied joint effort in placed axes, before passive terms.
#[derive(Clone, Copy, Default)]
struct AppliedEffort {
    scalar: f64,
    torque: DVec3,
    force: DVec3,
}

impl JointEfforts<'_> {
    pub(super) const NONE: JointEfforts<'static> = JointEfforts {
        drives: &[],
        loads: &[],
    };

    fn applied_to(&self, id: ComponentId, spec: &ComponentSpec) -> AppliedEffort {
        let mut cmd = AppliedEffort::default();
        for d in self.drives.iter().filter(|d| d.component == id) {
            cmd.scalar += d.effort_scalar;
            cmd.torque += d.torque_3d_nm;
            cmd.force += d.force_3d_n;
        }
        if let Some(act) = spec.joint.actuator {
            cmd.scalar = cmd.scalar.clamp(-act.max_effort, act.max_effort);
        }
        for l in self.loads.iter().filter(|l| l.component == id) {
            cmd.scalar += l.effort_scalar;
            cmd.torque += l.torque_3d_nm;
            cmd.force += l.force_3d_n;
        }
        cmd
    }
}

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
    /// Travel-stop acceleration demand along `s_1dof`; the inward pass turns
    /// it into a torque with the joint's articulated inertia.
    pub(super) stop_accel_1dof: f64,
    /// Spherical cone-stop angular acceleration demand, child axes.
    pub(super) stop_accel_child: DVec3,
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
    efforts: JointEfforts<'_>,
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
        // The root is the floating base itself; a joint there has no parent
        // body to move against, so its coordinate would be silently frozen.
        if pos == 0 && !matches!(spec.joint.kind, JointKind::Fixed) {
            return Err(MultibodyError::NonFixedRootJoint { id });
        }
        links.push(build_one_link(
            id,
            parent_order,
            spec,
            state,
            dynamic_masses,
            &efforts,
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
    efforts: &JointEfforts<'_>,
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
    let effort = joint_effort(spec, state, j_rot, efforts.applied_to(id, spec));

    Ok(LinkData {
        id,
        parent_order,
        x_parent_from_child,
        i_local,
        v_j_child,
        s_1dof: effort.s_1dof,
        q_j_rot: j_rot,
        tau_1dof: effort.tau_1dof,
        tau_3d_child: j_rot_inv * effort.torque_placed,
        force_3d_child: j_rot_inv * effort.force_placed,
        stop_accel_1dof: effort.stop_1dof,
        stop_accel_child: j_rot_inv * effort.stop_placed,
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

/// Joint-space generalized efforts and stop demand of one link, in placed axes.
#[derive(Default)]
struct JointEffort {
    s_1dof: SpatialMotion,
    tau_1dof: f64,
    torque_placed: DVec3,
    force_placed: DVec3,
    stop_1dof: f64,
    stop_placed: DVec3,
}

fn joint_effort(
    spec: &ComponentSpec,
    state: &ComponentState,
    j_rot: DQuat,
    applied: AppliedEffort,
) -> JointEffort {
    let js = &state.joint;
    let stop = spec.joint.stop;
    let passive = spec.joint.actuator.map_or(0.0, |a| {
        -a.stiffness * js.q_scalar - a.damping * js.q_vel_scalar
    });
    let scalar = |s_1dof, min: f64, max: f64| JointEffort {
        s_1dof,
        tau_1dof: applied.scalar + passive,
        stop_1dof: overrun(js.q_scalar, min, max)
            .map_or(0.0, |over| stop.accel_demand(over, js.q_vel_scalar)),
        ..JointEffort::default()
    };
    match spec.joint.kind {
        JointKind::Fixed => JointEffort::default(),
        JointKind::Revolute {
            axis,
            min_rad,
            max_rad,
        } => scalar(SpatialMotion::new(axis, DVec3::ZERO), min_rad, max_rad),
        JointKind::Prismatic { axis, min_m, max_m } => {
            scalar(SpatialMotion::new(DVec3::ZERO, axis), min_m, max_m)
        }
        JointKind::Spherical {
            cone_half_angle_rad,
        } => {
            let (ax, angle) = j_rot.to_axis_angle();
            let stop_placed = if angle > cone_half_angle_rad {
                stop.accel_demand_vec(-ax * (angle - cone_half_angle_rad), js.angular_vel_rad_s)
            } else {
                DVec3::ZERO
            };
            JointEffort {
                torque_placed: applied.torque,
                stop_placed,
                ..JointEffort::default()
            }
        }
        JointKind::Compliant6Dof {
            linear_stiffness_n_m,
            linear_damping_n_s_m,
            angular_stiffness_nm_rad,
            angular_damping_nm_s_rad,
        } => JointEffort {
            torque_placed: applied.torque
                - j_rot.to_scaled_axis() * angular_stiffness_nm_rad
                - js.angular_vel_rad_s * angular_damping_nm_s_rad,
            force_placed: applied.force
                - js.translation_m * linear_stiffness_n_m
                - js.linear_vel_m_s * linear_damping_n_s_m,
            ..JointEffort::default()
        },
    }
}

/// Signed distance from `q` back into `[min, max]`, or `None` inside it.
fn overrun(q: f64, min: f64, max: f64) -> Option<f64> {
    if q < min {
        Some(min - q)
    } else if q > max {
        Some(max - q)
    } else {
        None
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
