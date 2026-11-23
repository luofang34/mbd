//! Link chain of a component tree: the topology and constants prepared once,
//! and the per-evaluation link records the articulated-body passes consume.

use alloc::vec;
use alloc::vec::Vec;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointKind,
    JointSpec, JointState, MbdError, Placement,
};

use super::FloatingBaseState;
use crate::MultibodyError;
use crate::model::layout::LinkSlots;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion, SpatialTransform, outer3};

/// Net applied joint effort in placed axes, before passive terms.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct AppliedEffort {
    pub(crate) scalar: f64,
    pub(crate) torque: DVec3,
    pub(crate) force: DVec3,
}

/// One attached link: everything about it that does not change while the
/// tree's topology and mass stay fixed.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PreparedLink {
    pub(crate) id: ComponentId,
    pub(crate) parent_order: Option<usize>,
    pub(crate) placement: Placement,
    pub(crate) joint: JointSpec,
    pub(crate) slots: LinkSlots,
    pub(crate) mass_kg: f64,
    pub(crate) cg_local_m: DVec3,
    pub(crate) inertia_cg_local: DMat3,
    pub(crate) i_local: SpatialMat6,
}

/// Pose and motion of one link at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LinkFrame {
    pub(crate) joint: JointState,
    pub(crate) x_parent_from_child: SpatialTransform,
    pub(crate) q_j_rot: DQuat,
    pub(crate) v_j_child: SpatialMotion,
    /// World-from-link rotation.
    pub(crate) r_world: DMat3,
    /// Link origin in world axes, m.
    pub(crate) p_world: DVec3,
    /// Absolute spatial velocity in link axes.
    pub(crate) v: SpatialMotion,
}

/// Link record for one forward-dynamics solve.
#[derive(Debug, Clone, Copy)]
pub(crate) struct LinkData {
    pub(crate) id: ComponentId,
    pub(crate) parent_order: Option<usize>,
    pub(crate) x_parent_from_child: SpatialTransform,
    /// Inertia that gravity acts on.
    pub(crate) i_local: SpatialMat6,
    /// Inertia that resists acceleration: `i_local` plus any added inertia.
    pub(crate) i_inertial: SpatialMat6,
    pub(crate) v_j_child: SpatialMotion,
    pub(crate) s_1dof: SpatialMotion,
    pub(crate) q_j_rot: DQuat,
    pub(crate) tau_1dof: f64,
    pub(crate) tau_3d_child: DVec3,
    pub(crate) force_3d_child: DVec3,
    /// Travel-stop acceleration demand along `s_1dof`; the inward pass turns
    /// it into a torque with the joint's articulated inertia.
    pub(crate) stop_accel_1dof: f64,
    /// Spherical cone-stop angular acceleration demand, child axes.
    pub(crate) stop_accel_child: DVec3,
    pub(crate) ext_force_local: SpatialForce,
    pub(crate) kind: JointKind,
}

/// Everything applied to one link during one evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct LinkLoads {
    /// Summed actuator commands, limited by the actuator before use.
    pub(crate) drive: AppliedEffort,
    /// Summed unlimited joint loads, absent while none was added so that an
    /// unloaded joint's command is used exactly as limited.
    pub(crate) load: Option<AppliedEffort>,
    /// Summed external wrench about the link origin, link axes.
    pub(crate) wrench: SpatialForce,
    /// Added inertia about the link origin, link axes.
    pub(crate) added_inertia: Option<SpatialMat6>,
}

pub(crate) fn validate_base(base_state: &FloatingBaseState) -> Result<(), MultibodyError> {
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
    Ok(())
}

/// Attached links of `tree` in parent-before-child order, with the chain
/// position of every component id.
pub(crate) fn prepare_links(
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    slots: &[LinkSlots],
) -> Result<(Vec<PreparedLink>, Vec<Option<usize>>), MultibodyError> {
    for m in dynamic_masses {
        m.validate()?;
    }
    let order = attached_order(tree)?;
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
        let (mass_kg, cg_local_m, inertia_cg_local) =
            combine_link_mass(id, spec, state, dynamic_masses);
        links.push(PreparedLink {
            id,
            parent_order,
            placement: spec.placement,
            joint: spec.joint,
            slots: slots.get(pos).copied().unwrap_or_default(),
            mass_kg,
            cg_local_m,
            inertia_cg_local,
            i_local: SpatialMat6::from_rigid_inertia(mass_kg, cg_local_m, inertia_cg_local),
        });
    }
    Ok((links, id_to_order))
}

/// Joint kinds of the attached links of `tree`, in chain order.
pub(crate) fn attached_joint_kinds(tree: &ComponentTree) -> Result<Vec<JointKind>, MultibodyError> {
    let order = attached_order(tree)?;
    let mut kinds = Vec::with_capacity(order.len());
    for id in order {
        kinds.push(tree.spec(id)?.joint.kind);
    }
    Ok(kinds)
}

fn attached_order(tree: &ComponentTree) -> Result<Vec<ComponentId>, MultibodyError> {
    let poses = tree.evaluate_pose_table()?;
    let order = topological_attached_ids(tree, &poses)?;
    if order.is_empty() {
        return Err(MultibodyError::Mbd(MbdError::NoAttachedMass));
    }
    Ok(order)
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

/// Recomputes the mass properties of `link` from the tree's current masses.
pub(crate) fn refresh_link_mass(
    link: &mut PreparedLink,
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
) -> Result<(), MultibodyError> {
    let (mass_kg, cg, inertia) = combine_link_mass(
        link.id,
        tree.spec(link.id)?,
        tree.state(link.id)?,
        dynamic_masses,
    );
    link.mass_kg = mass_kg;
    link.cg_local_m = cg;
    link.inertia_cg_local = inertia;
    link.i_local = SpatialMat6::from_rigid_inertia(mass_kg, cg, inertia);
    Ok(())
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

/// Pose and velocity of `link` at joint state `joint`, given its parent's
/// frame, or the base state for the root.
pub(crate) fn link_frame(
    link: &PreparedLink,
    joint: JointState,
    parent: Option<&LinkFrame>,
    base: &FloatingBaseState,
) -> Result<LinkFrame, MultibodyError> {
    let (j_trans, j_rot) = link.joint.local_transform(&joint)?;
    let (j_vel, j_omega) = link.joint.local_velocity(&joint);
    let q_p_from_c = (link.placement.rotation * j_rot).normalize();
    let p_c_in_p = link.placement.translation_m + link.placement.rotation * j_trans;
    let x_parent_from_child = SpatialTransform::from_pos_quat(p_c_in_p, q_p_from_c);
    let j_rot_inv = j_rot.conjugate();
    let v_j_child = SpatialMotion::new(j_rot_inv * j_omega, j_rot_inv * j_vel);
    let (r_world, p_world, v) = match parent {
        Some(p) => (
            p.r_world * x_parent_from_child.r_parent_from_child,
            p.p_world + p.r_world * x_parent_from_child.p_child_in_parent,
            x_parent_from_child.apply_motion_to_child(p.v) + v_j_child,
        ),
        None => (
            DMat3::from_quat(base.rotation_world_from_root),
            base.position_world_m,
            base.spatial_velocity_root(),
        ),
    };
    Ok(LinkFrame {
        joint,
        x_parent_from_child,
        q_j_rot: j_rot,
        v_j_child,
        r_world,
        p_world,
        v,
    })
}

/// Which joint-space terms a solve includes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EffortTerms {
    /// Applied efforts, actuator limits, passive springs and dampers, stops.
    Full,
    /// Applied efforts only, unlimited: the map from generalized force to
    /// acceleration with every state-dependent term removed.
    AppliedOnly,
}

/// Assembles the solve record of `link` from its frame and applied loads.
pub(crate) fn assemble_link(
    link: &PreparedLink,
    frame: &LinkFrame,
    loads: &LinkLoads,
    terms: EffortTerms,
) -> LinkData {
    let mut applied = loads.drive;
    if let (Some(act), EffortTerms::Full) = (link.joint.actuator, terms) {
        applied.scalar = applied.scalar.clamp(-act.max_effort, act.max_effort);
    }
    if let Some(load) = loads.load {
        applied.scalar += load.scalar;
        applied.torque += load.torque;
        applied.force += load.force;
    }
    let effort = joint_effort(&link.joint, &frame.joint, frame.q_j_rot, applied, terms);
    let j_rot_inv = frame.q_j_rot.conjugate();
    LinkData {
        id: link.id,
        parent_order: link.parent_order,
        x_parent_from_child: frame.x_parent_from_child,
        i_local: link.i_local,
        i_inertial: loads
            .added_inertia
            .map_or(link.i_local, |added| link.i_local + added),
        v_j_child: frame.v_j_child,
        s_1dof: effort.s_1dof,
        q_j_rot: frame.q_j_rot,
        tau_1dof: effort.tau_1dof,
        tau_3d_child: j_rot_inv * effort.torque_placed,
        force_3d_child: j_rot_inv * effort.force_placed,
        stop_accel_1dof: effort.stop_1dof,
        stop_accel_child: j_rot_inv * effort.stop_placed,
        ext_force_local: loads.wrench,
        kind: link.joint.kind,
    }
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
    spec: &JointSpec,
    js: &JointState,
    j_rot: DQuat,
    applied: AppliedEffort,
    terms: EffortTerms,
) -> JointEffort {
    let full = terms == EffortTerms::Full;
    let stop = spec.stop;
    let passive = match spec.actuator {
        Some(a) if full => -a.stiffness * js.q_scalar - a.damping * js.q_vel_scalar,
        _ => 0.0,
    };
    let scalar = |s_1dof, min: f64, max: f64| JointEffort {
        s_1dof,
        tau_1dof: applied.scalar + passive,
        stop_1dof: overrun(js.q_scalar, min, max)
            .filter(|_| full)
            .map_or(0.0, |over| stop.accel_demand(over, js.q_vel_scalar)),
        ..JointEffort::default()
    };
    match spec.kind {
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
            let stop_placed = if full && angle > cone_half_angle_rad {
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
        JointKind::Compliant6Dof { .. } if !full => JointEffort {
            torque_placed: applied.torque,
            force_placed: applied.force,
            ..JointEffort::default()
        },
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
