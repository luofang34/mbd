use alloc::vec;
use alloc::vec::Vec;

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, DynamicMassContribution, JointKind};

use crate::MultibodyError;
use crate::aba::MultiBodyDynamics;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion, SpatialTransform, outer3};

mod collision;

#[cfg(test)]
mod tests;

pub use collision::{LinkCapsule, minimum_capsule_clearance};

/// Open kinematic manipulator chain inside a [`ComponentTree`] used for
/// forward kinematics, free-floating Generalized Jacobian $J^*$, damped
/// least-squares inverse kinematics (DLS-IK), and null-space joint avoidance.
#[derive(Debug, Clone, PartialEq)]
pub struct ManipulatorChain {
    joint_ids: Vec<ComponentId>,
    ee_component: ComponentId,
    ee_offset_local_m: DVec3,
}

impl ManipulatorChain {
    /// Builds and validates a manipulator chain from ordered 1-DoF joint
    /// components ending at `ee_component` with end-effector local offset.
    pub fn new(
        tree: &ComponentTree,
        joint_ids: Vec<ComponentId>,
        ee_component: ComponentId,
        ee_offset_local_m: DVec3,
    ) -> Result<Self, MultibodyError> {
        if joint_ids.is_empty() {
            return Err(MultibodyError::InvalidChain {
                detail: alloc::string::String::from("manipulator chain has no joints"),
            });
        }
        if !ee_offset_local_m.is_finite() {
            return Err(MultibodyError::InvalidParameter {
                field: "ee_offset_local_m",
                value: ee_offset_local_m.length(),
            });
        }
        for &id in &joint_ids {
            let spec = tree.spec(id)?;
            if !tree.is_attached(id)? {
                return Err(MultibodyError::InvalidChain {
                    detail: alloc::format!("joint {id} is detached"),
                });
            }
            if !matches!(
                spec.joint.kind,
                JointKind::Revolute { .. } | JointKind::Prismatic { .. }
            ) {
                return Err(MultibodyError::WrongJointKind { id });
            }
        }
        if !tree.is_attached(ee_component)? {
            return Err(MultibodyError::InvalidChain {
                detail: alloc::format!("end effector {ee_component} is detached"),
            });
        }
        Ok(Self {
            joint_ids,
            ee_component,
            ee_offset_local_m,
        })
    }

    /// Joint component IDs belonging to this manipulator chain.
    pub fn joint_ids(&self) -> &[ComponentId] {
        &self.joint_ids
    }

    /// Component ID carrying the robotic arm end-effector frame.
    pub const fn ee_component(&self) -> ComponentId {
        self.ee_component
    }

    /// End-effector offset expressed in `ee_component` local axes, m.
    pub const fn ee_offset_local_m(&self) -> DVec3 {
        self.ee_offset_local_m
    }

    /// Forward kinematics: end-effector `(position_root_m, rotation_root_from_ee)`.
    pub fn end_effector_pose(
        &self,
        tree: &ComponentTree,
    ) -> Result<(DVec3, DQuat), MultibodyError> {
        let poses = tree.evaluate_pose_table()?;
        let ee = poses.get(self.ee_component)?;
        let pos = ee.position_root_m + ee.rotation_root_from_link * self.ee_offset_local_m;
        Ok((pos, ee.rotation_root_from_link))
    }

    /// Fixed-base geometric Jacobian columns $J_m \in \mathbb{R}^{6 \times n}$
    /// mapping $\dot{\mathbf{q}}$ to end-effector spatial twist `(ang, lin)` at
    /// the end-effector origin in root-frame orientation.
    pub fn geometric_jacobian(
        &self,
        tree: &ComponentTree,
    ) -> Result<Vec<SpatialMotion>, MultibodyError> {
        let poses = tree.evaluate_pose_table()?;
        let ee = poses.get(self.ee_component)?;
        let p_ee = ee.position_root_m + ee.rotation_root_from_link * self.ee_offset_local_m;
        let mut cols = Vec::with_capacity(self.joint_ids.len());
        for &jid in &self.joint_ids {
            cols.push(joint_column_at_point(tree, &poses, jid, p_ee)?);
        }
        Ok(cols)
    }

    /// Exact free-floating Generalized Jacobian $J^* = J_m - J_b H_b^{-1} H_{bm}$
    /// accounting for dynamic spacecraft base recoil assuming conserved zero
    /// perturbation momentum.
    pub fn generalized_jacobian(
        &self,
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
    ) -> Result<Vec<SpatialMotion>, MultibodyError> {
        let poses = tree.evaluate_pose_table()?;
        let ee = poses.get(self.ee_component)?;
        let p_ee = ee.position_root_m + ee.rotation_root_from_link * self.ee_offset_local_m;
        let h_base = MultiBodyDynamics::locked_root_spatial_inertia(tree, dynamic_masses)?;
        let subtrees_root = subtree_root_inertias(tree, &poses, dynamic_masses)?;

        let mut cols = Vec::with_capacity(self.joint_ids.len());
        for &jid in &self.joint_ids {
            let jm_col = joint_column_at_point(tree, &poses, jid, p_ee)?;
            let s_root = joint_column_at_point(tree, &poses, jid, DVec3::ZERO)?;
            let i_sub = subtrees_root
                .get(jid.0 as usize)
                .copied()
                .ok_or(MultibodyError::WrongJointKind { id: jid })?;
            let h_bm_col = i_sub.mul_motion(s_root);
            let base_twist = -h_base
                .solve(h_bm_col)
                .ok_or(MultibodyError::SingularMassMatrix)?;
            let ee_from_base =
                SpatialMotion::new(base_twist.ang, base_twist.lin + base_twist.ang.cross(p_ee));
            cols.push(jm_col + ee_from_base);
        }
        Ok(cols)
    }

    /// Damped least-squares (DLS) velocity inverse kinematics with optional
    /// redundant null-space objective projection (`null_space_vel`).
    pub fn solve_dls_ik(
        &self,
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        desired_ee_twist: SpatialMotion,
        damping: f64,
        free_floating: bool,
        null_space_vel: Option<&[f64]>,
    ) -> Result<Vec<f64>, MultibodyError> {
        if !damping.is_finite() || damping <= 0.0 || !desired_ee_twist.is_finite() {
            return Err(MultibodyError::InvalidParameter {
                field: "damping_or_twist",
                value: damping,
            });
        }
        let jac = if free_floating {
            self.generalized_jacobian(tree, dynamic_masses)?
        } else {
            self.geometric_jacobian(tree)?
        };
        if let Some(ns) = null_space_vel
            && ns.len() != jac.len()
        {
            return Err(MultibodyError::InvalidChain {
                detail: alloc::format!("expected {} null-space rates, got {}", jac.len(), ns.len()),
            });
        }
        let a_mat = dls_normal_matrix(&jac, damping);
        let y = a_mat
            .solve(SpatialForce::new(
                desired_ee_twist.ang,
                desired_ee_twist.lin,
            ))
            .ok_or(MultibodyError::SingularMassMatrix)?;

        let mut q_dot: Vec<f64> = jac
            .iter()
            .map(|col| col.ang.dot(y.ang) + col.lin.dot(y.lin))
            .collect();

        if let Some(q0) = null_space_vel {
            let mut w0 = SpatialMotion::ZERO;
            for (col, &q0_i) in jac.iter().zip(q0) {
                w0 += *col * q0_i;
            }
            let y_null = a_mat
                .solve(SpatialForce::new(w0.ang, w0.lin))
                .ok_or(MultibodyError::SingularMassMatrix)?;
            for ((qd, col), &q0_i) in q_dot.iter_mut().zip(&jac).zip(q0) {
                let proj = col.ang.dot(y_null.ang) + col.lin.dot(y_null.lin);
                *qd += q0_i - proj;
            }
        }

        for (qd, &jid) in q_dot.iter_mut().zip(&self.joint_ids) {
            if let Some(act) = tree.spec(jid)?.joint.actuator {
                *qd = qd.clamp(-act.max_velocity, act.max_velocity);
            }
        }
        Ok(q_dot)
    }

    /// Computes a joint-limit-avoidance gradient velocity vector suitable for
    /// `solve_dls_ik(..., Some(&null_vel))` on redundant 7-DoF arms.
    pub fn joint_limit_avoidance_null_vel(
        &self,
        tree: &ComponentTree,
        gain: f64,
    ) -> Result<Vec<f64>, MultibodyError> {
        let mut v = Vec::with_capacity(self.joint_ids.len());
        for &jid in &self.joint_ids {
            let spec = tree.spec(jid)?;
            let q = tree.state(jid)?.joint.q_scalar;
            let (min, max) = match spec.joint.kind {
                JointKind::Revolute {
                    min_rad, max_rad, ..
                } => (min_rad, max_rad),
                JointKind::Prismatic { min_m, max_m, .. } => (min_m, max_m),
                _ => (0.0, 0.0),
            };
            // A joint with no travel has no centre to pull toward.
            let span = max - min;
            let mid = 0.5 * (min + max);
            v.push(if span > 0.0 {
                gain * (mid - q) / (0.5 * span)
            } else {
                0.0
            });
        }
        Ok(v)
    }
}

/// 6-DoF admittance / impedance contact controller generating compliant
/// end-effector velocity commands from measured vs desired contact wrenches.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdmittanceController {
    /// Virtual translational mass, kg.
    pub virtual_mass_kg: f64,
    /// Virtual rotational inertia, kg·m².
    pub virtual_inertia_kg_m2: f64,
    /// Translational damping, N·s/m.
    pub linear_damping_n_s_m: f64,
    /// Rotational damping, N·m·s/rad.
    pub angular_damping_nm_s_rad: f64,
    /// Translational spring stiffness, N/m.
    pub linear_stiffness_n_m: f64,
    /// Rotational spring stiffness, N·m/rad.
    pub angular_stiffness_nm_rad: f64,
    offset_pos_m: DVec3,
    offset_rot_rad: DVec3,
    twist: SpatialMotion,
}

impl AdmittanceController {
    /// Creates an initialized controller at zero compliance deflection.
    pub fn new(
        virtual_mass_kg: f64,
        virtual_inertia_kg_m2: f64,
        linear_damping_n_s_m: f64,
        angular_damping_nm_s_rad: f64,
        linear_stiffness_n_m: f64,
        angular_stiffness_nm_rad: f64,
    ) -> Result<Self, MultibodyError> {
        if virtual_mass_kg <= 0.0 || virtual_inertia_kg_m2 <= 0.0 {
            return Err(MultibodyError::InvalidParameter {
                field: "virtual_mass_or_inertia",
                value: virtual_mass_kg.min(virtual_inertia_kg_m2),
            });
        }
        Ok(Self {
            virtual_mass_kg,
            virtual_inertia_kg_m2,
            linear_damping_n_s_m,
            angular_damping_nm_s_rad,
            linear_stiffness_n_m,
            angular_stiffness_nm_rad,
            offset_pos_m: DVec3::ZERO,
            offset_rot_rad: DVec3::ZERO,
            twist: SpatialMotion::ZERO,
        })
    }

    /// Current integrated virtual compliance deflection `(lin_m, ang_rad)`.
    pub const fn offset(&self) -> (DVec3, DVec3) {
        (self.offset_pos_m, self.offset_rot_rad)
    }

    /// Advances admittance dynamics over `dt_s` and returns the compliant twist command.
    pub fn step(
        &mut self,
        measured_wrench: SpatialForce,
        desired_wrench: SpatialForce,
        dt_s: f64,
    ) -> SpatialMotion {
        let err = measured_wrench - desired_wrench;
        let lin_acc = (err.force
            - self.twist.lin * self.linear_damping_n_s_m
            - self.offset_pos_m * self.linear_stiffness_n_m)
            / self.virtual_mass_kg;
        let ang_acc = (err.torque
            - self.twist.ang * self.angular_damping_nm_s_rad
            - self.offset_rot_rad * self.angular_stiffness_nm_rad)
            / self.virtual_inertia_kg_m2;
        self.twist.lin += lin_acc * dt_s;
        self.twist.ang += ang_acc * dt_s;
        self.offset_pos_m += self.twist.lin * dt_s;
        self.offset_rot_rad += self.twist.ang * dt_s;
        self.twist
    }
}

fn joint_column_at_point(
    tree: &ComponentTree,
    poses: &mbd_core::PoseTable,
    jid: ComponentId,
    point_root_m: DVec3,
) -> Result<SpatialMotion, MultibodyError> {
    let spec = tree.spec(jid)?;
    let pose = poses.get(jid)?;
    let axis_local = match spec.joint.kind {
        JointKind::Revolute { axis, .. } => axis,
        JointKind::Prismatic { axis, .. } => axis,
        _ => return Err(MultibodyError::WrongJointKind { id: jid }),
    };
    let axis_root = pose.rotation_root_from_link * axis_local;
    Ok(match spec.joint.kind {
        JointKind::Revolute { .. } => SpatialMotion::new(
            axis_root,
            axis_root.cross(point_root_m - pose.position_root_m),
        ),
        JointKind::Prismatic { .. } => SpatialMotion::new(DVec3::ZERO, axis_root),
        _ => SpatialMotion::ZERO,
    })
}

fn dls_normal_matrix(jac: &[SpatialMotion], damping: f64) -> SpatialMat6 {
    let mut a = SpatialMat6::scaled_identity(damping * damping);
    for col in jac {
        a.aa += outer3(col.ang, col.ang);
        a.al += outer3(col.ang, col.lin);
        a.la += outer3(col.lin, col.ang);
        a.ll += outer3(col.lin, col.lin);
    }
    a
}

fn subtree_root_inertias(
    tree: &ComponentTree,
    poses: &mbd_core::PoseTable,
    dynamic_masses: &[DynamicMassContribution],
) -> Result<Vec<SpatialMat6>, MultibodyError> {
    let n = tree.len();
    let mut inertias_root = vec![SpatialMat6::ZERO; n];
    for id in tree.ids() {
        let pose = poses.get(id)?;
        if !pose.attached {
            continue;
        }
        let spec = tree.spec(id)?;
        let st = tree.state(id)?;
        let base_m = (spec.dry_mass_kg + st.dry_mass_delta_kg).max(0.0)
            + st.fluid_mass_kg
            + st.payload_mass_kg;
        let x_link_to_root =
            SpatialTransform::from_pos_quat(pose.position_root_m, pose.rotation_root_from_link);
        let mut total_i =
            SpatialMat6::from_rigid_inertia(base_m, DVec3::ZERO, spec.dry_inertia_body);
        for extra in dynamic_masses.iter().filter(|d| d.component == id) {
            total_i += SpatialMat6::from_rigid_inertia(
                extra.mass_kg,
                extra.cg_local_m,
                extra.inertia_cg_local,
            );
        }
        if let Some(slot) = inertias_root.get_mut(id.0 as usize) {
            *slot = total_i.transform_to_parent(&x_link_to_root);
        }
    }
    // Fold bottom-up in depth order by accumulating edges outward-to-root via hops.
    let mut order = topological_ids(tree, poses)?;
    order.reverse();
    for id in order {
        let Some(pid) = tree.spec(id)?.placement.parent else {
            continue;
        };
        let Some(&child_i) = inertias_root.get(id.0 as usize) else {
            continue;
        };
        if let Some(parent_i) = inertias_root.get_mut(pid.0 as usize) {
            *parent_i += child_i;
        }
    }
    Ok(inertias_root)
}

fn topological_ids(
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
        let mut cur = Some(id);
        while let Some(c) = cur {
            if visited.get(c.0 as usize).copied().unwrap_or(false) {
                break;
            }
            stack.push(c);
            cur = tree.spec(c)?.placement.parent;
        }
        while let Some(n) = stack.pop() {
            if poses.get(n)?.attached {
                if let Some(f) = visited.get_mut(n.0 as usize) {
                    *f = true;
                }
                order.push(n);
            }
        }
    }
    Ok(order)
}
