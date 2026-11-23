use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{ComponentId, ComponentTree, JointKind, PoseTable, WrenchAtSite};

use crate::MultibodyError;
use crate::aba::JointDriveInput;

#[cfg(test)]
mod tests;

/// Closed-loop kinematic constraint evaluated inside a [`ComponentTree`] (e.g.,
/// Mars Perseverance rocker-bogie differential bar or closed robotic grapple).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoopConstraint {
    /// Enforces `q_a + ratio * q_b == offset` between two 1-DoF joints via
    /// balanced equal-and-opposite internal joint efforts (differential linkage).
    JointRatio {
        /// First 1-DoF joint component.
        joint_a: ComponentId,
        /// Second 1-DoF joint component.
        joint_b: ComponentId,
        /// Kinematic ratio multiplier (e.g. `+1.0` for `q_L + q_R == 0`).
        ratio: f64,
        /// Desired scalar offset, rad or m.
        offset: f64,
        /// Penalty spring stiffness, N·m/rad or N/m.
        stiffness: f64,
        /// Penalty viscous damping, N·m·s/rad or N·s/m.
        damping: f64,
    },
    /// Bilateral compliant weld tying a point on `site_a` to a point on `site_b`.
    PointWeld {
        /// First component.
        site_a: ComponentId,
        /// Anchor offset on `site_a` in `site_a` local axes, m.
        offset_a_m: DVec3,
        /// Second component.
        site_b: ComponentId,
        /// Anchor offset on `site_b` in `site_b` local axes, m.
        offset_b_m: DVec3,
        /// Translational stiffness, N/m.
        linear_stiffness_n_m: f64,
        /// Translational damping, N·s/m.
        linear_damping_n_s_m: f64,
    },
    /// Unilateral tether/stop limiting distance between two sites to `<= max_length_m`.
    UnilateralDistance {
        /// First component.
        site_a: ComponentId,
        /// Anchor offset on `site_a`, m.
        offset_a_m: DVec3,
        /// Second component.
        site_b: ComponentId,
        /// Anchor offset on `site_b`, m.
        offset_b_m: DVec3,
        /// Maximum taut length before internal constraint engages, m.
        max_length_m: f64,
        /// Axial tensile stiffness, N/m.
        stiffness_n_m: f64,
        /// Axial tensile damping, N·s/m.
        damping_n_s_m: f64,
    },
}

/// Internal equal-and-opposite wrenches and joint drives returned by [`evaluate_loop_constraints`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LoopOutput {
    /// Local equal-and-opposite constraint wrenches ready for [`crate::MultiBodyDynamics::forward_dynamics`].
    pub wrenches: Vec<WrenchAtSite>,
    /// Internal joint-space efforts enforcing differential / closed-loop linkages.
    pub joint_drives: Vec<JointDriveInput>,
}

/// Evaluates every closed-loop constraint cleanly, producing strictly balanced
/// internal actions with zero net external force or moment on the vehicle.
pub fn evaluate_loop_constraints(
    tree: &ComponentTree,
    poses: &PoseTable,
    constraints: &[LoopConstraint],
) -> Result<LoopOutput, MultibodyError> {
    let mut out = LoopOutput::default();
    for &constraint in constraints {
        match constraint {
            LoopConstraint::JointRatio {
                joint_a,
                joint_b,
                ratio,
                offset,
                stiffness,
                damping,
            } => {
                if !tree.is_attached(joint_a)? || !tree.is_attached(joint_b)? {
                    continue;
                }
                check_1dof(tree, joint_a)?;
                check_1dof(tree, joint_b)?;
                let qa = tree.state(joint_a)?.joint;
                let qb = tree.state(joint_b)?.joint;
                let err = (qa.q_scalar + ratio * qb.q_scalar) - offset;
                let err_dot = qa.q_vel_scalar + ratio * qb.q_vel_scalar;
                let force_eq = -(stiffness * err + damping * err_dot);
                out.joint_drives
                    .push(JointDriveInput::scalar(joint_a, force_eq));
                out.joint_drives
                    .push(JointDriveInput::scalar(joint_b, force_eq * ratio));
            }
            LoopConstraint::PointWeld {
                site_a,
                offset_a_m,
                site_b,
                offset_b_m,
                linear_stiffness_n_m,
                linear_damping_n_s_m,
            } => {
                eval_bilateral_weld(
                    poses,
                    site_a,
                    offset_a_m,
                    site_b,
                    offset_b_m,
                    linear_stiffness_n_m,
                    linear_damping_n_s_m,
                    &mut out.wrenches,
                )?;
            }
            LoopConstraint::UnilateralDistance {
                site_a,
                offset_a_m,
                site_b,
                offset_b_m,
                max_length_m,
                stiffness_n_m,
                damping_n_s_m,
            } => {
                eval_unilateral_dist(
                    poses,
                    site_a,
                    offset_a_m,
                    site_b,
                    offset_b_m,
                    max_length_m,
                    stiffness_n_m,
                    damping_n_s_m,
                    &mut out.wrenches,
                )?;
            }
        }
    }
    Ok(out)
}

fn check_1dof(tree: &ComponentTree, id: ComponentId) -> Result<(), MultibodyError> {
    if matches!(
        tree.spec(id)?.joint.kind,
        JointKind::Revolute { .. } | JointKind::Prismatic { .. }
    ) {
        return Ok(());
    }
    Err(MultibodyError::WrongJointKind { id })
}

#[allow(clippy::too_many_arguments)]
fn eval_bilateral_weld(
    poses: &PoseTable,
    site_a: ComponentId,
    offset_a_m: DVec3,
    site_b: ComponentId,
    offset_b_m: DVec3,
    stiffness: f64,
    damping: f64,
    wrenches: &mut Vec<WrenchAtSite>,
) -> Result<(), MultibodyError> {
    let pa = poses.get(site_a)?;
    let pb = poses.get(site_b)?;
    if !pa.attached || !pb.attached {
        return Ok(());
    }
    let ra = pa.rotation_root_from_link * offset_a_m;
    let rb = pb.rotation_root_from_link * offset_b_m;
    let xa = pa.position_root_m + ra;
    let xb = pb.position_root_m + rb;
    let va = pa.linear_vel_root_m_s + pa.angular_vel_root_rad_s.cross(ra);
    let vb = pb.linear_vel_root_m_s + pb.angular_vel_root_rad_s.cross(rb);
    // Spring pulls A toward B.
    let f_on_a_root = (xb - xa) * stiffness + (vb - va) * damping;
    push_balanced_pair(pa, offset_a_m, pb, offset_b_m, f_on_a_root, wrenches);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn eval_unilateral_dist(
    poses: &PoseTable,
    site_a: ComponentId,
    offset_a_m: DVec3,
    site_b: ComponentId,
    offset_b_m: DVec3,
    max_length_m: f64,
    stiffness: f64,
    damping: f64,
    wrenches: &mut Vec<WrenchAtSite>,
) -> Result<(), MultibodyError> {
    let pa = poses.get(site_a)?;
    let pb = poses.get(site_b)?;
    if !pa.attached || !pb.attached {
        return Ok(());
    }
    let ra = pa.rotation_root_from_link * offset_a_m;
    let rb = pb.rotation_root_from_link * offset_b_m;
    let delta = (pb.position_root_m + rb) - (pa.position_root_m + ra);
    let dist = delta.length();
    if dist <= max_length_m || dist <= 1e-12 {
        return Ok(());
    }
    let dir = delta / dist;
    let va = pa.linear_vel_root_m_s + pa.angular_vel_root_rad_s.cross(ra);
    let vb = pb.linear_vel_root_m_s + pb.angular_vel_root_rad_s.cross(rb);
    let v_rel = (vb - va).dot(dir);
    let tension = ((dist - max_length_m) * stiffness + v_rel * damping).max(0.0);
    push_balanced_pair(pa, offset_a_m, pb, offset_b_m, dir * tension, wrenches);
    Ok(())
}

fn push_balanced_pair(
    pa: &mbd_core::LinkPose,
    off_a: DVec3,
    pb: &mbd_core::LinkPose,
    off_b: DVec3,
    f_on_a_root: DVec3,
    wrenches: &mut Vec<WrenchAtSite>,
) {
    wrenches.push(WrenchAtSite {
        site: pa.id,
        point_local_m: off_a,
        force_local_n: pa.rotation_root_from_link.conjugate() * f_on_a_root,
        torque_local_nm: DVec3::ZERO,
    });
    wrenches.push(WrenchAtSite {
        site: pb.id,
        point_local_m: off_b,
        force_local_n: pb.rotation_root_from_link.conjugate() * (-f_on_a_root),
        torque_local_nm: DVec3::ZERO,
    });
}
