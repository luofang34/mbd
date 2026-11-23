//! Closed kinematic loops as penalty constraints: balanced internal joint
//! loads and wrench pairs that are pure functions of pose and velocity.

use glam::DVec3;
use mbd_core::{ComponentId, JointKind, LinkPose, WrenchAtSite};

use crate::MultibodyError;
use crate::aba::JointLoadInput;
use crate::element::{ElementContext, ForceElement};
use crate::model::{Kinematics, Loads};

#[cfg(test)]
mod tests;

/// Closed-loop kinematic constraint evaluated inside a [`mbd_core::ComponentTree`] (e.g.,
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

fn check_1dof(id: ComponentId, kind: JointKind) -> Result<(), MultibodyError> {
    if matches!(
        kind,
        JointKind::Revolute { .. } | JointKind::Prismatic { .. }
    ) {
        return Ok(());
    }
    Err(MultibodyError::WrongJointKind { id })
}

impl LoopConstraint {
    /// Balanced joint loads of a [`LoopConstraint::JointRatio`] at joint
    /// coordinates and rates `(q_a, q̇_a, q_b, q̇_b)`; empty for other kinds.
    fn ratio_loads(&self, (qa, qa_dot, qb, qb_dot): (f64, f64, f64, f64)) -> [JointLoadInput; 2] {
        let Self::JointRatio {
            joint_a,
            joint_b,
            ratio,
            offset,
            stiffness,
            damping,
        } = *self
        else {
            return [JointLoadInput::scalar(ComponentId(0), 0.0); 2];
        };
        let err = (qa + ratio * qb) - offset;
        let err_dot = qa_dot + ratio * qb_dot;
        let force_eq = -(stiffness * err + damping * err_dot);
        [
            JointLoadInput::scalar(joint_a, force_eq),
            JointLoadInput::scalar(joint_b, force_eq * ratio),
        ]
    }

    /// Equal-and-opposite wrench pair of a point constraint between links at
    /// root-frame poses `pa` and `pb`; `None` for a joint constraint or a
    /// slack unilateral one.
    fn point_pair(&self, pa: &LinkPose, pb: &LinkPose) -> Option<[WrenchAtSite; 2]> {
        let (off_a, off_b) = match *self {
            Self::PointWeld {
                offset_a_m,
                offset_b_m,
                ..
            }
            | Self::UnilateralDistance {
                offset_a_m,
                offset_b_m,
                ..
            } => (offset_a_m, offset_b_m),
            Self::JointRatio { .. } => return None,
        };
        let ra = pa.rotation_root_from_link * off_a;
        let rb = pb.rotation_root_from_link * off_b;
        let delta = (pb.position_root_m + rb) - (pa.position_root_m + ra);
        let va = pa.linear_vel_root_m_s + pa.angular_vel_root_rad_s.cross(ra);
        let vb = pb.linear_vel_root_m_s + pb.angular_vel_root_rad_s.cross(rb);
        let f_on_a_root = match *self {
            // Spring pulls A toward B.
            Self::PointWeld {
                linear_stiffness_n_m,
                linear_damping_n_s_m,
                ..
            } => delta * linear_stiffness_n_m + (vb - va) * linear_damping_n_s_m,
            Self::UnilateralDistance {
                max_length_m,
                stiffness_n_m,
                damping_n_s_m,
                ..
            } => {
                let dist = delta.length();
                if dist <= max_length_m || dist == 0.0 {
                    return None;
                }
                let dir = delta / dist;
                let v_rel = (vb - va).dot(dir);
                dir * ((dist - max_length_m) * stiffness_n_m + v_rel * damping_n_s_m).max(0.0)
            }
            Self::JointRatio { .. } => return None,
        };
        Some([
            WrenchAtSite {
                site: pa.id,
                point_local_m: off_a,
                force_local_n: pa.rotation_root_from_link.conjugate() * f_on_a_root,
                torque_local_nm: DVec3::ZERO,
            },
            WrenchAtSite {
                site: pb.id,
                point_local_m: off_b,
                force_local_n: pb.rotation_root_from_link.conjugate() * (-f_on_a_root),
                torque_local_nm: DVec3::ZERO,
            },
        ])
    }
}

impl LoopConstraint {
    /// Balanced joint loads of a [`LoopConstraint::JointRatio`] at the
    /// state `kinematics` describes; `None` for a point constraint or a
    /// detached joint.
    pub fn joint_loads(
        &self,
        kinematics: &Kinematics<'_>,
    ) -> Result<Option<[JointLoadInput; 2]>, MultibodyError> {
        let Self::JointRatio {
            joint_a, joint_b, ..
        } = *self
        else {
            return Ok(None);
        };
        if !kinematics.is_attached(joint_a) || !kinematics.is_attached(joint_b) {
            return Ok(None);
        }
        check_1dof(joint_a, kinematics.joint_kind(joint_a)?)?;
        check_1dof(joint_b, kinematics.joint_kind(joint_b)?)?;
        let (qa, qb) = (kinematics.joint(joint_a)?, kinematics.joint(joint_b)?);
        let coords = (qa.q_scalar, qa.q_vel_scalar, qb.q_scalar, qb.q_vel_scalar);
        Ok(Some(self.ratio_loads(coords)))
    }

    /// Equal-and-opposite wrench pair of a point constraint at the state
    /// `kinematics` describes; `None` for a joint constraint, a slack
    /// unilateral one or a detached site.
    pub fn wrench_pair(
        &self,
        kinematics: &Kinematics<'_>,
    ) -> Result<Option<[WrenchAtSite; 2]>, MultibodyError> {
        let (Self::PointWeld { site_a, site_b, .. }
        | Self::UnilateralDistance { site_a, site_b, .. }) = *self
        else {
            return Ok(None);
        };
        if !kinematics.is_attached(site_a) || !kinematics.is_attached(site_b) {
            return Ok(None);
        }
        let pa = kinematics.link_pose_in_root(site_a)?;
        let pb = kinematics.link_pose_in_root(site_b)?;
        Ok(self.point_pair(&pa, &pb))
    }
}

/// A loop constraint is a stateless force element: its loads depend on the
/// evaluated state alone.
impl<I: ?Sized> ForceElement<I> for LoopConstraint {
    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        for load in self.joint_loads(ctx.kinematics)?.iter().flatten() {
            loads.add_joint_load(load)?;
        }
        for wrench in self.wrench_pair(ctx.kinematics)?.iter().flatten() {
            loads.add_wrench(wrench)?;
        }
        Ok(())
    }
}
