use alloc::vec::Vec;

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, WrenchAtSite};

use crate::spatial::SpatialMotion;

mod articulated;
mod chain;
mod integrator;
mod solver;

#[cfg(test)]
mod tests;

pub use integrator::{step_rk4, step_rk4_with_forces, step_rk4_with_loads};
pub use solver::MultiBodyDynamics;

/// Full 6-DoF rigid pose and body-frame spatial velocity of a floating base
/// (vehicle root component) in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloatingBaseState {
    /// Root origin in inertial/world coordinates, m.
    pub position_world_m: DVec3,
    /// Root-to-world orientation quaternion.
    pub rotation_world_from_root: DQuat,
    /// Linear velocity of root origin expressed in root axes, m/s.
    pub linear_vel_root_m_s: DVec3,
    /// Angular velocity of root frame expressed in root axes, rad/s.
    pub angular_vel_root_rad_s: DVec3,
}

impl FloatingBaseState {
    /// At-rest identity pose at the world origin.
    pub const AT_REST: Self = Self {
        position_world_m: DVec3::ZERO,
        rotation_world_from_root: DQuat::IDENTITY,
        linear_vel_root_m_s: DVec3::ZERO,
        angular_vel_root_rad_s: DVec3::ZERO,
    };

    /// Spatial twist in root axes `(angular_vel_root_rad_s, linear_vel_root_m_s)`.
    pub const fn spatial_velocity_root(&self) -> SpatialMotion {
        SpatialMotion::new(self.angular_vel_root_rad_s, self.linear_vel_root_m_s)
    }

    /// Inertial velocity of the root origin expressed in world axes, m/s.
    pub fn linear_vel_world_m_s(&self) -> DVec3 {
        self.rotation_world_from_root * self.linear_vel_root_m_s
    }

    /// Inertial angular velocity expressed in world axes, rad/s.
    pub fn angular_vel_world_rad_s(&self) -> DVec3 {
        self.rotation_world_from_root * self.angular_vel_root_rad_s
    }
}

/// Actuator command (motor, servo, catapult) across a non-fixed component joint.
///
/// All commands addressed to one joint are summed, and the scalar sum is
/// limited to the joint actuator's `max_effort`. Efforts that are not the
/// actuator's own command belong in [`JointLoadInput`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointDriveInput {
    /// Component owning the target joint.
    pub component: ComponentId,
    /// Generalized scalar torque (N·m) for `Revolute` or force (N) for `Prismatic`.
    pub effort_scalar: f64,
    /// Pure torque in placed frame (N·m) for `Spherical` or `Compliant6Dof`.
    pub torque_3d_nm: DVec3,
    /// Pure force in placed frame (N) for `Compliant6Dof`.
    pub force_3d_n: DVec3,
}

impl JointDriveInput {
    /// 1-DoF scalar drive effort on `component`.
    pub const fn scalar(component: ComponentId, effort_scalar: f64) -> Self {
        Self {
            component,
            effort_scalar,
            torque_3d_nm: DVec3::ZERO,
            force_3d_n: DVec3::ZERO,
        }
    }
}

/// Joint-space load that is not an actuator command: a closed-loop
/// constraint, a mechanism spring or damper, or a brake. Loads on one joint
/// are summed with each other and with the limited actuator command, and are
/// never limited themselves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLoadInput {
    /// Component owning the target joint.
    pub component: ComponentId,
    /// Generalized scalar torque (N·m) for `Revolute` or force (N) for `Prismatic`.
    pub effort_scalar: f64,
    /// Pure torque in placed frame (N·m) for `Spherical` or `Compliant6Dof`.
    pub torque_3d_nm: DVec3,
    /// Pure force in placed frame (N) for `Compliant6Dof`.
    pub force_3d_n: DVec3,
}

impl JointLoadInput {
    /// 1-DoF scalar load on `component`.
    pub const fn scalar(component: ComponentId, effort_scalar: f64) -> Self {
        Self {
            component,
            effort_scalar,
            torque_3d_nm: DVec3::ZERO,
            force_3d_n: DVec3::ZERO,
        }
    }
}

/// Everything a stage callback of [`step_rk4_with_loads`] supplies.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct StageForces {
    /// Uniform gravity in world axes, m/s².
    pub gravity_world_m_s2: DVec3,
    /// Actuator commands, limited per joint by the actuator.
    pub joint_drives: Vec<JointDriveInput>,
    /// Constraint and mechanism joint loads, never limited.
    pub joint_loads: Vec<JointLoadInput>,
    /// External wrenches at component sites.
    pub external_wrenches: Vec<WrenchAtSite>,
}

/// Joint-space acceleration resolved by [`MultiBodyDynamics::forward_dynamics`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointAccel {
    /// Component owning this joint.
    pub component: ComponentId,
    /// Generalized scalar acceleration (rad/s² or m/s²) for `Revolute` / `Prismatic`.
    pub q_accel_scalar: f64,
    /// Relative linear acceleration in placed frame (m/s²) for `Compliant6Dof`.
    pub linear_accel_m_s2: DVec3,
    /// Relative angular acceleration in placed frame (rad/s²) for `Spherical` / `Compliant6Dof`.
    pub angular_accel_rad_s2: DVec3,
}

/// Complete forward-dynamics output for floating base and all articulated joints.
#[derive(Debug, Clone, PartialEq)]
pub struct MultiBodyAccel {
    /// Time derivative of `(angular_vel_root_rad_s, linear_vel_root_m_s)` in rotating root axes.
    pub base_spatial_accel_root: SpatialMotion,
    /// Joint accelerations in component order for every attached non-fixed joint.
    pub joint_accels: Vec<JointAccel>,
    /// True when evaluated via the zero-DoF `RigidFastPath`.
    pub used_rigid_fast_path: bool,
}

/// Total inertial linear and angular momentum across every attached link and fluid contribution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SystemMomentum {
    /// Total attached mass, kg.
    pub total_mass_kg: f64,
    /// Instantaneous world centre of mass, m.
    pub com_world_m: DVec3,
    /// Total world linear momentum, kg·m/s.
    pub linear_world_kg_m_s: DVec3,
    /// Total world angular momentum about `com_world_m`, N·m·s.
    pub angular_about_com_world_nms: DVec3,
}
