use glam::{DQuat, DVec3};

use crate::MbdError;

mod stop;
#[cfg(test)]
mod tests;

pub use stop::JointStopSpec;

/// Relative tolerance around unit length for joint axes and quaternions.
const UNIT_TOLERANCE: f64 = 1e-6;

/// Kinematic joint connecting a component to its parent in [`crate::ComponentTree`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum JointKind {
    /// Rigidly bolted frame (0 DoF).
    #[default]
    Fixed,
    /// Single-axis hinge/gimbal rotation (1 DoF).
    Revolute {
        /// Unit rotation axis in the component's placed frame.
        axis: DVec3,
        /// Lower mechanical stop, rad.
        min_rad: f64,
        /// Upper mechanical stop, rad.
        max_rad: f64,
    },
    /// Single-axis linear slide rail, winch, or linear actuator (1 DoF).
    Prismatic {
        /// Unit translation axis in the component's placed frame.
        axis: DVec3,
        /// Lower travel stop, m.
        min_m: f64,
        /// Upper travel stop, m.
        max_m: f64,
    },
    /// Three-axis ball/gimbal joint (3 DoF, SO(3) parameterization).
    Spherical {
        /// Half-angle of the permissible deflection cone about identity, rad.
        cone_half_angle_rad: f64,
    },
    /// Six-DoF compliant joint used for docking capture latches, compliant mounts, or soft couplers.
    Compliant6Dof {
        /// Translational spring stiffness along placed X/Y/Z, N/m.
        linear_stiffness_n_m: DVec3,
        /// Translational viscous damping along placed X/Y/Z, N·s/m.
        linear_damping_n_s_m: DVec3,
        /// Rotational spring stiffness about placed X/Y/Z, N·m/rad.
        angular_stiffness_nm_rad: DVec3,
        /// Rotational viscous damping about placed X/Y/Z, N·m·s/rad.
        angular_damping_nm_s_rad: DVec3,
    },
}

/// Optional servo/motor/brake parameters driving a non-fixed [`JointSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointActuatorSpec {
    /// Peak force (N) or torque (N·m).
    pub max_effort: f64,
    /// Peak linear speed (m/s) or angular speed (rad/s).
    pub max_velocity: f64,
    /// Passive viscous damping in joint space.
    pub damping: f64,
    /// Passive spring stiffness toward neutral position.
    pub stiffness: f64,
    /// Static/Coulomb friction force or torque.
    pub coulomb_friction: f64,
    /// Whether external wrenches can back-drive the actuator when idle.
    pub backdrivable: bool,
}

/// Complete kinematic and actuator specification of one component joint.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JointSpec {
    /// Kinematic joint degree-of-freedom type.
    pub kind: JointKind,
    /// Optional motor/servo characteristics.
    pub actuator: Option<JointActuatorSpec>,
    /// Travel-stop response at the joint's limits.
    pub stop: JointStopSpec,
}

/// Runtime generalized position and velocity of one component joint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointState {
    /// Generalized scalar coordinate: angle (rad) for [`JointKind::Revolute`],
    /// displacement (m) for [`JointKind::Prismatic`].
    pub q_scalar: f64,
    /// Generalized scalar rate: rad/s or m/s.
    pub q_vel_scalar: f64,
    /// Translational deflection for [`JointKind::Compliant6Dof`], metres.
    pub translation_m: DVec3,
    /// Translational velocity for [`JointKind::Compliant6Dof`], m/s.
    pub linear_vel_m_s: DVec3,
    /// Rotational deflection for [`JointKind::Spherical`] or [`JointKind::Compliant6Dof`].
    pub rotation: DQuat,
    /// Angular velocity in joint frame, rad/s.
    pub angular_vel_rad_s: DVec3,
}

impl Default for JointState {
    fn default() -> Self {
        Self::FIXED
    }
}

impl JointSpec {
    /// Zero-DoF rigid attachment.
    pub const FIXED: Self = Self {
        kind: JointKind::Fixed,
        actuator: None,
        stop: JointStopSpec::DEFAULT,
    };

    /// Creates a 1-DoF revolute joint with normalized axis checks.
    pub const fn revolute(axis: DVec3, min_rad: f64, max_rad: f64) -> Self {
        Self {
            kind: JointKind::Revolute {
                axis,
                min_rad,
                max_rad,
            },
            actuator: None,
            stop: JointStopSpec::DEFAULT,
        }
    }

    /// Creates a 1-DoF prismatic rail joint.
    pub const fn prismatic(axis: DVec3, min_m: f64, max_m: f64) -> Self {
        Self {
            kind: JointKind::Prismatic { axis, min_m, max_m },
            actuator: None,
            stop: JointStopSpec::DEFAULT,
        }
    }

    /// Creates a 3-DoF spherical joint bounded by `cone_half_angle_rad`.
    pub const fn spherical(cone_half_angle_rad: f64) -> Self {
        Self {
            kind: JointKind::Spherical {
                cone_half_angle_rad,
            },
            actuator: None,
            stop: JointStopSpec::DEFAULT,
        }
    }

    /// Creates a 6-DoF compliant latch/mount joint.
    pub const fn compliant_6dof(
        linear_stiffness_n_m: DVec3,
        linear_damping_n_s_m: DVec3,
        angular_stiffness_nm_rad: DVec3,
        angular_damping_nm_s_rad: DVec3,
    ) -> Self {
        Self {
            kind: JointKind::Compliant6Dof {
                linear_stiffness_n_m,
                linear_damping_n_s_m,
                angular_stiffness_nm_rad,
                angular_damping_nm_s_rad,
            },
            actuator: None,
            stop: JointStopSpec::DEFAULT,
        }
    }

    /// Attaches an actuator specification to this joint.
    pub const fn with_actuator(mut self, actuator: JointActuatorSpec) -> Self {
        self.actuator = Some(actuator);
        self
    }

    /// Replaces the travel-stop response.
    pub const fn with_stop(mut self, stop: JointStopSpec) -> Self {
        self.stop = stop;
        self
    }

    /// Kinematic degrees of freedom (`0`, `1`, `3`, or `6`).
    pub const fn dof(&self) -> usize {
        match self.kind {
            JointKind::Fixed => 0,
            JointKind::Revolute { .. } | JointKind::Prismatic { .. } => 1,
            JointKind::Spherical { .. } => 3,
            JointKind::Compliant6Dof { .. } => 6,
        }
    }

    /// Validates axes, limits, and actuator parameters.
    pub fn validate(&self, component_name: &str) -> Result<(), MbdError> {
        self.kind.validate(component_name)?;
        stop::validate_stop(component_name, &self.stop)?;
        if let Some(actuator) = self.actuator {
            actuator.validate(component_name)?;
        }
        Ok(())
    }

    /// Relative `(translation, rotation)` from child frame into parent placed frame.
    pub fn local_transform(&self, state: &JointState) -> Result<(DVec3, DQuat), MbdError> {
        if !state.rotation.is_finite() || (state.rotation.length_squared() - 1.0).abs() > 1e-9 {
            return Err(MbdError::InvalidJoint {
                name: alloc::string::String::from("joint"),
                field: "rotation",
                detail: alloc::format!("{} is not a unit quaternion", state.rotation),
            });
        }
        let transform = match self.kind {
            JointKind::Fixed => (DVec3::ZERO, DQuat::IDENTITY),
            JointKind::Revolute { axis, .. } => {
                (DVec3::ZERO, DQuat::from_axis_angle(axis, state.q_scalar))
            }
            JointKind::Prismatic { axis, .. } => (axis * state.q_scalar, DQuat::IDENTITY),
            JointKind::Spherical { .. } => (DVec3::ZERO, state.rotation),
            JointKind::Compliant6Dof { .. } => (state.translation_m, state.rotation),
        };
        Ok(transform)
    }

    /// Relative `(linear_velocity, angular_velocity)` expressed in the placed frame.
    pub fn local_velocity(&self, state: &JointState) -> (DVec3, DVec3) {
        match self.kind {
            JointKind::Fixed => (DVec3::ZERO, DVec3::ZERO),
            JointKind::Revolute { axis, .. } => (DVec3::ZERO, axis * state.q_vel_scalar),
            JointKind::Prismatic { axis, .. } => (axis * state.q_vel_scalar, DVec3::ZERO),
            JointKind::Spherical { .. } => (DVec3::ZERO, state.angular_vel_rad_s),
            JointKind::Compliant6Dof { .. } => (state.linear_vel_m_s, state.angular_vel_rad_s),
        }
    }
}

impl JointKind {
    fn validate(&self, component_name: &str) -> Result<(), MbdError> {
        match *self {
            Self::Fixed => Ok(()),
            Self::Revolute {
                axis,
                min_rad,
                max_rad,
            } => {
                check_unit_axis(component_name, "joint.axis", axis)?;
                check_ordered_bounds(component_name, "joint.limits_rad", min_rad, max_rad)
            }
            Self::Prismatic { axis, min_m, max_m } => {
                check_unit_axis(component_name, "joint.axis", axis)?;
                check_ordered_bounds(component_name, "joint.limits_m", min_m, max_m)
            }
            Self::Spherical {
                cone_half_angle_rad,
            } => {
                if cone_half_angle_rad.is_finite()
                    && (0.0..=core::f64::consts::PI).contains(&cone_half_angle_rad)
                {
                    Ok(())
                } else {
                    Err(invalid_joint(
                        component_name,
                        "joint.cone_half_angle_rad",
                        alloc::format!("{cone_half_angle_rad} must be in 0..=π"),
                    ))
                }
            }
            Self::Compliant6Dof {
                linear_stiffness_n_m,
                linear_damping_n_s_m,
                angular_stiffness_nm_rad,
                angular_damping_nm_s_rad,
            } => {
                check_nonneg_vec(
                    component_name,
                    "joint.linear_stiffness_n_m",
                    linear_stiffness_n_m,
                )?;
                check_nonneg_vec(
                    component_name,
                    "joint.linear_damping_n_s_m",
                    linear_damping_n_s_m,
                )?;
                check_nonneg_vec(
                    component_name,
                    "joint.angular_stiffness_nm_rad",
                    angular_stiffness_nm_rad,
                )?;
                check_nonneg_vec(
                    component_name,
                    "joint.angular_damping_nm_s_rad",
                    angular_damping_nm_s_rad,
                )
            }
        }
    }
}

impl JointActuatorSpec {
    fn validate(&self, component_name: &str) -> Result<(), MbdError> {
        let checks = [
            ("actuator.max_effort", self.max_effort),
            ("actuator.max_velocity", self.max_velocity),
            ("actuator.damping", self.damping),
            ("actuator.stiffness", self.stiffness),
            ("actuator.coulomb_friction", self.coulomb_friction),
        ];
        for (field, value) in checks {
            if !value.is_finite() || value < 0.0 {
                return Err(invalid_joint(
                    component_name,
                    field,
                    alloc::format!("{value} must be finite and nonnegative"),
                ));
            }
        }
        Ok(())
    }
}

impl JointState {
    /// Zero state at neutral joint pose.
    pub const FIXED: Self = Self {
        q_scalar: 0.0,
        q_vel_scalar: 0.0,
        translation_m: DVec3::ZERO,
        linear_vel_m_s: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        angular_vel_rad_s: DVec3::ZERO,
    };

    /// Revolute state at `angle_rad` and `rate_rad_s`.
    pub const fn revolute(angle_rad: f64, rate_rad_s: f64) -> Self {
        Self {
            q_scalar: angle_rad,
            q_vel_scalar: rate_rad_s,
            ..Self::FIXED
        }
    }

    /// Prismatic state at displacement `displacement_m` and `velocity_m_s`.
    pub const fn prismatic(displacement_m: f64, velocity_m_s: f64) -> Self {
        Self {
            q_scalar: displacement_m,
            q_vel_scalar: velocity_m_s,
            ..Self::FIXED
        }
    }

    /// Spherical joint state.
    pub const fn spherical(rotation: DQuat, angular_vel_rad_s: DVec3) -> Self {
        Self {
            rotation,
            angular_vel_rad_s,
            ..Self::FIXED
        }
    }

    /// 6-DoF compliant joint state.
    pub const fn compliant_6dof(
        translation_m: DVec3,
        linear_vel_m_s: DVec3,
        rotation: DQuat,
        angular_vel_rad_s: DVec3,
    ) -> Self {
        Self {
            translation_m,
            linear_vel_m_s,
            rotation,
            angular_vel_rad_s,
            ..Self::FIXED
        }
    }

    /// Validates finiteness and the unit rotation quaternion.
    pub fn validate(&self, component_name: &str) -> Result<(), MbdError> {
        let scalars = [
            ("joint_state.q_scalar", self.q_scalar),
            ("joint_state.q_vel_scalar", self.q_vel_scalar),
        ];
        for (field, value) in scalars {
            if !value.is_finite() {
                return Err(invalid_joint(
                    component_name,
                    field,
                    alloc::format!("{value} is not finite"),
                ));
            }
        }
        let vecs = [
            ("joint_state.translation_m", self.translation_m),
            ("joint_state.linear_vel_m_s", self.linear_vel_m_s),
            ("joint_state.angular_vel_rad_s", self.angular_vel_rad_s),
        ];
        for (field, vec) in vecs {
            if !vec.is_finite() {
                return Err(invalid_joint(
                    component_name,
                    field,
                    alloc::format!("{vec} is not finite"),
                ));
            }
        }
        if !self.rotation.is_finite() || (self.rotation.length_squared() - 1.0).abs() > 1e-9 {
            return Err(invalid_joint(
                component_name,
                "joint_state.rotation",
                alloc::format!("{} is not a unit quaternion", self.rotation),
            ));
        }
        Ok(())
    }
}

fn invalid_joint(name: &str, field: &'static str, detail: alloc::string::String) -> MbdError {
    MbdError::InvalidJoint {
        name: alloc::string::String::from(name),
        field,
        detail,
    }
}

fn check_unit_axis(name: &str, field: &'static str, axis: DVec3) -> Result<(), MbdError> {
    if axis.is_finite() && (axis.length_squared() - 1.0).abs() <= UNIT_TOLERANCE {
        return Ok(());
    }
    Err(invalid_joint(
        name,
        field,
        alloc::format!("{axis} is not a unit vector"),
    ))
}

fn check_ordered_bounds(
    name: &str,
    field: &'static str,
    min: f64,
    max: f64,
) -> Result<(), MbdError> {
    if min.is_finite() && max.is_finite() && min <= max {
        return Ok(());
    }
    Err(invalid_joint(
        name,
        field,
        alloc::format!("invalid bounds [{min}, {max}]"),
    ))
}

fn check_nonneg_vec(name: &str, field: &'static str, vec: DVec3) -> Result<(), MbdError> {
    if vec.is_finite() && vec.min_element() >= 0.0 {
        return Ok(());
    }
    Err(invalid_joint(
        name,
        field,
        alloc::format!("{vec} must be finite with nonnegative entries"),
    ))
}
