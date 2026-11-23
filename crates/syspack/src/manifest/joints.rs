use glam::DVec3;
use mbd_core::{JointActuatorSpec, JointSpec, JointState};
use serde::{Deserialize, Serialize};

use crate::SyspackError;

/// Serialized kinematic joint specification attached to a [`super::ComponentRecord`].
///
/// Defaults to [`JointRecord::Fixed`] when omitted in `schema_version = 1` or `2`
/// manifests, keeping full backwards compatibility with rigid v1 packages.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum JointRecord {
    /// Zero-DoF rigidly fastened mounting.
    #[default]
    Fixed,
    /// Single-axis hinge, sweep wing pivot, SLA petal hinge, or manipulator joint.
    Revolute {
        /// Unit rotation axis in the component's placed frame.
        axis_local: [f64; 3],
        /// Lower mechanical stop, rad.
        min_rad: f64,
        /// Upper mechanical stop, rad.
        max_rad: f64,
        /// Maximum actuator torque, N·m.
        max_torque_nm: f64,
        /// Passive viscous damping coefficient, N·m·s/rad.
        damping_nm_s_rad: f64,
        /// Initial joint angle, rad.
        #[serde(default)]
        initial_angle_rad: f64,
    },
    /// Single-axis linear rail, seat ejection catapult, or telescoping strut.
    Prismatic {
        /// Unit translation axis in the component's placed frame.
        axis_local: [f64; 3],
        /// Lower travel stop, m.
        min_m: f64,
        /// Upper travel stop, m.
        max_m: f64,
        /// Maximum linear actuator force, N.
        max_force_n: f64,
        /// Passive viscous damping coefficient, N·s/m.
        damping_n_s_m: f64,
        /// Initial linear displacement, m.
        #[serde(default)]
        initial_position_m: f64,
    },
    /// Three-axis spherical joint / gimbal bounded by `max_cone_rad`.
    Spherical {
        /// Half-angle of allowed deflection cone about identity, rad.
        max_cone_rad: f64,
        /// Viscous rotational damping, N·m·s/rad.
        damping_nm_s_rad: f64,
    },
    /// Six-DoF compliant coupler, shock mount, or docking capture ring.
    Compliant6Dof {
        /// Translational stiffness along X/Y/Z, N/m.
        linear_stiffness_n_m: [f64; 3],
        /// Rotational stiffness about X/Y/Z, N·m/rad.
        angular_stiffness_nm_rad: [f64; 3],
        /// Translational viscous damping along X/Y/Z, N·s/m.
        linear_damping_n_s_m: [f64; 3],
        /// Rotational viscous damping about X/Y/Z, N·m·s/rad.
        angular_damping_nm_s_rad: [f64; 3],
    },
}

impl JointRecord {
    /// Returns `true` when this joint is rigidly fixed (used to skip serialization when default).
    pub const fn is_fixed(&self) -> bool {
        matches!(self, Self::Fixed)
    }

    /// Converts and validates this serialized joint into `(mbd_core::JointSpec, mbd_core::JointState)`.
    pub fn to_core_joint(
        self,
        component_name: &str,
    ) -> Result<(JointSpec, JointState), SyspackError> {
        let (spec, state) = match self {
            Self::Fixed => (JointSpec::FIXED, JointState::FIXED),
            Self::Revolute {
                axis_local,
                min_rad,
                max_rad,
                max_torque_nm,
                damping_nm_s_rad,
                initial_angle_rad,
            } => (
                JointSpec::revolute(DVec3::from_array(axis_local), min_rad, max_rad)
                    .with_actuator(simple_actuator(max_torque_nm, 100.0, damping_nm_s_rad)),
                JointState::revolute(initial_angle_rad, 0.0),
            ),
            Self::Prismatic {
                axis_local,
                min_m,
                max_m,
                max_force_n,
                damping_n_s_m,
                initial_position_m,
            } => (
                JointSpec::prismatic(DVec3::from_array(axis_local), min_m, max_m)
                    .with_actuator(simple_actuator(max_force_n, 200.0, damping_n_s_m)),
                JointState::prismatic(initial_position_m, 0.0),
            ),
            Self::Spherical {
                max_cone_rad,
                damping_nm_s_rad,
            } => (
                JointSpec::spherical(max_cone_rad).with_actuator(simple_actuator(
                    0.0,
                    100.0,
                    damping_nm_s_rad,
                )),
                JointState::FIXED,
            ),
            Self::Compliant6Dof {
                linear_stiffness_n_m,
                angular_stiffness_nm_rad,
                linear_damping_n_s_m,
                angular_damping_nm_s_rad,
            } => (
                JointSpec::compliant_6dof(
                    DVec3::from_array(linear_stiffness_n_m),
                    DVec3::from_array(linear_damping_n_s_m),
                    DVec3::from_array(angular_stiffness_nm_rad),
                    DVec3::from_array(angular_damping_nm_s_rad),
                ),
                JointState::FIXED,
            ),
        };
        spec.validate(component_name)
            .map_err(SyspackError::InvalidTree)?;
        state
            .validate(component_name, &spec.kind)
            .map_err(SyspackError::InvalidTree)?;
        Ok((spec, state))
    }
}

fn simple_actuator(max_effort: f64, max_velocity: f64, damping: f64) -> JointActuatorSpec {
    JointActuatorSpec {
        max_effort,
        max_velocity,
        damping,
        stiffness: 0.0,
        coulomb_friction: 0.0,
        backdrivable: true,
    }
}
