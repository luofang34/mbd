//! Travel-stop dynamics shared by every bounded joint.

use glam::DVec3;

use crate::MbdError;

/// Penalty travel stop expressed as a second-order response rather than a
/// raw stiffness: the solver scales it by the articulated inertia the joint
/// drives, so an overrun decays the same way on a 10 g arm and a 10 t boom.
///
/// Beyond a limit the stop demands the joint acceleration
/// `ω² · overrun − 2 ζ ω · rate`, i.e. stiffness `I ω²` and damping `2 ζ ω I`
/// for effective inertia `I` about the joint's axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointStopSpec {
    /// Undamped natural frequency `ω` of the stop, rad/s.
    pub natural_frequency_rad_s: f64,
    /// Dimensionless damping ratio `ζ`.
    pub damping_ratio: f64,
}

impl JointStopSpec {
    /// Default stop: `ω = 45 rad/s` is fast against mechanism motion yet
    /// resolved by RK4 at a 10 ms step (`ω·dt ≈ 0.45`), and `ζ = 0.9` settles
    /// with a few percent of rebound.
    pub const DEFAULT: Self = Self {
        natural_frequency_rad_s: 45.0,
        damping_ratio: 0.9,
    };

    /// Joint acceleration demanded at `overrun` past a limit (signed toward
    /// the allowed range) while moving at `rate`.
    pub fn accel_demand(&self, overrun: f64, rate: f64) -> f64 {
        let w = self.natural_frequency_rad_s;
        w * w * overrun - 2.0 * self.damping_ratio * w * rate
    }

    /// Vector form of [`Self::accel_demand`] for a multi-axis joint.
    pub fn accel_demand_vec(&self, overrun: DVec3, rate: DVec3) -> DVec3 {
        let w = self.natural_frequency_rad_s;
        overrun * (w * w) - rate * (2.0 * self.damping_ratio * w)
    }

    pub(super) fn validate(&self) -> Result<(), (&'static str, f64)> {
        let w = self.natural_frequency_rad_s;
        if !w.is_finite() || w <= 0.0 {
            return Err(("stop.natural_frequency_rad_s", w));
        }
        let z = self.damping_ratio;
        if !z.is_finite() || z < 0.0 {
            return Err(("stop.damping_ratio", z));
        }
        Ok(())
    }
}

impl Default for JointStopSpec {
    fn default() -> Self {
        Self::DEFAULT
    }
}

pub(super) fn validate_stop(component_name: &str, stop: &JointStopSpec) -> Result<(), MbdError> {
    stop.validate()
        .map_err(|(field, value)| MbdError::InvalidJoint {
            name: alloc::string::String::from(component_name),
            field,
            detail: alloc::format!("{value} must be finite and positive (ratio: nonnegative)"),
        })
}
