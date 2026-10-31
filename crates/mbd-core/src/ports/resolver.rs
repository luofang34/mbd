use alloc::format;

use super::{ElectricalPinState, EngineeringValue, Transducer};
use crate::{MbdError, math};

impl ElectricalPinState {
    /// Windings of a resolver with shaft at `angle_rad`.
    pub fn from_angle_rad(excitation_v: f64, angle_rad: f64) -> Self {
        let (sin, cos) = math::sin_cos(angle_rad);
        Self::ResolverSinCos {
            excitation_v,
            sin_v: excitation_v * sin,
            cos_v: excitation_v * cos,
        }
    }

    /// Shaft angle in `(-π, π]`, measured ratiometrically: dividing by the
    /// excitation keeps the quadrant right on a carrier half-cycle where the
    /// excitation is negative. An error reports a resolver that has lost
    /// excitation or a winding-pair signal.
    pub fn decode_angle_rad(&self) -> Result<f64, MbdError> {
        let Self::ResolverSinCos {
            excitation_v,
            sin_v,
            cos_v,
        } = *self
        else {
            return Err(MbdError::WrongPortKind {
                expected: "ResolverSinCos",
            });
        };
        let sin = sin_v / excitation_v;
        let cos = cos_v / excitation_v;
        if !sin.is_finite() || !cos.is_finite() || (sin == 0.0 && cos == 0.0) {
            return Err(MbdError::ResolverSignalLost {
                detail: format!("excitation {excitation_v} V, sin {sin_v} V, cos {cos_v} V"),
            });
        }
        Ok(math::atan2(sin, cos))
    }
}

/// A shaft-angle sensor: a [`EngineeringValue::Scalar`] angle in radians at
/// the engineering tier, resolver windings at the pin tier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolverTransducer {
    /// Excitation voltage on the rotor.
    pub excitation_v: f64,
}

impl Transducer for ResolverTransducer {
    type Upper = EngineeringValue;
    type Lower = ElectricalPinState;

    fn encode(&self, upper: &EngineeringValue) -> Result<ElectricalPinState, MbdError> {
        let EngineeringValue::Scalar(angle_rad) = *upper else {
            return Err(MbdError::WrongPortKind { expected: "Scalar" });
        };
        Ok(ElectricalPinState::from_angle_rad(
            self.excitation_v,
            angle_rad,
        ))
    }

    fn decode(&self, lower: &ElectricalPinState) -> Result<EngineeringValue, MbdError> {
        lower.decode_angle_rad().map(EngineeringValue::Scalar)
    }
}
