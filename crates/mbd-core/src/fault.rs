use crate::MbdError;

/// Unified fault injection and failure mode for engines, valves, sensors,
/// hinges, latches, and electrical channels.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum FaultState {
    /// Hardware operates as designed.
    #[default]
    Nominal,
    /// Valve or thruster fails open/firing at `fraction` (`0.0..=1.0`, or > 1 for overthrust).
    StuckOn {
        /// Active fractional level of full capacity.
        fraction: f64,
    },
    /// Valve, thruster, or channel fails closed / unresponsive.
    StuckOff,
    /// Degraded output scaled by `efficiency` in `0.0..=1.0`.
    Degraded {
        /// Multiplicative efficiency factor.
        efficiency: f64,
    },
    /// Propellant, pressurant, or coolant leak rate, kg/s.
    Leak {
        /// Mass leak rate, kg/s (nonnegative).
        rate_kg_s: f64,
    },
    /// Mechanism or joint mechanically frozen at its current position.
    Jammed,
    /// Electrical short circuit forcing command/channel active (e.g., Gemini 8 OAMS #8).
    ShortCircuit,
    /// Additive transducer bias in native physical units.
    SensorBias {
        /// Bias added to ideal measurement.
        offset: f64,
    },
}

impl FaultState {
    /// Whether this component has no active fault.
    pub const fn is_nominal(&self) -> bool {
        matches!(self, Self::Nominal)
    }

    /// Validates numeric parameters stored inside the fault variant.
    pub fn validate(&self) -> Result<(), MbdError> {
        match *self {
            Self::Nominal | Self::StuckOff | Self::Jammed | Self::ShortCircuit => Ok(()),
            Self::StuckOn { fraction } => check_nonneg("fault.fraction", fraction),
            Self::Degraded { efficiency } => {
                if efficiency.is_finite() && (0.0..=1.0).contains(&efficiency) {
                    Ok(())
                } else {
                    Err(MbdError::InvalidFlow {
                        field: "fault.efficiency",
                        value: efficiency,
                    })
                }
            }
            Self::Leak { rate_kg_s } => check_nonneg("fault.rate_kg_s", rate_kg_s),
            Self::SensorBias { offset } => {
                if offset.is_finite() {
                    Ok(())
                } else {
                    Err(MbdError::InvalidFlow {
                        field: "fault.offset",
                        value: offset,
                    })
                }
            }
        }
    }
}

fn check_nonneg(field: &'static str, value: f64) -> Result<(), MbdError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(MbdError::InvalidFlow { field, value })
    }
}
