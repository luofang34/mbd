use super::dial::{DialDrive, DialReading};
use crate::isa::pressure_lapse_pa_m;
use crate::units::{DialUnit, Quantity};
use crate::{AirdataError, DialScale, NeedleConfig, PneumaticLag};

#[cfg(test)]
mod tests;

/// How a vertical speed indicator is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerticalSpeedConfig {
    /// The dial's rate unit, such as feet per minute.
    pub unit: DialUnit,
    /// The dial face.
    pub scale: DialScale,
    /// Time constant of the case filling through the calibrated leak, s.
    pub leak_tau_s: f64,
    /// The needle, driven in dial angle (rad).
    pub needle: NeedleConfig,
}

/// A rate-of-climb indicator. A capsule is fed directly from the static
/// line while its case is fed through a capillary, so the case pressure
/// trails static pressure by the leak's time constant `τ`. In a steady climb
/// the capsule's differential settles at `τ · dp/dt`, which the linkage reads
/// as `dh/dt` through the standard atmosphere's `dp/dh`.
///
/// Like the real instrument it needs several time constants to settle after
/// a change of rate, and a sudden change of static pressure that is not a
/// climb, such as switching to the alternate static source, swings the
/// needle briefly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerticalSpeedIndicator {
    case: PneumaticLag,
    drive: DialDrive,
}

impl VerticalSpeedIndicator {
    /// An indicator settled in level flight at `static_pa`.
    pub fn new(config: VerticalSpeedConfig, static_pa: f64) -> Result<Self, AirdataError> {
        crate::error::in_range("calibrated leak tau", config.leak_tau_s, 1e-3, 1e3)?;
        let mut indicator = Self {
            case: PneumaticLag::new(config.leak_tau_s, static_pa),
            drive: DialDrive::new(
                "vertical speed indicator",
                Quantity::Speed,
                config.unit,
                config.scale,
                config.needle,
            )?,
        };
        indicator.reset(static_pa);
        Ok(indicator)
    }

    /// The dial face.
    pub fn scale(&self) -> &DialScale {
        self.drive.scale()
    }

    /// The case pressure behind the leak, Pa.
    pub fn case_pa(&self) -> f64 {
        self.case.value()
    }

    /// The climb rate the mechanism reads, in dial units, from capsule
    /// pressure `static_pa` and case pressure `case_pa`.
    pub fn mechanism_reading(&self, static_pa: f64, case_pa: f64) -> f64 {
        let gradient = pressure_lapse_pa_m(static_pa);
        if gradient.is_nan() || gradient >= 0.0 {
            return 0.0;
        }
        let climb_m_s = (static_pa - case_pa) / (self.case.tau_s() * gradient);
        self.drive.unit().from_si(climb_m_s)
    }

    /// Settles case and needle in level flight at `static_pa`.
    pub fn reset(&mut self, static_pa: f64) {
        self.case.reset(static_pa);
        self.drive.reset(0.0);
    }

    /// Advances `dt_s` with `static_pa` at the static connection.
    pub fn step(&mut self, dt_s: f64, static_pa: f64) -> DialReading {
        let case_pa = self.case.step(dt_s, static_pa);
        let reading = self.mechanism_reading(static_pa, case_pa);
        self.drive.step(dt_s, reading)
    }
}
