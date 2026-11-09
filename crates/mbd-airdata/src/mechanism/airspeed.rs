use super::dial::{DialDrive, DialReading};
use crate::pitot::cas_from_impact_pressure_m_s;
use crate::units::{DialUnit, Quantity};
use crate::{AirdataError, DialScale, NeedleConfig, PiecewiseLinear};

#[cfg(test)]
mod tests;

/// How an airspeed indicator is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedConfig {
    /// The dial's speed unit.
    pub unit: DialUnit,
    /// The dial face.
    pub scale: DialScale,
    /// Instrument error in dial units against the reading of a perfect
    /// instrument, from the instrument's bench calibration; added to it.
    pub instrument_error: PiecewiseLinear,
    /// The needle, driven in dial angle (rad).
    pub needle: NeedleConfig,
}

/// A pitot-static airspeed indicator. A diaphragm capsule fed with total
/// pressure inside a case fed with static pressure deflects with their
/// difference, the impact pressure; the linkage is calibrated so that
/// deflection reads the sea-level CAS law. It therefore shows calibrated
/// airspeed when its pressures are exact, and indicated airspeed when they
/// carry the airframe's position error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedIndicator {
    instrument_error: PiecewiseLinear,
    drive: DialDrive,
}

impl AirspeedIndicator {
    /// An indicator at rest on its first graduation.
    pub fn new(config: AirspeedConfig) -> Result<Self, AirdataError> {
        Ok(Self {
            instrument_error: config.instrument_error,
            drive: DialDrive::new(
                "airspeed indicator",
                Quantity::Speed,
                config.unit,
                config.scale,
                config.needle,
            )?,
        })
    }

    /// The dial face.
    pub fn scale(&self) -> &DialScale {
        self.drive.scale()
    }

    /// The speed the mechanism reads, in dial units, from pitot (total) and
    /// static pressure at the instrument's connections, Pa. Reverse flow
    /// cannot drive the capsule below zero.
    pub fn mechanism_reading(&self, total_pa: f64, static_pa: f64) -> f64 {
        let impact_pa = (total_pa - static_pa).max(0.0);
        let perfect = self
            .drive
            .unit()
            .from_si(cas_from_impact_pressure_m_s(impact_pa));
        perfect + self.instrument_error.eval(perfect)
    }

    /// Places the needle at rest on the reading for these pressures.
    pub fn reset(&mut self, total_pa: f64, static_pa: f64) {
        self.drive
            .reset(self.mechanism_reading(total_pa, static_pa));
    }

    /// Advances `dt_s` with these pressures at the connections.
    pub fn step(&mut self, dt_s: f64, total_pa: f64, static_pa: f64) -> DialReading {
        let reading = self.mechanism_reading(total_pa, static_pa);
        self.drive.step(dt_s, reading)
    }
}
