//! Gauges whose sensing element follows its quantity with one lag: a
//! vapour-pressure or resistance thermometer's bulb, and a fuel-quantity
//! transmitter's float in its damped well.

use super::dial::{DialDrive, DialReading};
use crate::units::{DialUnit, Quantity};
use crate::{AirdataError, DialScale, NeedleConfig, PneumaticLag};

#[cfg(test)]
mod tests;

/// How a lagged sensing gauge is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensedGaugeConfig {
    /// The dial's unit.
    pub unit: DialUnit,
    /// The dial face.
    pub scale: DialScale,
    /// Time constant of the sensing element, s; zero for none.
    pub element_tau_s: f64,
    /// The needle, driven in dial angle (rad).
    pub needle: NeedleConfig,
}

/// One lagged element and its dial, for any quantity.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SensedGauge {
    element: PneumaticLag,
    drive: DialDrive,
}

impl SensedGauge {
    fn new(
        instrument: &'static str,
        quantity: Quantity,
        config: SensedGaugeConfig,
        si: f64,
    ) -> Result<Self, AirdataError> {
        crate::error::in_range(instrument, config.element_tau_s, 0.0, f64::MAX)?;
        let mut gauge = Self {
            element: PneumaticLag::new(config.element_tau_s, si),
            drive: DialDrive::new(
                instrument,
                quantity,
                config.unit,
                config.scale,
                config.needle,
            )?,
        };
        gauge.reset(si);
        Ok(gauge)
    }

    fn reset(&mut self, si: f64) {
        self.element.reset(si);
        self.drive.reset(self.drive.unit().from_si(si));
    }

    fn step(&mut self, dt_s: f64, si: f64) -> DialReading {
        let sensed = self.element.step(dt_s, si);
        self.drive.step(dt_s, self.drive.unit().from_si(sensed))
    }
}

/// A thermometer: oil, coolant or carburettor air temperature, read through
/// a bulb that warms with its own time constant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thermometer(SensedGauge);

impl Thermometer {
    /// A thermometer settled at `temperature_k`.
    pub fn new(config: SensedGaugeConfig, temperature_k: f64) -> Result<Self, AirdataError> {
        SensedGauge::new("thermometer", Quantity::Temperature, config, temperature_k).map(Self)
    }

    /// The dial face.
    pub fn scale(&self) -> &DialScale {
        self.0.drive.scale()
    }

    /// Settles bulb and needle at `temperature_k`.
    pub fn reset(&mut self, temperature_k: f64) {
        self.0.reset(temperature_k);
    }

    /// Advances `dt_s` with the bulb's surroundings at `temperature_k`.
    pub fn step(&mut self, dt_s: f64, temperature_k: f64) -> DialReading {
        self.0.step(dt_s, temperature_k)
    }
}

/// A fuel-quantity gauge: a tank's float transmitter, damped against
/// sloshing, driving a dial in volume.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FuelQuantityGauge(SensedGauge);

impl FuelQuantityGauge {
    /// A gauge settled at `volume_m3`.
    pub fn new(config: SensedGaugeConfig, volume_m3: f64) -> Result<Self, AirdataError> {
        SensedGauge::new("fuel quantity gauge", Quantity::Volume, config, volume_m3).map(Self)
    }

    /// The dial face.
    pub fn scale(&self) -> &DialScale {
        self.0.drive.scale()
    }

    /// Settles float and needle at `volume_m3`.
    pub fn reset(&mut self, volume_m3: f64) {
        self.0.reset(volume_m3);
    }

    /// Advances `dt_s` with `volume_m3` of fuel in the tank.
    pub fn step(&mut self, dt_s: f64, volume_m3: f64) -> DialReading {
        self.0.step(dt_s, volume_m3)
    }
}
