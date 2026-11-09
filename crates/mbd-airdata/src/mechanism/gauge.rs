use super::dial::{DialDrive, DialReading};
use crate::units::{DialUnit, Quantity};
use crate::{AirdataError, DialScale, NeedleConfig, PneumaticLag};

#[cfg(test)]
mod tests;

/// How a pressure gauge is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureGaugeConfig {
    /// The dial's pressure unit.
    pub unit: DialUnit,
    /// The dial face.
    pub scale: DialScale,
    /// Time constant of the sensing line and element, s; zero for none.
    pub line_tau_s: f64,
    /// The needle, driven in dial angle (rad).
    pub needle: NeedleConfig,
}

/// A Bourdon-tube or capsule pressure gauge. It reads whatever pressure its
/// line delivers: absolute pressure for a manifold pressure gauge, pressure
/// above ambient for an oil or fuel pressure gauge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureGauge {
    line: PneumaticLag,
    drive: DialDrive,
}

impl PressureGauge {
    /// A gauge settled at `pressure_pa`.
    pub fn new(config: PressureGaugeConfig, pressure_pa: f64) -> Result<Self, AirdataError> {
        crate::error::in_range("pressure gauge line tau", config.line_tau_s, 0.0, f64::MAX)?;
        let mut gauge = Self {
            line: PneumaticLag::new(config.line_tau_s, pressure_pa),
            drive: DialDrive::new(
                "pressure gauge",
                Quantity::Pressure,
                config.unit,
                config.scale,
                config.needle,
            )?,
        };
        gauge.reset(pressure_pa);
        Ok(gauge)
    }

    /// The dial face.
    pub fn scale(&self) -> &DialScale {
        self.drive.scale()
    }

    /// Settles line and needle at `pressure_pa`.
    pub fn reset(&mut self, pressure_pa: f64) {
        self.line.reset(pressure_pa);
        self.drive.reset(self.drive.unit().from_si(pressure_pa));
    }

    /// Advances `dt_s` with `pressure_pa` at the line's source.
    pub fn step(&mut self, dt_s: f64, pressure_pa: f64) -> DialReading {
        let sensed = self.line.step(dt_s, pressure_pa);
        self.drive.step(dt_s, self.drive.unit().from_si(sensed))
    }
}

/// How a tachometer is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TachometerConfig {
    /// The dial's unit; revolutions per minute.
    pub unit: DialUnit,
    /// The dial face.
    pub scale: DialScale,
    /// Time constant of the drag cup against its hairspring, s.
    pub drag_cup_tau_s: f64,
    /// The needle, driven in dial angle (rad).
    pub needle: NeedleConfig,
}

/// A magnetic-drag (eddy-current) tachometer: a magnet spun by the drive
/// cable drags an aluminium cup against a hairspring, so the cup's
/// deflection follows shaft speed with a first-order lag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tachometer {
    cup: PneumaticLag,
    drive: DialDrive,
}

impl Tachometer {
    /// A tachometer settled at `rpm`.
    pub fn new(config: TachometerConfig, rpm: f64) -> Result<Self, AirdataError> {
        crate::error::in_range(
            "tachometer drag cup tau",
            config.drag_cup_tau_s,
            0.0,
            f64::MAX,
        )?;
        let mut tachometer = Self {
            cup: PneumaticLag::new(config.drag_cup_tau_s, rpm),
            drive: DialDrive::new(
                "tachometer",
                Quantity::RotationRate,
                config.unit,
                config.scale,
                config.needle,
            )?,
        };
        tachometer.reset(rpm);
        Ok(tachometer)
    }

    /// The dial face.
    pub fn scale(&self) -> &DialScale {
        self.drive.scale()
    }

    /// Settles cup and needle at `rpm`.
    pub fn reset(&mut self, rpm: f64) {
        self.cup.reset(rpm);
        self.drive.reset(self.drive.unit().from_si(rpm));
    }

    /// Advances `dt_s` with the drive shaft at `rpm`.
    pub fn step(&mut self, dt_s: f64, rpm: f64) -> DialReading {
        let sensed = self.cup.step(dt_s, rpm);
        self.drive.step(dt_s, self.drive.unit().from_si(sensed))
    }
}
