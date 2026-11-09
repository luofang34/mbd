use core::f64::consts::TAU;

use crate::isa::pressure_altitude_m;
use crate::units::{DialUnit, Quantity};
use crate::{AirdataError, NeedleConfig, NeedleDynamics, PneumaticLag};

#[cfg(test)]
mod tests;

/// One inch of mercury at 0 °C, Pa.
const INCH_HG_PA: f64 = 3_386.389;

/// The range of a Kollsman subscale, Pa. The knob turns the subscale only
/// between its stops.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KollsmanWindow {
    /// Lowest setting, Pa.
    pub min_pa: f64,
    /// Highest setting, Pa.
    pub max_pa: f64,
}

impl KollsmanWindow {
    /// The 28.10–31.00 inHg window of US sensitive altimeters.
    pub const INCHES_OF_MERCURY: Self = Self {
        min_pa: 28.10 * INCH_HG_PA,
        max_pa: 31.00 * INCH_HG_PA,
    };
    /// The 946–1050 hPa window of millibar-scale altimeters.
    pub const HECTOPASCALS: Self = Self {
        min_pa: 94_600.0,
        max_pa: 105_000.0,
    };

    fn validate(&self) -> Result<(), AirdataError> {
        crate::error::in_range("kollsman min", self.min_pa, f64::MIN_POSITIVE, f64::MAX)?;
        crate::error::in_range("kollsman max", self.max_pa, self.min_pa, f64::MAX)?;
        Ok(())
    }
}

/// How a sensitive altimeter is built.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltimeterConfig {
    /// Feet or metres. The long hand turns once per 1 000 units, the short
    /// hand once per 10 000 and the pointer once per 100 000.
    pub unit: DialUnit,
    /// The subscale's range.
    pub kollsman: KollsmanWindow,
    /// Time constant of the aneroid and case volume behind the static line,
    /// s; zero for none.
    pub aneroid_tau_s: f64,
    /// The gear train and hands, driven in altitude in dial units; the stops
    /// are the mechanism's altitude range.
    pub needle: NeedleConfig,
}

/// What a sensitive altimeter shows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltimeterReading {
    /// Altitude under the hands, in dial units.
    pub indicated: f64,
    /// The subscale setting, Pa.
    pub kollsman_pa: f64,
    /// The long hand, rad clockwise from zero: one turn per 1 000 units.
    pub hundreds_hand_rad: f64,
    /// The short hand, rad: one turn per 10 000 units.
    pub thousands_hand_rad: f64,
    /// The thin pointer, rad: one turn per 100 000 units.
    pub ten_thousands_pointer_rad: f64,
}

/// A sensitive altimeter: a stack of aneroid capsules in a case at static
/// pressure, geared to hands calibrated to the standard atmosphere's
/// pressure altitude. Turning the Kollsman knob rotates the dial by the
/// pressure altitude of the setting, so it reads
/// `Hp(static) − Hp(setting)`: field elevation when set to the field's QNH.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Altimeter {
    unit: DialUnit,
    kollsman: KollsmanWindow,
    kollsman_pa: f64,
    aneroid: PneumaticLag,
    hands: NeedleDynamics,
}

impl Altimeter {
    /// An altimeter set to `kollsman_pa` (clamped to the window) and settled
    /// at `static_pa`.
    pub fn new(
        config: AltimeterConfig,
        kollsman_pa: f64,
        static_pa: f64,
    ) -> Result<Self, AirdataError> {
        config.unit.require("altimeter", Quantity::Length)?;
        config.kollsman.validate()?;
        crate::error::in_range("aneroid tau", config.aneroid_tau_s, 0.0, f64::MAX)?;
        let mut altimeter = Self {
            unit: config.unit,
            kollsman: config.kollsman,
            kollsman_pa: config.kollsman.min_pa,
            aneroid: PneumaticLag::new(config.aneroid_tau_s, static_pa),
            hands: NeedleDynamics::new(config.needle, 0.0)?,
        };
        altimeter.set_kollsman(kollsman_pa);
        altimeter.reset(static_pa);
        Ok(altimeter)
    }

    /// Turns the Kollsman knob to `pressure_pa`, stopping at the window's
    /// limits, and returns the setting reached. A non-finite request is
    /// ignored.
    pub fn set_kollsman(&mut self, pressure_pa: f64) -> f64 {
        if pressure_pa.is_finite() {
            self.kollsman_pa = pressure_pa.clamp(self.kollsman.min_pa, self.kollsman.max_pa);
        }
        self.kollsman_pa
    }

    /// The subscale setting, Pa.
    pub fn kollsman_pa(&self) -> f64 {
        self.kollsman_pa
    }

    /// The altitude the mechanism reads, in dial units, at aneroid pressure
    /// `static_pa`.
    pub fn mechanism_reading(&self, static_pa: f64) -> f64 {
        let metres = pressure_altitude_m(static_pa) - pressure_altitude_m(self.kollsman_pa);
        self.unit.from_si(metres)
    }

    /// Settles aneroid and hands at `static_pa`.
    pub fn reset(&mut self, static_pa: f64) {
        self.aneroid.reset(static_pa);
        self.hands.reset(self.mechanism_reading(static_pa));
    }

    /// Advances `dt_s` with `static_pa` at the static connection.
    pub fn step(&mut self, dt_s: f64, static_pa: f64) -> AltimeterReading {
        let sensed = self.aneroid.step(dt_s, static_pa);
        let indicated = self.hands.step(dt_s, self.mechanism_reading(sensed));
        AltimeterReading {
            indicated,
            kollsman_pa: self.kollsman_pa,
            hundreds_hand_rad: hand_angle(indicated, 1_000.0),
            thousands_hand_rad: hand_angle(indicated, 10_000.0),
            ten_thousands_pointer_rad: hand_angle(indicated, 100_000.0),
        }
    }
}

/// A hand turning once per `per_turn` units, rad in `[0, 2π)`.
fn hand_angle(value: f64, per_turn: f64) -> f64 {
    let turns = value / per_turn;
    TAU * (turns - libm::floor(turns))
}
