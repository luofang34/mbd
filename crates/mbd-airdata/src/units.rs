use crate::AirdataError;

/// One inch of mercury at 0 °C, Pa.
const INCH_HG_PA: f64 = 3_386.389;
/// One pound-force per square inch, Pa.
const PSI_PA: f64 = 6_894.757_293_168;
/// One international knot, m/s.
const KNOT_M_S: f64 = 1_852.0 / 3_600.0;
/// One statute mile per hour, m/s.
const MPH_M_S: f64 = 0.447_04;
/// One international foot, m.
const FOOT_M: f64 = 0.3048;

/// The unit a dial is engraved in. Mechanisms work in SI and convert at the
/// dial, so one mechanism serves a knots ASI and a km/h ASI alike.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DialUnit {
    /// International knots.
    Knots,
    /// Statute miles per hour.
    MilesPerHour,
    /// Kilometres per hour.
    KilometresPerHour,
    /// International feet.
    Feet,
    /// Metres.
    Metres,
    /// Feet per minute.
    FeetPerMinute,
    /// Metres per second.
    MetresPerSecond,
    /// Inches of mercury at 0 °C.
    InchesOfMercury,
    /// Hectopascals, the same as millibars.
    Hectopascals,
    /// Kilopascals.
    Kilopascals,
    /// Pounds-force per square inch.
    PoundsPerSquareInch,
    /// Revolutions per minute.
    RevolutionsPerMinute,
}

/// The physical quantity a unit measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Quantity {
    Speed,
    Length,
    Pressure,
    RotationRate,
}

impl DialUnit {
    /// The unit's name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Knots => "knots",
            Self::MilesPerHour => "miles per hour",
            Self::KilometresPerHour => "kilometres per hour",
            Self::Feet => "feet",
            Self::Metres => "metres",
            Self::FeetPerMinute => "feet per minute",
            Self::MetresPerSecond => "metres per second",
            Self::InchesOfMercury => "inches of mercury",
            Self::Hectopascals => "hectopascals",
            Self::Kilopascals => "kilopascals",
            Self::PoundsPerSquareInch => "pounds per square inch",
            Self::RevolutionsPerMinute => "revolutions per minute",
        }
    }

    /// The SI value of one dial unit: m/s, m, Pa, or rev/min for rotation.
    fn si_per_unit(self) -> f64 {
        match self {
            Self::Knots => KNOT_M_S,
            Self::MilesPerHour => MPH_M_S,
            Self::KilometresPerHour => 1.0 / 3.6,
            Self::Feet => FOOT_M,
            Self::Metres | Self::MetresPerSecond | Self::RevolutionsPerMinute => 1.0,
            Self::FeetPerMinute => FOOT_M / 60.0,
            Self::InchesOfMercury => INCH_HG_PA,
            Self::Hectopascals => 100.0,
            Self::Kilopascals => 1_000.0,
            Self::PoundsPerSquareInch => PSI_PA,
        }
    }

    /// `si` (m/s, m, Pa or rev/min) in this unit.
    pub fn from_si(self, si: f64) -> f64 {
        si / self.si_per_unit()
    }

    /// `value` in this unit, in SI (m/s, m, Pa or rev/min).
    pub fn to_si(self, value: f64) -> f64 {
        value * self.si_per_unit()
    }

    pub(crate) fn quantity(self) -> Quantity {
        match self {
            Self::Knots
            | Self::MilesPerHour
            | Self::KilometresPerHour
            | Self::FeetPerMinute
            | Self::MetresPerSecond => Quantity::Speed,
            Self::Feet | Self::Metres => Quantity::Length,
            Self::InchesOfMercury
            | Self::Hectopascals
            | Self::Kilopascals
            | Self::PoundsPerSquareInch => Quantity::Pressure,
            Self::RevolutionsPerMinute => Quantity::RotationRate,
        }
    }

    /// Fails unless this unit measures `quantity`.
    pub(crate) fn require(
        self,
        instrument: &'static str,
        quantity: Quantity,
    ) -> Result<Self, AirdataError> {
        if self.quantity() == quantity {
            Ok(self)
        } else {
            Err(AirdataError::UnitMismatch {
                instrument,
                unit: self.name(),
            })
        }
    }
}
