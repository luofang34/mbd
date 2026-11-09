//! The `instruments.json` schema of an instrument pack: each instrument's
//! real designation, mechanism parameters, dial scale, markings, bezel,
//! signal source and optionally how the face is printed, and a default panel layout shared by every vehicle that
//! pins the pack. Vehicles supply only the panel's frame.

use serde::{Deserialize, Serialize};

use crate::SyspackError;

mod convert;
mod dial;
mod validate;

pub use convert::InstrumentMechanism;
pub use dial::{
    DialFaceSpec, FailureSpec, GraduationSpec, LegendSpec, NumeralSpec, PointerFaceSpec,
    PointerShapeSpec,
};
pub use validate::SourceBinding;

#[cfg(test)]
pub(crate) mod tests;

/// The asset of an instrument pack that holds its [`InstrumentPackDocument`].
pub const INSTRUMENTS_ASSET: &str = "instruments.json";

/// Diameter of a standard 3-1/8 in instrument hole, m.
const BEZEL_3_1_8_IN_M: f64 = 0.079_375;
/// Diameter of a standard 2-1/4 in instrument hole, m.
const BEZEL_2_1_4_IN_M: f64 = 0.057_15;

/// The content of `instruments.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstrumentPackDocument {
    /// Every instrument in the pack.
    pub instruments: Vec<InstrumentSpec>,
    /// Where each instrument sits on the panel by default.
    pub layout: Vec<LayoutSlotSpec>,
}

impl InstrumentPackDocument {
    /// Parses and validates `instruments.json` content, failing on the first
    /// problem, including any instrument whose mechanism cannot be built.
    pub fn parse(bytes: &[u8]) -> Result<Self, SyspackError> {
        let document: Self =
            serde_json::from_slice(bytes).map_err(SyspackError::InstrumentsParse)?;
        document.validate()?;
        Ok(document)
    }

    /// The instrument with `id`.
    pub fn instrument(&self, id: &str) -> Option<&InstrumentSpec> {
        self.instruments.iter().find(|spec| spec.id == id)
    }
}

/// One instrument.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstrumentSpec {
    /// Unique slug of the instrument in the pack.
    pub id: String,
    /// The real designation, such as `Kollsman Type C-12 sensitive altimeter`.
    pub name: String,
    /// The panel hole it fits.
    pub bezel: BezelSize,
    /// Where its input comes from; see [`SourceBinding`].
    pub source: String,
    /// The needle's dynamics.
    pub needle: NeedleSpec,
    /// Coloured arcs and radial lines on the dial, in dial units.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub markings: Vec<MarkingSpec>,
    /// What the instrument is, with its mechanism's parameters.
    pub kind: InstrumentKindSpec,
    /// How the face is printed, for a renderer; nothing here moves a needle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dial: Option<DialFaceSpec>,
    /// Further pointers in the same case, after the one `source` and `kind`
    /// describe, each with its own input, mechanism and scale: an engine
    /// gage unit's oil pressure and fuel pressure beside its oil
    /// temperature, or the right tank's pointer on a dual fuel gauge.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pointers: Vec<PointerChannelSpec>,
}

/// One further pointer of an instrument.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PointerChannelSpec {
    /// Slug, unique within the instrument.
    pub id: String,
    /// Where its input comes from; see [`SourceBinding`].
    pub source: String,
    /// Its mechanism and scale. An altimeter has hands of its own and is
    /// never a further pointer.
    pub kind: InstrumentKindSpec,
    /// How its scale is printed, such as a sub-dial's centre and radius.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dial: Option<DialFaceSpec>,
}

/// A standard instrument case size.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum BezelSize {
    /// The 3-1/8 in (79.4 mm) case of primary flight instruments.
    ThreeAndOneEighthInch,
    /// The 2-1/4 in (57.2 mm) case of engine and secondary instruments.
    TwoAndOneQuarterInch,
    /// Any other round case.
    Custom {
        /// Case diameter, m.
        diameter_m: f64,
    },
}

impl BezelSize {
    /// The case diameter, m.
    pub fn diameter_m(&self) -> f64 {
        match self {
            Self::ThreeAndOneEighthInch => BEZEL_3_1_8_IN_M,
            Self::TwoAndOneQuarterInch => BEZEL_2_1_4_IN_M,
            Self::Custom { diameter_m } => *diameter_m,
        }
    }
}

/// A needle as a second-order system.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NeedleSpec {
    /// Undamped natural frequency, rad/s.
    pub natural_frequency_rad_s: f64,
    /// Damping ratio; below 1 the needle overshoots.
    pub damping_ratio: f64,
}

/// One graduation of a dial face.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScalePoint {
    /// The engraved value, in dial units.
    pub value: f64,
    /// Needle angle, degrees clockwise from the dial's reference mark.
    pub angle_deg: f64,
}

/// The unit a dial is engraved in, as [`mbd_airdata::DialUnit`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DialUnitSpec {
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
    /// Hectopascals (millibars).
    Hectopascals,
    /// Kilopascals.
    Kilopascals,
    /// Pounds-force per square inch.
    PoundsPerSquareInch,
    /// Revolutions per minute.
    RevolutionsPerMinute,
    /// Degrees Celsius.
    DegreesCelsius,
    /// United States liquid gallons.
    UsGallons,
}

/// The Kollsman window's range, in its engraved unit.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct KollsmanSpec {
    /// `InchesOfMercury` or `Hectopascals`.
    pub units: DialUnitSpec,
    /// Lowest setting.
    pub min: f64,
    /// Highest setting.
    pub max: f64,
}

/// What an instrument is, with its mechanism's parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum InstrumentKindSpec {
    /// A pitot-static airspeed indicator, calibrated to the CAS law.
    Airspeed {
        /// The dial's speed unit.
        units: DialUnitSpec,
        /// The dial face.
        scale: Vec<ScalePoint>,
        /// Bench instrument error as `[reading, error]` in dial units.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        instrument_error: Vec<[f64; 2]>,
    },
    /// A sensitive altimeter with a Kollsman subscale.
    Altimeter {
        /// `Feet` or `Metres`.
        units: DialUnitSpec,
        /// The subscale's range.
        kollsman: KollsmanSpec,
        /// Aneroid and case time constant, s; zero for none.
        aneroid_tau_s: f64,
        /// The mechanism's altitude range `[low, high]` in dial units.
        range: [f64; 2],
    },
    /// A capillary-leak vertical speed indicator.
    VerticalSpeed {
        /// The dial's rate unit.
        units: DialUnitSpec,
        /// The dial face.
        scale: Vec<ScalePoint>,
        /// Time constant of the calibrated leak, s.
        leak_tau_s: f64,
    },
    /// A Bourdon-tube or capsule pressure gauge.
    PressureGauge {
        /// The dial's pressure unit.
        units: DialUnitSpec,
        /// The dial face.
        scale: Vec<ScalePoint>,
        /// Sensing line time constant, s; zero for none.
        line_tau_s: f64,
    },
    /// A magnetic-drag tachometer.
    Tachometer {
        /// `RevolutionsPerMinute`.
        units: DialUnitSpec,
        /// The dial face.
        scale: Vec<ScalePoint>,
        /// Drag-cup time constant, s.
        drag_cup_tau_s: f64,
    },
    /// A vapour-pressure or resistance thermometer: oil, coolant or
    /// carburettor air temperature.
    Thermometer {
        /// `DegreesCelsius`.
        units: DialUnitSpec,
        /// The dial face.
        scale: Vec<ScalePoint>,
        /// Time constant of the bulb, s; zero for none.
        bulb_tau_s: f64,
    },
    /// A float-transmitter fuel-quantity gauge for one tank.
    FuelQuantity {
        /// `UsGallons`.
        units: DialUnitSpec,
        /// The dial face.
        scale: Vec<ScalePoint>,
        /// Time constant of the damped float, s; zero for none.
        transmitter_tau_s: f64,
    },
}

/// The colour of a dial marking, as the certification rules assign them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MarkingColour {
    /// Flap operating range on an airspeed indicator.
    White,
    /// Normal operating range.
    Green,
    /// Caution range.
    Yellow,
    /// Limit: never exceed.
    Red,
    /// Best single-engine rate of climb, or another advisory value.
    Blue,
}

/// A marking on a dial face.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum MarkingSpec {
    /// An arc between two values.
    Arc {
        /// Start, dial units.
        from: f64,
        /// End, dial units; above `from`.
        to: f64,
        /// Its colour.
        colour: MarkingColour,
    },
    /// A radial line at one value.
    Radial {
        /// Position, dial units.
        at: f64,
        /// Its colour.
        colour: MarkingColour,
    },
}

/// Where one instrument sits on the panel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayoutSlotSpec {
    /// Unique slot name, such as `top_left`.
    pub slot: String,
    /// The instrument's id.
    pub instrument: String,
    /// The instrument's centre on the panel plane as `[right, up]` from the
    /// vehicle's panel origin, m.
    pub offset_m: [f64; 2],
}
