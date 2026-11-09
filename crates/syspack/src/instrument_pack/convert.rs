use mbd_airdata::{
    AirdataError, AirspeedConfig, AirspeedIndicator, Altimeter, AltimeterConfig, DialScale,
    DialUnit, ISA_P0_PA, KollsmanWindow, NeedleConfig, PiecewiseLinear, PressureGauge,
    PressureGaugeConfig, TABLE_CAPACITY, Tachometer, TachometerConfig, VerticalSpeedConfig,
    VerticalSpeedIndicator,
};

use super::{DialUnitSpec, InstrumentKindSpec, InstrumentSpec, KollsmanSpec, ScalePoint};
use crate::SyspackError;

/// An instrument's mechanism, built from its [`InstrumentSpec`] and at rest
/// in sea-level standard air with the engine stopped. A consumer resets it to
/// its vehicle's state before stepping it.
#[derive(Debug, Clone, PartialEq)]
pub enum InstrumentMechanism {
    /// An airspeed indicator.
    Airspeed(Box<AirspeedIndicator>),
    /// An altimeter, set to 1013.25 hPa or the nearest setting it allows.
    Altimeter(Box<Altimeter>),
    /// A vertical speed indicator.
    VerticalSpeed(Box<VerticalSpeedIndicator>),
    /// A pressure gauge, reading zero.
    PressureGauge(Box<PressureGauge>),
    /// A tachometer, reading zero.
    Tachometer(Box<Tachometer>),
}

impl From<DialUnitSpec> for DialUnit {
    fn from(unit: DialUnitSpec) -> Self {
        match unit {
            DialUnitSpec::Knots => Self::Knots,
            DialUnitSpec::MilesPerHour => Self::MilesPerHour,
            DialUnitSpec::KilometresPerHour => Self::KilometresPerHour,
            DialUnitSpec::Feet => Self::Feet,
            DialUnitSpec::Metres => Self::Metres,
            DialUnitSpec::FeetPerMinute => Self::FeetPerMinute,
            DialUnitSpec::MetresPerSecond => Self::MetresPerSecond,
            DialUnitSpec::InchesOfMercury => Self::InchesOfMercury,
            DialUnitSpec::Hectopascals => Self::Hectopascals,
            DialUnitSpec::Kilopascals => Self::Kilopascals,
            DialUnitSpec::PoundsPerSquareInch => Self::PoundsPerSquareInch,
            DialUnitSpec::RevolutionsPerMinute => Self::RevolutionsPerMinute,
        }
    }
}

/// `items` as a fixed-capacity point buffer and its length, since the
/// mechanism tables hold at most [`TABLE_CAPACITY`] points.
fn fixed_points<T>(
    field: &'static str,
    items: &[T],
    point: impl Fn(&T) -> (f64, f64),
) -> Result<([(f64, f64); TABLE_CAPACITY], usize), AirdataError> {
    let mut points = [(0.0, 0.0); TABLE_CAPACITY];
    if items.len() > TABLE_CAPACITY {
        return Err(AirdataError::TooManyPoints {
            field,
            count: items.len(),
            capacity: TABLE_CAPACITY,
        });
    }
    for (slot, item) in points.iter_mut().zip(items) {
        *slot = point(item);
    }
    Ok((points, items.len()))
}

fn dial_scale(scale: &[ScalePoint]) -> Result<DialScale, AirdataError> {
    let (points, len) = fixed_points("dial scale", scale, |p| (p.value, p.angle_deg.to_radians()))?;
    DialScale::new(points.get(..len).unwrap_or(&[]))
}

fn instrument_error(pairs: &[[f64; 2]]) -> Result<PiecewiseLinear, AirdataError> {
    if pairs.is_empty() {
        return Ok(PiecewiseLinear::constant(0.0));
    }
    let (points, len) = fixed_points("instrument error", pairs, |&[reading, error]| {
        (reading, error)
    })?;
    PiecewiseLinear::new("instrument error", points.get(..len).unwrap_or(&[]))
}

fn kollsman_window(spec: &KollsmanSpec) -> Result<KollsmanWindow, AirdataError> {
    let unit = DialUnit::from(spec.units);
    if !matches!(unit, DialUnit::InchesOfMercury | DialUnit::Hectopascals) {
        return Err(AirdataError::UnitMismatch {
            instrument: "kollsman window",
            unit: unit.name(),
        });
    }
    Ok(KollsmanWindow {
        min_pa: unit.to_si(spec.min),
        max_pa: unit.to_si(spec.max),
    })
}

impl InstrumentSpec {
    /// Builds the instrument's mechanism, failing as loading does if its
    /// parameters are not physical.
    pub fn mechanism(&self) -> Result<InstrumentMechanism, SyspackError> {
        self.build()
            .map_err(|source| SyspackError::InstrumentMechanism {
                instrument: self.id.clone(),
                source,
            })
    }

    /// A needle with this spec's dynamics between `stops`.
    fn needle_config(&self, (min_stop, max_stop): (f64, f64)) -> NeedleConfig {
        NeedleConfig {
            natural_frequency_rad_s: self.needle.natural_frequency_rad_s,
            damping_ratio: self.needle.damping_ratio,
            min_stop,
            max_stop,
        }
    }

    fn build(&self) -> Result<InstrumentMechanism, AirdataError> {
        Ok(match &self.kind {
            InstrumentKindSpec::Airspeed {
                units,
                scale,
                instrument_error: error,
            } => {
                let scale = dial_scale(scale)?;
                let mut indicator = AirspeedIndicator::new(AirspeedConfig {
                    unit: (*units).into(),
                    scale,
                    instrument_error: instrument_error(error)?,
                    needle: self.needle_config(scale.angle_range_rad()),
                })?;
                indicator.reset(ISA_P0_PA, ISA_P0_PA);
                InstrumentMechanism::Airspeed(Box::new(indicator))
            }
            InstrumentKindSpec::Altimeter {
                units,
                kollsman,
                aneroid_tau_s,
                range: [low, high],
            } => {
                let config = AltimeterConfig {
                    unit: (*units).into(),
                    kollsman: kollsman_window(kollsman)?,
                    aneroid_tau_s: *aneroid_tau_s,
                    needle: self.needle_config((*low, *high)),
                };
                let altimeter = Altimeter::new(config, ISA_P0_PA, ISA_P0_PA)?;
                InstrumentMechanism::Altimeter(Box::new(altimeter))
            }
            InstrumentKindSpec::VerticalSpeed {
                units,
                scale,
                leak_tau_s,
            } => {
                let scale = dial_scale(scale)?;
                let config = VerticalSpeedConfig {
                    unit: (*units).into(),
                    scale,
                    leak_tau_s: *leak_tau_s,
                    needle: self.needle_config(scale.angle_range_rad()),
                };
                let indicator = VerticalSpeedIndicator::new(config, ISA_P0_PA)?;
                InstrumentMechanism::VerticalSpeed(Box::new(indicator))
            }
            InstrumentKindSpec::PressureGauge {
                units,
                scale,
                line_tau_s,
            } => {
                let scale = dial_scale(scale)?;
                let config = PressureGaugeConfig {
                    unit: (*units).into(),
                    scale,
                    line_tau_s: *line_tau_s,
                    needle: self.needle_config(scale.angle_range_rad()),
                };
                InstrumentMechanism::PressureGauge(Box::new(PressureGauge::new(config, 0.0)?))
            }
            InstrumentKindSpec::Tachometer {
                units,
                scale,
                drag_cup_tau_s,
            } => {
                let scale = dial_scale(scale)?;
                let config = TachometerConfig {
                    unit: (*units).into(),
                    scale,
                    drag_cup_tau_s: *drag_cup_tau_s,
                    needle: self.needle_config(scale.angle_range_rad()),
                };
                InstrumentMechanism::Tachometer(Box::new(Tachometer::new(config, 0.0)?))
            }
        })
    }
}
