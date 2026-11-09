use crate::units::{DialUnit, Quantity};
use crate::{AirdataError, NeedleConfig, NeedleDynamics, PiecewiseLinear};

#[cfg(test)]
mod tests;

/// A dial face: a monotone piecewise-linear map from the engraved value to
/// the needle angle. Real airspeed and vertical speed dials are non-linear,
/// expanded where precision matters and compressed elsewhere.
///
/// Angles are radians, clockwise from the dial's reference mark, and strictly
/// increase with value. Beyond the end graduations the needle is held at the
/// end angle, as the linkage reaches its travel limit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DialScale {
    angle_by_value: PiecewiseLinear,
}

impl DialScale {
    /// A dial through `(value, angle rad)` graduations, at least two, with
    /// both value and angle strictly increasing.
    pub fn new(points: &[(f64, f64)]) -> Result<Self, AirdataError> {
        let field = "dial scale";
        if points.len() < 2 {
            return Err(AirdataError::TooFewPoints {
                field,
                count: points.len(),
                min: 2,
            });
        }
        let angle_by_value = PiecewiseLinear::new(field, points)?;
        if let Some(index) = angle_by_value
            .points()
            .iter()
            .zip(angle_by_value.points().iter().skip(1))
            .position(|(a, b)| b.1 <= a.1)
        {
            return Err(AirdataError::NotIncreasing {
                field: "dial scale angles",
                index: index.wrapping_add(1),
            });
        }
        Ok(Self { angle_by_value })
    }

    /// The needle angle, rad, at `value`.
    pub fn angle_rad(&self, value: f64) -> f64 {
        self.angle_by_value.eval(value)
    }

    /// The value read where the needle points at `angle_rad`.
    pub fn value_at(&self, angle_rad: f64) -> f64 {
        self.angle_by_value.inverse(angle_rad)
    }

    /// The first and last graduated values.
    pub fn value_range(&self) -> (f64, f64) {
        self.angle_by_value.domain()
    }

    /// The angles of the first and last graduations, rad.
    pub fn angle_range_rad(&self) -> (f64, f64) {
        let (low, high) = self.value_range();
        (self.angle_rad(low), self.angle_rad(high))
    }

    /// The graduations as `(value, angle rad)`.
    pub fn points(&self) -> &[(f64, f64)] {
        self.angle_by_value.points()
    }
}

/// What a single-needle dial shows after one step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DialReading {
    /// The value under the needle, in dial units: what a pilot reads.
    pub indicated: f64,
    /// The value the mechanism drives the needle towards, in dial units,
    /// before needle dynamics and dial limits.
    pub commanded: f64,
    /// The needle angle, rad, clockwise from the dial's reference mark.
    pub needle_angle_rad: f64,
}

/// The part every single-needle instrument shares: a unit conversion, a dial
/// face and a needle driven in dial angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DialDrive {
    unit: DialUnit,
    scale: DialScale,
    needle: NeedleDynamics,
}

impl DialDrive {
    /// A drive whose needle rests at the first graduation. `needle` stops
    /// outside the scale's angles are kept; the scale itself holds the
    /// command within its graduations.
    pub(crate) fn new(
        instrument: &'static str,
        quantity: Quantity,
        unit: DialUnit,
        scale: DialScale,
        needle: NeedleConfig,
    ) -> Result<Self, AirdataError> {
        unit.require(instrument, quantity)?;
        let (rest, _) = scale.angle_range_rad();
        Ok(Self {
            unit,
            scale,
            needle: NeedleDynamics::new(needle, rest)?,
        })
    }

    pub(crate) fn unit(&self) -> DialUnit {
        self.unit
    }

    pub(crate) fn scale(&self) -> &DialScale {
        &self.scale
    }

    /// Places the needle at rest on `value` in dial units.
    pub(crate) fn reset(&mut self, value: f64) {
        self.needle.reset(self.scale.angle_rad(value));
    }

    /// Drives the needle towards `value` in dial units for `dt_s`.
    pub(crate) fn step(&mut self, dt_s: f64, value: f64) -> DialReading {
        let angle = self.needle.step(dt_s, self.scale.angle_rad(value));
        DialReading {
            indicated: self.scale.value_at(angle),
            commanded: value,
            needle_angle_rad: angle,
        }
    }
}
