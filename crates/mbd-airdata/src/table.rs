use crate::AirdataError;
use crate::error::finite;

#[cfg(test)]
mod tests;

/// Most points any table holds. Real calibration cards and dial faces have a
/// dozen or so; the fixed capacity keeps tables `Copy` and allocation-free.
pub const TABLE_CAPACITY: usize = 16;

/// A piecewise-linear function through up to [`TABLE_CAPACITY`] points with
/// strictly increasing abscissae, held flat beyond its end points so a
/// lookup outside the calibrated range reads the nearest calibrated value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PiecewiseLinear {
    points: [(f64, f64); TABLE_CAPACITY],
    len: usize,
}

impl PiecewiseLinear {
    /// A table through `points`; `field` names it in errors.
    pub fn new(field: &'static str, points: &[(f64, f64)]) -> Result<Self, AirdataError> {
        if points.is_empty() {
            return Err(AirdataError::TooFewPoints {
                field,
                count: 0,
                min: 1,
            });
        }
        if points.len() > TABLE_CAPACITY {
            return Err(AirdataError::TooManyPoints {
                field,
                count: points.len(),
                capacity: TABLE_CAPACITY,
            });
        }
        let mut stored = [(0.0, 0.0); TABLE_CAPACITY];
        let mut previous = f64::NEG_INFINITY;
        for (index, (slot, &(x, y))) in stored.iter_mut().zip(points).enumerate() {
            finite(field, x)?;
            finite(field, y)?;
            if x <= previous {
                return Err(AirdataError::NotIncreasing { field, index });
            }
            previous = x;
            *slot = (x, y);
        }
        Ok(Self {
            points: stored,
            len: points.len(),
        })
    }

    /// The constant function `y`.
    pub const fn constant(y: f64) -> Self {
        Self {
            points: [(0.0, y); TABLE_CAPACITY],
            len: 1,
        }
    }

    /// The table's points.
    pub fn points(&self) -> &[(f64, f64)] {
        self.points.get(..self.len).unwrap_or(&[])
    }

    /// The function at `x`, held at the end values outside the table.
    pub fn eval(&self, x: f64) -> f64 {
        interpolate(self.points().iter().copied(), x)
    }

    /// Whether the ordinates also strictly increase, so [`Self::inverse`] is
    /// a function.
    pub fn is_strictly_increasing(&self) -> bool {
        self.points()
            .iter()
            .zip(self.points().iter().skip(1))
            .all(|(a, b)| b.1 > a.1)
    }

    /// The abscissa where the function takes `y`, held at the end points.
    /// Meaningful only when [`Self::is_strictly_increasing`] holds.
    pub fn inverse(&self, y: f64) -> f64 {
        interpolate(self.points().iter().map(|&(x, y)| (y, x)), y)
    }

    /// The first and last abscissae.
    pub fn domain(&self) -> (f64, f64) {
        let first = self.points().first().map_or(0.0, |p| p.0);
        let last = self.points().last().map_or(0.0, |p| p.0);
        (first, last)
    }
}

/// Linear interpolation over points sorted by increasing abscissa.
fn interpolate(points: impl Iterator<Item = (f64, f64)>, x: f64) -> f64 {
    let mut previous: Option<(f64, f64)> = None;
    for (x1, y1) in points {
        match previous {
            None if x <= x1 => return y1,
            Some((x0, y0)) if x <= x1 => {
                let t = (x - x0) / (x1 - x0);
                return y0 + t * (y1 - y0);
            }
            _ => previous = Some((x1, y1)),
        }
    }
    previous.map_or(0.0, |p| p.1)
}
