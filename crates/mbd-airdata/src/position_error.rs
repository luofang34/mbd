//! Static-source position error: the airframe disturbs the flow around the
//! static ports, so they sense `ps + Δps` rather than the free-stream static
//! pressure `ps`.
//!
//! The error is tabulated as `Δps/qc` against calibrated airspeed for each
//! flap setting, the form flight-test reports give. Positive `Δps` means the
//! ports read high: the airspeed indicator then reads low and the altimeter
//! reads low. The total-pressure probe is assumed exact, as it is to within
//! a fraction of a percent below about 20° of flow angle, so all the error is
//! in static pressure.

use crate::pitot::{cas_from_impact_pressure_m_s, impact_pressure_from_cas_pa};
use crate::{AirdataError, PiecewiseLinear};

#[cfg(test)]
mod tests;

/// Most flap settings a [`PositionErrorTable`] holds.
const MAX_FLAP_CURVES: usize = 6;

/// `Δps/qc` against calibrated airspeed at one flap setting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionErrorCurve {
    flap: f64,
    ratio_by_cas: PiecewiseLinear,
}

impl PositionErrorCurve {
    /// A curve at flap setting `flap` (in whatever unit the table's lookups
    /// use, such as degrees) through `(CAS m/s, Δps/qc)` points.
    pub fn new(flap: f64, points: &[(f64, f64)]) -> Result<Self, AirdataError> {
        crate::error::finite("position error flap setting", flap)?;
        Ok(Self {
            flap,
            ratio_by_cas: PiecewiseLinear::new("position error curve", points)?,
        })
    }

    /// A curve from a flight-test calibration card's `(IAS m/s, CAS m/s)`
    /// pairs, with IAS free of instrument error. With exact total pressure,
    /// the indicator senses `qc − Δps` and shows the CAS-law speed of it, so
    /// `Δps/qc = (qc(CAS) − qc(IAS)) / qc(CAS)`.
    pub fn from_ias_cas_pairs(flap: f64, pairs_m_s: &[(f64, f64)]) -> Result<Self, AirdataError> {
        let mut points = [(0.0, 0.0); crate::TABLE_CAPACITY];
        if pairs_m_s.len() > points.len() {
            return Err(AirdataError::TooManyPoints {
                field: "position error calibration",
                count: pairs_m_s.len(),
                capacity: points.len(),
            });
        }
        for (slot, &(ias, cas)) in points.iter_mut().zip(pairs_m_s) {
            crate::error::in_range("calibrated airspeed", cas, f64::MIN_POSITIVE, f64::MAX)?;
            crate::error::in_range("indicated airspeed", ias, 0.0, f64::MAX)?;
            let qc_cas = impact_pressure_from_cas_pa(cas);
            *slot = (cas, (qc_cas - impact_pressure_from_cas_pa(ias)) / qc_cas);
        }
        let used = points.get(..pairs_m_s.len()).unwrap_or(&[]);
        Self::new(flap, used)
    }

    /// The flap setting.
    pub fn flap(&self) -> f64 {
        self.flap
    }

    /// `Δps/qc` at calibrated airspeed `cas_m_s`.
    pub fn ratio(&self, cas_m_s: f64) -> f64 {
        self.ratio_by_cas.eval(cas_m_s)
    }
}

/// Position error over every flap setting, interpolated linearly between the
/// two nearest settings and held beyond the first and last.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionErrorTable {
    curves: [PositionErrorCurve; MAX_FLAP_CURVES],
    len: usize,
}

impl PositionErrorTable {
    /// No position error: ports that sense free-stream static pressure.
    pub const fn none() -> Self {
        let zero = PositionErrorCurve {
            flap: 0.0,
            ratio_by_cas: PiecewiseLinear::constant(0.0),
        };
        Self {
            curves: [zero; MAX_FLAP_CURVES],
            len: 1,
        }
    }

    /// A table of `curves` in strictly increasing flap order.
    pub fn new(curves: &[PositionErrorCurve]) -> Result<Self, AirdataError> {
        let field = "position error flap settings";
        let mut table = Self::none();
        if curves.is_empty() {
            return Err(AirdataError::TooFewPoints {
                field,
                count: 0,
                min: 1,
            });
        }
        if curves.len() > MAX_FLAP_CURVES {
            return Err(AirdataError::TooManyPoints {
                field,
                count: curves.len(),
                capacity: MAX_FLAP_CURVES,
            });
        }
        let mut previous = f64::NEG_INFINITY;
        for (index, (slot, curve)) in table.curves.iter_mut().zip(curves).enumerate() {
            if curve.flap <= previous {
                return Err(AirdataError::NotIncreasing { field, index });
            }
            previous = curve.flap;
            *slot = *curve;
        }
        table.len = curves.len();
        Ok(table)
    }

    fn curves(&self) -> &[PositionErrorCurve] {
        self.curves.get(..self.len).unwrap_or(&[])
    }

    /// `Δps/qc` at flap setting `flap` and calibrated airspeed `cas_m_s`.
    pub fn ratio(&self, flap: f64, cas_m_s: f64) -> f64 {
        let mut below: Option<&PositionErrorCurve> = None;
        for curve in self.curves() {
            if flap <= curve.flap {
                return match below {
                    None => curve.ratio(cas_m_s),
                    Some(low) => {
                        let t = (flap - low.flap) / (curve.flap - low.flap);
                        let low_ratio = low.ratio(cas_m_s);
                        low_ratio + t * (curve.ratio(cas_m_s) - low_ratio)
                    }
                };
            }
            below = Some(curve);
        }
        below.map_or(0.0, |curve| curve.ratio(cas_m_s))
    }

    /// The static pressure the ports sense, Pa, given free-stream static
    /// pressure `static_pa`, true impact pressure `qc_pa` and flap setting.
    pub fn sensed_static_pa(&self, static_pa: f64, qc_pa: f64, flap: f64) -> f64 {
        let cas_m_s = cas_from_impact_pressure_m_s(qc_pa);
        static_pa + self.ratio(flap, cas_m_s) * qc_pa.max(0.0)
    }
}
