//! Error norm and step-size control shared by the adaptive integrators.

use alloc::vec::Vec;

use super::{OdeSystem, StepReport, Tolerances};
use crate::MultibodyError;
use crate::model::StateKind;

/// Per-component error weights of a system under a set of tolerances.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct ErrorScale {
    pub(crate) kinds: Vec<StateKind>,
    /// Characteristic magnitude of each component.
    pub(crate) magnitude: Vec<f64>,
    pub(crate) rtol: Vec<f64>,
    pub(crate) atol: Vec<f64>,
}

impl ErrorScale {
    /// Sizes the weights to `system` and fills them from `tolerances`.
    pub(crate) fn fit<S: OdeSystem + ?Sized>(&mut self, system: &S, tolerances: &Tolerances) {
        let n = system.dim();
        self.kinds.resize(n, StateKind::Auxiliary);
        self.magnitude.resize(n, 1.0);
        self.rtol.resize(n, 0.0);
        self.atol.resize(n, 0.0);
        system.describe(&mut self.kinds, &mut self.magnitude);
        let weights = self.rtol.iter_mut().zip(&mut self.atol);
        for ((rtol, atol), (kind, magnitude)) in weights.zip(self.kinds.iter().zip(&self.magnitude))
        {
            let tol = tolerances.of(*kind);
            *rtol = tol.rel;
            *atol = tol.abs * magnitude;
        }
    }

    /// Root-mean-square of the error estimate `err` in units of the
    /// tolerance; a step is acceptable when this is at most one.
    pub(crate) fn norm(&self, err: &[f64], x0: &[f64], x1: &[f64]) -> f64 {
        let mut sum = 0.0;
        let weights = self.rtol.iter().zip(&self.atol);
        for ((e, (a, b)), (rtol, atol)) in err.iter().zip(x0.iter().zip(x1)).zip(weights) {
            let scale = atol + rtol * a.abs().max(b.abs());
            let ratio = if scale > 0.0 {
                e / scale
            } else {
                e / f64::MIN_POSITIVE
            };
            sum += ratio * ratio;
        }
        if err.is_empty() {
            0.0
        } else {
            libm::sqrt(sum / err.len() as f64)
        }
    }
}

/// Progress of one interval through adaptive internal steps.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Interval {
    t0: f64,
    dt: f64,
    /// Time reached.
    pub(crate) t: f64,
    /// Trial step.
    pub(crate) h: f64,
    /// Whether the trial step ends on the interval's end.
    pub(crate) last: bool,
    max_steps: u32,
    pub(crate) report: StepReport,
}

/// Largest and smallest change of step size between consecutive trials.
const GROW_MAX: f64 = 5.0;
const SHRINK_MIN: f64 = 0.2;
const SAFETY: f64 = 0.9;
/// A trial step within this factor of the remaining interval is stretched
/// to end on it, so no sliver step is left over.
const STRETCH: f64 = 1.1;
/// Steps below this fraction of the interval cannot make progress in
/// double precision against round-off in the time itself.
const MIN_STEP_FRACTION: f64 = 1.0e-12;

impl Interval {
    pub(crate) fn new(t0: f64, dt: f64, first_step: Option<f64>, max_steps: u32) -> Self {
        let mut interval = Self {
            t0,
            dt,
            t: t0,
            h: dt,
            last: true,
            max_steps,
            report: StepReport {
                end_time_s: t0 + dt,
                ..StepReport::default()
            },
        };
        interval.propose(first_step.filter(|h| *h > 0.0).unwrap_or(dt));
        interval
    }

    fn propose(&mut self, h: f64) {
        let remaining = (self.t0 + self.dt) - self.t;
        self.last = h * STRETCH >= remaining;
        self.h = if self.last { remaining } else { h };
    }

    /// Factor by which the step should change after an error `err` (in
    /// tolerance units) from a method whose estimate is of order `order`.
    fn factor(err: f64, order: f64) -> f64 {
        if !err.is_finite() {
            return SHRINK_MIN;
        }
        if err == 0.0 {
            return GROW_MAX;
        }
        (SAFETY * libm::pow(err, -1.0 / order)).clamp(SHRINK_MIN, GROW_MAX)
    }

    /// Records an accepted step; returns whether the interval is complete.
    pub(crate) fn accept(&mut self, err: f64, order: f64) -> Result<bool, MultibodyError> {
        let h = self.h;
        self.report.accepted_steps = self.report.accepted_steps.wrapping_add(1);
        self.report.last_step_s = h;
        if self.last {
            self.t = self.t0 + self.dt;
            return Ok(true);
        }
        self.t += h;
        if self.report.accepted_steps >= self.max_steps {
            return Err(MultibodyError::StepLimitExceeded {
                limit: self.max_steps,
                time_s: self.t,
            });
        }
        self.propose(h * Self::factor(err, order));
        Ok(false)
    }

    /// Records a rejected step and shrinks the trial step.
    pub(crate) fn reject(&mut self, err: f64, order: f64) -> Result<(), MultibodyError> {
        self.report.rejected_steps = self.report.rejected_steps.wrapping_add(1);
        let h = self.h * Self::factor(err, order).min(1.0);
        if h.abs() < MIN_STEP_FRACTION * self.dt.abs() || h == self.h {
            return Err(MultibodyError::StepSizeUnderflow {
                time_s: self.t,
                step_s: h,
            });
        }
        self.propose(h);
        Ok(())
    }

    /// Counts derivative evaluations.
    pub(crate) fn count(&mut self, evals: u32) {
        self.report.derivative_evals = self.report.derivative_evals.wrapping_add(evals);
    }
}
