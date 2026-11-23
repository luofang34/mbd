//! Time integrators over a flat state vector, independent of what the state
//! describes.
//!
//! # Contract between an integrator and a system
//!
//! - The state lives on a manifold embedded in `ℝⁿ` (unit quaternions). The
//!   derivative is a vector of the embedding space, so an integrator forms
//!   states by ordinary vector arithmetic and then calls
//!   [`OdeSystem::retract`] on **every** state it builds, stage or final,
//!   before handing it back to the system.
//! - [`OdeSystem::derivative`] may be called at any retracted state any
//!   number of times; it has no side effect on the result of later calls.
//! - After each accepted step the integrator calls [`OdeSystem::accept`],
//!   which may apply discrete events and edit the state. If it reports a
//!   change, derivatives kept from before the boundary are stale.
//! - [`Integrator::advance`] covers exactly the requested interval: internal
//!   sub-steps are an implementation detail and the last one ends on
//!   `t0 + dt`.
//! - An integrator carries nothing from one `advance` to the next except
//!   scratch storage and its configuration, so equal inputs give equal bits
//!   and a restored state continues identically.

use crate::MultibodyError;
use crate::model::StateKind;

mod dense;
mod dormand_prince;
mod rk4;
mod rosenbrock;
mod step_control;

pub use dormand_prince::DormandPrince54;
pub use rk4::Rk4;
pub use rosenbrock::Rosenbrock23;

/// A first-order system `ẋ = f(t, x)` on a flat state.
pub trait OdeSystem {
    /// Number of state components.
    fn dim(&self) -> usize;

    /// Writes `f(t, x)` into `x_dot`. `&mut self` admits scratch storage
    /// only: the result depends on nothing but `t` and `x`.
    fn derivative(&mut self, t: f64, x: &[f64], x_dot: &mut [f64]) -> Result<(), MultibodyError>;

    /// Projects `x` back onto the state manifold.
    fn retract(&self, x: &mut [f64]);

    /// Writes the kind and the characteristic magnitude of each component.
    /// Absolute tolerances and finite-difference steps are multiples of the
    /// magnitude, so they follow the size of the system described.
    fn describe(&self, kinds: &mut [StateKind], magnitudes: &mut [f64]);

    /// Called once at the end of every accepted step, at time `t` and state
    /// `x`. Returns whether a discrete event changed anything.
    fn accept(&mut self, t: f64, x: &mut [f64]) -> Result<bool, MultibodyError> {
        let _unused = (t, x);
        Ok(false)
    }
}

/// Work done by one [`Integrator::advance`] call.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StepReport {
    /// Time reached: exactly `t0 + dt`.
    pub end_time_s: f64,
    /// Accepted internal steps.
    pub accepted_steps: u32,
    /// Trial steps rejected by error control.
    pub rejected_steps: u32,
    /// Calls to [`OdeSystem::derivative`], Jacobian columns included.
    pub derivative_evals: u32,
    /// Size of the last accepted internal step, s. Feeding it back as the
    /// first trial step of the next interval is the host's choice.
    pub last_step_s: f64,
    /// Whether any accepted boundary reported a discrete event.
    pub event: bool,
}

/// An integrator advancing a system over a requested interval.
pub trait Integrator {
    /// Advances `x` from `t0` to exactly `t0 + dt`. On error `x` holds
    /// whatever internal boundary was reached; a caller that needs the
    /// starting state back keeps a copy, as
    /// [`crate::MultibodyModel::advance`] does.
    fn advance<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t0: f64,
        dt: f64,
        x: &mut [f64],
    ) -> Result<StepReport, MultibodyError>;
}

/// Relative and absolute error tolerance of one state group. The absolute
/// part is a fraction of the group's characteristic magnitude, so it is
/// dimensionless and holds at any scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerance {
    /// Admissible error relative to the component's current magnitude.
    pub rel: f64,
    /// Admissible error relative to the group's characteristic magnitude.
    pub abs: f64,
}

/// Error tolerances per state group.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerances {
    /// Translations.
    pub length: Tolerance,
    /// Quaternion components.
    pub attitude: Tolerance,
    /// Hinge angles.
    pub angle: Tolerance,
    /// Translational rates.
    pub linear_rate: Tolerance,
    /// Rotational rates.
    pub angular_rate: Tolerance,
    /// Force-element states.
    pub auxiliary: Tolerance,
}

impl Tolerances {
    /// The same tolerance for every group.
    pub const fn uniform(rel: f64, abs: f64) -> Self {
        let t = Tolerance { rel, abs };
        Self {
            length: t,
            attitude: t,
            angle: t,
            linear_rate: t,
            angular_rate: t,
            auxiliary: t,
        }
    }

    /// Tolerance of the group a component of `kind` belongs to.
    pub const fn of(&self, kind: StateKind) -> Tolerance {
        match kind {
            StateKind::Length => self.length,
            StateKind::Attitude => self.attitude,
            StateKind::Angle => self.angle,
            StateKind::LinearRate => self.linear_rate,
            StateKind::AngularRate => self.angular_rate,
            StateKind::Auxiliary => self.auxiliary,
        }
    }
}

/// Rejects a non-finite or negative interval; `Ok(false)` means there is
/// nothing to integrate.
pub(crate) fn check_interval(t0: f64, dt: f64) -> Result<bool, MultibodyError> {
    if !dt.is_finite() || dt < 0.0 {
        return Err(MultibodyError::InvalidParameter {
            field: "dt_s",
            value: dt,
        });
    }
    if !t0.is_finite() {
        return Err(MultibodyError::InvalidParameter {
            field: "t0_s",
            value: t0,
        });
    }
    Ok(dt > 0.0)
}

/// Rejects a state whose length differs from the system's dimension.
pub(crate) fn check_dim(expected: usize, actual: usize) -> Result<(), MultibodyError> {
    if expected == actual {
        Ok(())
    } else {
        Err(MultibodyError::StateLengthMismatch { expected, actual })
    }
}
