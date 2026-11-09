use crate::AirdataError;
use crate::error::{finite, in_range};

#[cfg(test)]
mod tests;

/// Largest sub-step as a fraction of the natural period's radian, `ωn·h`.
const MAX_PHASE_PER_SUBSTEP: f64 = 0.2;
/// Most sub-steps per call, which bounds the work of one step.
const MAX_SUBSTEPS: f64 = 64.0;

/// A needle's moving parts as a second-order system: the pointer, its staff
/// and the linkage have inertia, the hairspring and drive give stiffness, and
/// the damping is what makes a needle settle or ring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NeedleConfig {
    /// Undamped natural frequency `ωn`, rad/s.
    pub natural_frequency_rad_s: f64,
    /// Damping ratio `ζ`; below 1 the needle overshoots.
    pub damping_ratio: f64,
    /// Lower hard stop, in the units the needle is driven in.
    pub min_stop: f64,
    /// Upper hard stop, in the units the needle is driven in.
    pub max_stop: f64,
}

impl NeedleConfig {
    /// Fails unless `0 < ωn ≤ 10⁴ rad/s`, `0 ≤ ζ ≤ 100` and the stops are
    /// finite and ordered.
    pub fn validate(&self) -> Result<(), AirdataError> {
        in_range(
            "needle natural frequency",
            self.natural_frequency_rad_s,
            f64::MIN_POSITIVE,
            1.0e4,
        )?;
        in_range("needle damping ratio", self.damping_ratio, 0.0, 100.0)?;
        finite("needle min stop", self.min_stop)?;
        in_range("needle max stop", self.max_stop, self.min_stop, f64::MAX)?;
        Ok(())
    }
}

/// A needle following its commanded position. Steps integrate with the
/// implicit midpoint rule, which is unconditionally stable for this linear
/// system, over a fixed number of sub-steps derived from `dt` and `ωn`; a
/// needle against a stop loses its velocity, as a pointer hitting its pin.
///
/// The struct is its own state: copy it to snapshot, assign it to rewind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NeedleDynamics {
    config: NeedleConfig,
    position: f64,
    rate: f64,
}

impl NeedleDynamics {
    /// A needle at rest at `position`, clamped to the stops.
    pub fn new(config: NeedleConfig, position: f64) -> Result<Self, AirdataError> {
        config.validate()?;
        let mut needle = Self {
            config,
            position: config.min_stop,
            rate: 0.0,
        };
        needle.reset(position);
        Ok(needle)
    }

    /// The configuration.
    pub fn config(&self) -> &NeedleConfig {
        &self.config
    }

    /// The needle's position.
    pub fn position(&self) -> f64 {
        self.position
    }

    /// The needle's rate of change of position, per second.
    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Places the needle at rest at `position`, clamped to the stops. A
    /// non-finite position leaves it where it is.
    pub fn reset(&mut self, position: f64) {
        if position.is_finite() {
            self.position = position.clamp(self.config.min_stop, self.config.max_stop);
            self.rate = 0.0;
        }
    }

    /// Advances `dt_s` towards `command` and returns the new position. A
    /// non-finite command or a non-positive or non-finite `dt_s` leaves the
    /// needle unchanged.
    pub fn step(&mut self, dt_s: f64, command: f64) -> f64 {
        if !(dt_s > 0.0 && dt_s.is_finite() && command.is_finite()) {
            return self.position;
        }
        let omega = self.config.natural_frequency_rad_s;
        let substeps = libm::ceil(dt_s * omega / MAX_PHASE_PER_SUBSTEP).clamp(1.0, MAX_SUBSTEPS);
        let h = dt_s / substeps;
        // Error dynamics e' = v, v' = −ωn² e − 2ζωn v, solved by the implicit
        // midpoint rule: (I − hA/2) x₁ = (I + hA/2) x₀.
        let k = h * omega * omega / 2.0;
        let c = h * self.config.damping_ratio * omega;
        let det = 1.0 + c + h * k / 2.0;
        let mut remaining = substeps;
        while remaining > 0.0 {
            let error = self.position - command;
            let r0 = error + h / 2.0 * self.rate;
            let r1 = -k * error + (1.0 - c) * self.rate;
            let next_error = ((1.0 + c) * r0 + h / 2.0 * r1) / det;
            self.rate = (r1 - k * r0) / det;
            self.position = command + next_error;
            self.apply_stops();
            remaining -= 1.0;
        }
        self.position
    }

    fn apply_stops(&mut self) {
        if self.position <= self.config.min_stop {
            self.position = self.config.min_stop;
            self.rate = self.rate.max(0.0);
        } else if self.position >= self.config.max_stop {
            self.position = self.config.max_stop;
            self.rate = self.rate.min(0.0);
        }
    }
}
