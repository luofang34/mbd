#[cfg(test)]
mod tests;

/// A pressure line or instrument case as a first-order lag: the volume at the
/// end of a narrow tube fills through the tube's resistance with time constant
/// `tau`. Each step solves the lag exactly (the `exp(−dt/τ)` discretisation)
/// for an input that moves linearly from the previous step's input to this
/// one's, so a steady ramp, such as static pressure in a constant climb, is
/// followed with exactly `τ` of delay at any step size.
///
/// The struct is its own state: copy it to snapshot, assign it to rewind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PneumaticLag {
    tau_s: f64,
    value: f64,
    input: f64,
}

impl PneumaticLag {
    /// A lag with time constant `tau_s`, settled at `initial`. A non-positive
    /// or non-finite `tau_s` makes a direct connection with no lag.
    pub fn new(tau_s: f64, initial: f64) -> Self {
        let tau_s = if tau_s.is_finite() && tau_s > 0.0 {
            tau_s
        } else {
            0.0
        };
        Self {
            tau_s,
            value: initial,
            input: initial,
        }
    }

    /// The time constant, s; zero for a direct connection.
    pub fn tau_s(&self) -> f64 {
        self.tau_s
    }

    /// The pressure at the lagged end.
    pub fn value(&self) -> f64 {
        self.value
    }

    /// Settles the line at `value`, at both ends.
    pub fn reset(&mut self, value: f64) {
        self.value = value;
        self.input = value;
    }

    /// Advances `dt_s` while the open end moves linearly to `input`, and
    /// returns the new lagged value. A non-positive or non-finite `dt_s`
    /// leaves the state unchanged.
    pub fn step(&mut self, dt_s: f64, input: f64) -> f64 {
        if self.tau_s == 0.0 {
            self.value = input;
            self.input = input;
        } else if dt_s > 0.0 && dt_s.is_finite() {
            let x = dt_s / self.tau_s;
            let decay = libm::exp(-x);
            let fill = -libm::expm1(-x);
            let slope_gain = 1.0 - fill / x;
            self.value = decay * self.value + fill * self.input + slope_gain * (input - self.input);
            self.input = input;
        }
        self.value
    }
}
