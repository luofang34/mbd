//! Classical fourth-order Runge–Kutta at a fixed step.

use alloc::vec::Vec;

use super::{Integrator, OdeSystem, StepReport, check_dim, check_interval};
use crate::MultibodyError;

/// Classical RK4. Each `advance` takes `substeps` equal steps; four
/// derivative evaluations per step, no error control. The cheapest choice
/// when the step resolves the fastest dynamics (`ω·h ≲ 1`), and unstable
/// beyond `ω·h ≈ 2.8`.
#[derive(Debug, Clone, PartialEq)]
pub struct Rk4 {
    /// Equal steps taken per requested interval, at least one.
    pub substeps: u32,
    k: [Vec<f64>; 4],
    stage: Vec<f64>,
}

impl Default for Rk4 {
    fn default() -> Self {
        Self::new()
    }
}

impl Rk4 {
    /// One step per requested interval.
    pub const fn new() -> Self {
        Self::with_substeps(1)
    }

    /// `substeps` equal steps per requested interval.
    pub const fn with_substeps(substeps: u32) -> Self {
        Self {
            substeps,
            k: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            stage: Vec::new(),
        }
    }

    fn fit(&mut self, n: usize) {
        for k in &mut self.k {
            k.resize(n, 0.0);
        }
        self.stage.resize(n, 0.0);
    }

    fn step<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t: f64,
        h: f64,
        x: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let [k1, k2, k3, k4] = &mut self.k;
        let stage = &mut self.stage;
        system.derivative(t, x, k1)?;
        offset(stage, x, k1, 0.5 * h);
        system.retract(stage);
        system.derivative(t + 0.5 * h, stage, k2)?;
        offset(stage, x, k2, 0.5 * h);
        system.retract(stage);
        system.derivative(t + 0.5 * h, stage, k3)?;
        offset(stage, x, k3, h);
        system.retract(stage);
        system.derivative(t + h, stage, k4)?;
        let w = h / 6.0;
        let ks = k1.iter().zip(k2.iter()).zip(k3.iter()).zip(k4.iter());
        for (xi, (((a, b), c), d)) in x.iter_mut().zip(ks) {
            *xi += (a + 2.0 * b + 2.0 * c + d) * w;
        }
        system.retract(x);
        Ok(())
    }
}

/// `out = x + k * h`, component by component.
fn offset(out: &mut [f64], x: &[f64], k: &[f64], h: f64) {
    for (o, (xi, ki)) in out.iter_mut().zip(x.iter().zip(k)) {
        *o = xi + ki * h;
    }
}

impl Integrator for Rk4 {
    fn advance<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t0: f64,
        dt: f64,
        x: &mut [f64],
    ) -> Result<StepReport, MultibodyError> {
        check_dim(system.dim(), x.len())?;
        let mut report = StepReport {
            end_time_s: t0 + dt,
            ..StepReport::default()
        };
        if !check_interval(t0, dt)? {
            return Ok(report);
        }
        self.fit(x.len());
        let n = self.substeps.max(1);
        let h = dt / f64::from(n);
        for i in 0..n {
            // Sub-step times are multiples of `h` from `t0`, not a running
            // sum, so the last one ends on `t0 + dt` without drift.
            let t = t0 + f64::from(i) * h;
            self.step(system, t, h, x)?;
            let t_next = if i + 1 == n { t0 + dt } else { t + h };
            report.event |= system.accept(t_next, x)?;
            report.accepted_steps = report.accepted_steps.wrapping_add(1);
            report.derivative_evals = report.derivative_evals.wrapping_add(4);
        }
        report.last_step_s = h;
        Ok(report)
    }
}
