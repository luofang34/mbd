//! Dormand–Prince 5(4): embedded explicit Runge–Kutta with error control.

use alloc::vec::Vec;

use super::step_control::{ErrorScale, Interval};
use super::{Integrator, OdeSystem, StepReport, Tolerances, check_dim, check_interval};
use crate::MultibodyError;

/// Dormand–Prince 5(4) pair with local extrapolation: a fifth-order solution
/// advanced under the error estimate of its embedded fourth-order one.
///
/// It sub-steps inside the requested interval to meet `tolerances` and ends
/// exactly on the interval's end. Six derivative evaluations per accepted
/// step (the seventh stage is the next step's first). The method for smooth,
/// non-stiff motion where accuracy per evaluation matters: orbits, free
/// tumbling, mechanisms away from contact.
#[derive(Debug, Clone, PartialEq)]
pub struct DormandPrince54 {
    /// Error tolerances per state group.
    pub tolerances: Tolerances,
    /// First trial step of each interval, s; the whole interval when absent.
    /// A host that wants continuity across intervals feeds back
    /// [`StepReport::last_step_s`].
    pub first_step_s: Option<f64>,
    /// Largest number of accepted internal steps per interval.
    pub max_steps: u32,
    k: [Vec<f64>; 7],
    stage: Vec<f64>,
    err: Vec<f64>,
    scale: ErrorScale,
}

const C: [f64; 7] = [0.0, 0.2, 0.3, 0.8, 8.0 / 9.0, 1.0, 1.0];
const A2: [f64; 1] = [0.2];
const A3: [f64; 2] = [3.0 / 40.0, 9.0 / 40.0];
const A4: [f64; 3] = [44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0];
const A5: [f64; 4] = [
    19372.0 / 6561.0,
    -25360.0 / 2187.0,
    64448.0 / 6561.0,
    -212.0 / 729.0,
];
const A6: [f64; 5] = [
    9017.0 / 3168.0,
    -355.0 / 33.0,
    46732.0 / 5247.0,
    49.0 / 176.0,
    -5103.0 / 18656.0,
];
/// Fifth-order weights; also the seventh stage's row.
const B: [f64; 6] = [
    35.0 / 384.0,
    0.0,
    500.0 / 1113.0,
    125.0 / 192.0,
    -2187.0 / 6784.0,
    11.0 / 84.0,
];
/// Fifth-order minus fourth-order weights.
const E: [f64; 7] = [
    71.0 / 57600.0,
    0.0,
    -71.0 / 16695.0,
    71.0 / 1920.0,
    -17253.0 / 339200.0,
    22.0 / 525.0,
    -1.0 / 40.0,
];
/// Order of the error estimate plus one, the exponent of step control.
const CONTROL_ORDER: f64 = 5.0;

/// `out = x + h Σ aⱼ kⱼ`.
fn combine(out: &mut [f64], x: &[f64], h: f64, a: &[f64], k: &[Vec<f64>]) {
    for (i, (o, xi)) in out.iter_mut().zip(x).enumerate() {
        let mut sum = 0.0;
        for (aj, kj) in a.iter().zip(k) {
            sum += aj * kj.get(i).copied().unwrap_or(0.0);
        }
        *o = xi + h * sum;
    }
}

impl DormandPrince54 {
    /// An integrator meeting `tolerances`, starting each interval with the
    /// whole interval as its trial step.
    pub const fn new(tolerances: Tolerances) -> Self {
        Self {
            tolerances,
            first_step_s: None,
            max_steps: 100_000,
            k: [
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
            ],
            stage: Vec::new(),
            err: Vec::new(),
            scale: ErrorScale {
                kinds: Vec::new(),
                magnitude: Vec::new(),
                rtol: Vec::new(),
                atol: Vec::new(),
            },
        }
    }

    fn fit(&mut self, n: usize) {
        for k in &mut self.k {
            k.resize(n, 0.0);
        }
        self.stage.resize(n, 0.0);
        self.err.resize(n, 0.0);
    }

    /// Evaluates stages two to seven from `k[0]`; leaves the fifth-order
    /// candidate in `stage` and returns its error in tolerance units.
    fn trial<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t: f64,
        h: f64,
        x: &[f64],
    ) -> Result<f64, MultibodyError> {
        let rows: [&[f64]; 6] = [&A2, &A3, &A4, &A5, &A6, &B];
        for (s, (row, c)) in rows.iter().zip(C.iter().skip(1)).enumerate() {
            let (done, rest) = self.k.split_at_mut(s + 1);
            combine(&mut self.stage, x, h, row, done);
            system.retract(&mut self.stage);
            if let Some(next) = rest.first_mut() {
                system.derivative(t + c * h, &self.stage, next)?;
            }
        }
        for (i, e) in self.err.iter_mut().enumerate() {
            let mut sum = 0.0;
            for (ej, kj) in E.iter().zip(&self.k) {
                sum += ej * kj.get(i).copied().unwrap_or(0.0);
            }
            *e = h * sum;
        }
        Ok(self.scale.norm(&self.err, x, &self.stage))
    }
}

impl Integrator for DormandPrince54 {
    fn advance<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t0: f64,
        dt: f64,
        x: &mut [f64],
    ) -> Result<StepReport, MultibodyError> {
        check_dim(system.dim(), x.len())?;
        let mut run = Interval::new(t0, dt, self.first_step_s, self.max_steps);
        if !check_interval(t0, dt)? {
            return Ok(run.report);
        }
        self.fit(x.len());
        self.scale.fit(system, &self.tolerances);
        let mut have_first = false;
        loop {
            if !have_first {
                let [k1, ..] = &mut self.k;
                system.derivative(run.t, x, k1)?;
                run.count(1);
            }
            let err = self.trial(system, run.t, run.h, x)?;
            run.count(6);
            if err.is_nan() || err > 1.0 {
                // The first stage is unchanged by a rejection.
                have_first = true;
                run.reject(err, CONTROL_ORDER)?;
                continue;
            }
            x.copy_from_slice(&self.stage);
            let done = run.accept(err, CONTROL_ORDER)?;
            let event = system.accept(run.t, x)?;
            run.report.event |= event;
            // First-same-as-last: the seventh stage was evaluated at the
            // accepted state, unless an event has just changed it.
            have_first = !event;
            self.k.swap(0, 6);
            if done {
                return Ok(run.report);
            }
        }
    }
}
