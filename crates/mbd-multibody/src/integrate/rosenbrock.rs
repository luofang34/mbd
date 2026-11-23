//! Rosenbrock 2(3): a linearly implicit, L-stable method for stiff systems.

use alloc::vec::Vec;

use super::dense::DenseLu;
use super::step_control::{ErrorScale, Interval};
use super::{Integrator, OdeSystem, StepReport, Tolerances, check_dim, check_interval};
use crate::MultibodyError;

/// The Shampine–Reichelt modified Rosenbrock pair of orders 2 and 3 with a
/// finite-difference Jacobian.
///
/// # Why this method
///
/// The stiffness this library meets comes from penalty springs (contact,
/// tethers, joint stops) on small masses: a handful of very fast, strongly
/// damped modes in a system of a few tens of states. For that:
///
/// - It is **L-stable**: a mode too fast for the step is damped out in one
///   step instead of being aliased into a slow undamped oscillation, as an
///   A-stable but non-dissipative rule (implicit midpoint, trapezoid) would
///   do. A body resting on a penalty contact stays at rest at any step.
/// - It is **linearly implicit**: each step is one Jacobian, one LU
///   factorisation and three back-substitutions. There is no Newton
///   iteration that can fail to converge across the kink of a unilateral
///   contact, and the work per step is fixed, which keeps the result
///   deterministic and the cost bounded.
/// - It is a **W-method**: second order survives an inexact Jacobian, so
///   one-sided finite differences are enough and no element has to supply
///   derivatives.
/// - Its embedded error estimate lets it sub-step through an impact and
///   return to the full host step afterwards.
///
/// # Limits
///
/// L-stability is a property of the system linearised at the start of the
/// step. Where one step crosses the onset of a unilateral element, or a
/// stiff line turns through a large angle, that linearisation is wrong and
/// only the error estimate keeps the step honest: with tolerances loose
/// enough to accept everything, such a step can gain energy. Keep the
/// tolerances meaningful and bound the work with `max_steps`.
///
/// It is second order, so RK4 or [`super::DormandPrince54`] are cheaper for
/// smooth non-stiff motion, and the Jacobian costs one derivative evaluation
/// per state per step, so it suits small systems and pays off only when the
/// stiffness would force an explicit method into many sub-steps.
#[derive(Debug, Clone, PartialEq)]
pub struct Rosenbrock23 {
    /// Error tolerances per state group.
    pub tolerances: Tolerances,
    /// First trial step of each interval, s; the whole interval when absent.
    pub first_step_s: Option<f64>,
    /// Largest number of accepted internal steps per interval.
    pub max_steps: u32,
    jacobian: Vec<f64>,
    lu: DenseLu,
    f: [Vec<f64>; 3],
    k: [Vec<f64>; 3],
    x_new: Vec<f64>,
    probe: Vec<f64>,
    err: Vec<f64>,
    scale: ErrorScale,
}

/// `1 / (2 + √2)`, the diagonal coefficient that makes the method L-stable.
const D: f64 = 0.292_893_218_813_452_5;
/// `6 + √2`.
const E32: f64 = 7.414_213_562_373_095;
/// Order of the error estimate plus one, the exponent of step control.
const CONTROL_ORDER: f64 = 3.0;
/// Relative finite-difference step: `√ε` of the component's magnitude.
const DIFF_REL: f64 = 1.490_116_119_384_765_6e-8;
/// Floor of the finite-difference step relative to the component's own
/// value, so the perturbation survives rounding far from the origin.
const DIFF_FLOOR: f64 = 9.094_947_017_729_282e-13;

impl Rosenbrock23 {
    /// An integrator meeting `tolerances`, trying the whole interval as its
    /// first step.
    pub const fn new(tolerances: Tolerances) -> Self {
        Self {
            tolerances,
            first_step_s: None,
            max_steps: 100_000,
            jacobian: Vec::new(),
            lu: DenseLu {
                n: 0,
                a: Vec::new(),
                perm: Vec::new(),
                y: Vec::new(),
            },
            f: [Vec::new(), Vec::new(), Vec::new()],
            k: [Vec::new(), Vec::new(), Vec::new()],
            x_new: Vec::new(),
            probe: Vec::new(),
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
        for v in self.f.iter_mut().chain(&mut self.k) {
            v.resize(n, 0.0);
        }
        self.x_new.resize(n, 0.0);
        self.probe.resize(n, 0.0);
        self.err.resize(n, 0.0);
        self.jacobian.resize(n * n, 0.0);
    }

    /// Fills the Jacobian `∂f/∂x` at `(t, x)` by forward differences from
    /// `f[0]`. Each perturbed state is retracted before evaluation, so the
    /// column of a direction leaving the manifold is zero.
    fn differentiate<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t: f64,
        x: &[f64],
    ) -> Result<(), MultibodyError> {
        let n = x.len();
        let [f0, f1, _] = &mut self.f;
        for (j, (xj, magnitude)) in x.iter().zip(&self.scale.magnitude).enumerate() {
            let step = (DIFF_REL * magnitude).max(DIFF_FLOOR * xj.abs());
            self.probe.copy_from_slice(x);
            if let Some(p) = self.probe.get_mut(j) {
                *p = xj + step;
            }
            system.retract(&mut self.probe);
            system.derivative(t, &self.probe, f1)?;
            let column = self.jacobian.iter_mut().skip(j).step_by(n.max(1));
            for (slot, (a, b)) in column.zip(f1.iter().zip(f0.iter())) {
                *slot = (a - b) / step;
            }
        }
        Ok(())
    }

    /// One trial step of size `h` from `x` with `f[0] = f(t, x)` and the
    /// Jacobian in place; leaves the candidate in `x_new`, its derivative in
    /// `f[2]`, and returns the error in tolerance units.
    fn trial<S: OdeSystem + ?Sized>(
        &mut self,
        system: &mut S,
        t: f64,
        h: f64,
        x: &[f64],
    ) -> Result<f64, MultibodyError> {
        if !self.lu.factor_shifted(&self.jacobian, x.len(), h * D) {
            return Err(MultibodyError::SingularIterationMatrix { time_s: t });
        }
        let [f0, f1, f2] = &mut self.f;
        let [k1, k2, k3] = &mut self.k;
        k1.copy_from_slice(f0);
        self.lu.solve(k1);
        for (o, (xi, k)) in self.x_new.iter_mut().zip(x.iter().zip(k1.iter())) {
            *o = xi + 0.5 * h * k;
        }
        system.retract(&mut self.x_new);
        system.derivative(t + 0.5 * h, &self.x_new, f1)?;
        for (o, (f, k)) in k2.iter_mut().zip(f1.iter().zip(k1.iter())) {
            *o = f - k;
        }
        self.lu.solve(k2);
        for (o, k) in k2.iter_mut().zip(k1.iter()) {
            *o += k;
        }
        for (o, (xi, k)) in self.x_new.iter_mut().zip(x.iter().zip(k2.iter())) {
            *o = xi + h * k;
        }
        system.retract(&mut self.x_new);
        system.derivative(t + h, &self.x_new, f2)?;
        let stages = k1.iter().zip(k2.iter()).zip(f0.iter().zip(f1.iter()));
        for (o, (f2i, ((k1i, k2i), (f0i, f1i)))) in k3.iter_mut().zip(f2.iter().zip(stages)) {
            *o = f2i - E32 * (k2i - f1i) - 2.0 * (k1i - f0i);
        }
        self.lu.solve(k3);
        let stages = k1.iter().zip(k2.iter()).zip(k3.iter());
        for (e, ((a, b), c)) in self.err.iter_mut().zip(stages) {
            *e = h / 6.0 * (a - 2.0 * b + c);
        }
        Ok(self.scale.norm(&self.err, x, &self.x_new))
    }
}

impl Integrator for Rosenbrock23 {
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
        let n = x.len();
        self.fit(n);
        self.scale.fit(system, &self.tolerances);
        let (mut have_first, mut have_jacobian) = (false, false);
        let per_jacobian = u32::try_from(n).unwrap_or(u32::MAX);
        loop {
            if !have_first {
                let [f0, ..] = &mut self.f;
                system.derivative(run.t, x, f0)?;
                run.count(1);
                have_first = true;
            }
            if !have_jacobian {
                // The Jacobian depends on the state, not on the step, so a
                // rejected trial reuses it.
                self.differentiate(system, run.t, x)?;
                run.count(per_jacobian);
                have_jacobian = true;
            }
            let err = self.trial(system, run.t, run.h, x)?;
            run.count(2);
            if err.is_nan() || err > 1.0 {
                run.reject(err, CONTROL_ORDER)?;
                continue;
            }
            x.copy_from_slice(&self.x_new);
            let done = run.accept(err, CONTROL_ORDER)?;
            let event = system.accept(run.t, x)?;
            run.report.event |= event;
            // First-same-as-last, unless an event has just changed the state.
            have_first = !event;
            have_jacobian = false;
            self.f.swap(0, 2);
            if done {
                return Ok(run.report);
            }
        }
    }
}
