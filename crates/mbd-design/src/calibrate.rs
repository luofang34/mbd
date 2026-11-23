use mbd_provenance::{Basis, SourceTable, Sourced};
use serde::{Deserialize, Serialize};

use crate::error::{DesignError, check_finite, check_positive};

#[cfg(test)]
mod tests;

const MAX_PARAMS: usize = 32;

/// Parameter specification for reverse-MBD bounded parameter identification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationParamSpec {
    /// Parameter name used in report and `Sourced::check`.
    pub name: String,
    /// Optional physical unit (e.g. `"m^2"`, `"rad^-1"`).
    pub unit: Option<String>,
    /// Initial guess inside `[min, max]`.
    pub initial: f64,
    /// Inclusive lower bound.
    pub min: f64,
    /// Inclusive upper bound.
    pub max: f64,
    /// Central finite-difference perturbation step magnitude.
    pub fd_step: f64,
}

/// One empirical data sample `(inputs, target)` weighted by `weight > 0`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationObservation {
    /// Independent input variables evaluated by the forward model.
    pub inputs: Vec<f64>,
    /// Measured physical target output.
    pub target: f64,
    /// Positive observation weight (typically `1.0 / sigma_target`).
    pub weight: f64,
}

/// Reverse-MBD identification output tagging every fitted coefficient as [`Basis::Fitted`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationReport {
    /// Fitted parameter `(name, Sourced<f64>)` pairs verified against the dataset provenance.
    pub fitted_params: Vec<(String, Sourced<f64>)>,
    /// Root-mean-square unweighted prediction error across observations.
    pub rmse: f64,
    /// Maximum absolute prediction error across observations.
    pub max_abs_error: f64,
    /// Total Levenberg-Marquardt iterations executed.
    pub iterations: usize,
    /// Whether parameter step or SSE improvement converged within `tol`.
    pub converged: bool,
}

/// Bounded Levenberg-Marquardt least-squares optimizer recovering unknown physical model
/// parameters from experimental observations and tagging them with [`Basis::Fitted`].
pub fn fit_parameters_lm(
    params: &[CalibrationParamSpec],
    observations: &[CalibrationObservation],
    model: impl Fn(&[f64], &[f64]) -> Result<f64, DesignError>,
    max_iters: usize,
    tol: f64,
    dataset_label: &str,
    sources: &SourceTable,
) -> Result<CalibrationReport, DesignError> {
    validate_calibration_inputs(params, observations, tol, dataset_label)?;
    sources.validate()?;

    let mut theta: Vec<f64> = params.iter().map(|p| p.initial).collect();
    let mut lambda = 1.0e-3_f64;
    let mut res = eval_residuals(&theta, observations, &model)?;
    let mut sse = weighted_sse(&res);
    let mut iterations = 0usize;
    let mut converged = sse <= tol * tol;

    for _ in 0..max_iters {
        if converged {
            break;
        }
        iterations = iterations.wrapping_add(1);
        let (mut jtj, mut rhs) = build_normal_system(&theta, &res, params, observations, &model)?;
        for d in 0..params.len() {
            let idx = d * params.len() + d;
            if let Some(diag) = jtj.get_mut(idx) {
                *diag += lambda * (*diag).max(1.0);
            }
        }
        let Some(delta) = solve_linear_system(params.len(), &mut jtj, &mut rhs) else {
            lambda *= 10.0;
            continue;
        };
        let trial = apply_bounded_step(&theta, &delta, params);
        let step_norm = delta.iter().map(|d| d * d).sum::<f64>().sqrt();
        let trial_res = eval_residuals(&trial, observations, &model)?;
        let trial_sse = weighted_sse(&trial_res);
        if trial_sse < sse {
            let rel = (sse - trial_sse) / sse.max(1.0);
            theta = trial;
            res = trial_res;
            sse = trial_sse;
            lambda = (lambda * 0.3).max(1.0e-12);
            if step_norm <= tol || rel <= tol * tol {
                converged = true;
            }
        } else {
            lambda *= 5.0;
            if step_norm <= tol * 1.0e-2 || lambda > 1.0e10 {
                converged = step_norm <= tol;
                break;
            }
        }
    }

    build_report(
        params,
        &theta,
        observations,
        &model,
        dataset_label,
        sources,
        iterations,
        converged,
    )
}

fn validate_calibration_inputs(
    params: &[CalibrationParamSpec],
    observations: &[CalibrationObservation],
    tol: f64,
    dataset_label: &str,
) -> Result<(), DesignError> {
    if params.is_empty() || params.len() > MAX_PARAMS {
        return Err(DesignError::InvalidConfig {
            detail: format!("params count {} must be in 1..={MAX_PARAMS}", params.len()),
        });
    }
    if observations.is_empty() {
        return Err(DesignError::InvalidConfig {
            detail: "calibration requires at least one observation".into(),
        });
    }
    if dataset_label.trim().is_empty() {
        return Err(DesignError::InvalidConfig {
            detail: "dataset_label must not be empty".into(),
        });
    }
    check_positive("calibrate.tol", tol)?;
    for p in params {
        if p.name.trim().is_empty() {
            return Err(DesignError::InvalidConfig {
                detail: "parameter name must not be empty".into(),
            });
        }
        check_finite("calibrate.min", p.min)?;
        check_finite("calibrate.max", p.max)?;
        check_finite("calibrate.initial", p.initial)?;
        check_positive("calibrate.fd_step", p.fd_step)?;
        if p.min > p.max || p.initial < p.min || p.initial > p.max {
            return Err(DesignError::InvalidConfig {
                detail: format!("parameter {:?} violates min <= initial <= max", p.name),
            });
        }
    }
    for obs in observations {
        check_finite("calibrate.target", obs.target)?;
        check_positive("calibrate.weight", obs.weight)?;
        if !obs.inputs.iter().all(|v| v.is_finite()) {
            return Err(DesignError::InvalidConfig {
                detail: "observation contains non-finite input".into(),
            });
        }
    }
    Ok(())
}

fn eval_residuals(
    theta: &[f64],
    observations: &[CalibrationObservation],
    model: &impl Fn(&[f64], &[f64]) -> Result<f64, DesignError>,
) -> Result<Vec<f64>, DesignError> {
    let mut res = Vec::with_capacity(observations.len());
    for obs in observations {
        let pred = model(&obs.inputs, theta)?;
        check_finite("calibrate.model_prediction", pred)?;
        res.push(obs.weight * (pred - obs.target));
    }
    Ok(res)
}

fn weighted_sse(res: &[f64]) -> f64 {
    res.iter().map(|r| r * r).sum()
}

fn build_normal_system(
    theta: &[f64],
    base_res: &[f64],
    params: &[CalibrationParamSpec],
    observations: &[CalibrationObservation],
    model: &impl Fn(&[f64], &[f64]) -> Result<f64, DesignError>,
) -> Result<(Vec<f64>, Vec<f64>), DesignError> {
    let np = params.len();
    let m = observations.len();
    let mut jac = vec![0.0_f64; m * np];
    let mut theta_p = theta.to_vec();
    let mut theta_m = theta.to_vec();

    for (j, spec) in params.iter().enumerate() {
        let orig = theta.get(j).copied().unwrap_or(0.0);
        let hi = (orig + spec.fd_step).min(spec.max);
        let lo = (orig - spec.fd_step).max(spec.min);
        let span = hi - lo;
        if span <= 0.0 {
            continue;
        }
        if let Some(slot) = theta_p.get_mut(j) {
            *slot = hi;
        }
        if let Some(slot) = theta_m.get_mut(j) {
            *slot = lo;
        }
        let rp = eval_residuals(&theta_p, observations, model)?;
        let rm = eval_residuals(&theta_m, observations, model)?;
        for i in 0..m {
            let val =
                (rp.get(i).copied().unwrap_or(0.0) - rm.get(i).copied().unwrap_or(0.0)) / span;
            if let Some(cell) = jac.get_mut(i * np + j) {
                *cell = val;
            }
        }
        if let Some(slot) = theta_p.get_mut(j) {
            *slot = orig;
        }
        if let Some(slot) = theta_m.get_mut(j) {
            *slot = orig;
        }
    }

    let mut jtj = vec![0.0_f64; np * np];
    let mut rhs = vec![0.0_f64; np];
    for i in 0..m {
        let r_i = base_res.get(i).copied().unwrap_or(0.0);
        for a in 0..np {
            let j_ia = jac.get(i * np + a).copied().unwrap_or(0.0);
            if let Some(slot) = rhs.get_mut(a) {
                *slot -= j_ia * r_i;
            }
            for b in 0..np {
                let j_ib = jac.get(i * np + b).copied().unwrap_or(0.0);
                if let Some(cell) = jtj.get_mut(a * np + b) {
                    *cell += j_ia * j_ib;
                }
            }
        }
    }
    Ok((jtj, rhs))
}

fn apply_bounded_step(theta: &[f64], delta: &[f64], params: &[CalibrationParamSpec]) -> Vec<f64> {
    theta
        .iter()
        .zip(delta.iter())
        .zip(params.iter())
        .map(|((&t, &d), p)| (t + d).clamp(p.min, p.max))
        .collect()
}

fn solve_linear_system(n: usize, a: &mut [f64], b: &mut [f64]) -> Option<Vec<f64>> {
    for col in 0..n {
        let mut pivot_row = col;
        let mut pivot_abs = a.get(col * n + col).copied().unwrap_or(0.0).abs();
        for row in col.wrapping_add(1)..n {
            let cand = a.get(row * n + col).copied().unwrap_or(0.0).abs();
            if cand > pivot_abs {
                pivot_abs = cand;
                pivot_row = row;
            }
        }
        if pivot_abs <= 1.0e-14 {
            return None;
        }
        if pivot_row != col {
            for k in 0..n {
                a.swap(col * n + k, pivot_row * n + k);
            }
            b.swap(col, pivot_row);
        }
        let diag = a.get(col * n + col).copied().unwrap_or(1.0);
        let b_col = b.get(col).copied().unwrap_or(0.0);
        for row in col.wrapping_add(1)..n {
            let factor = a.get(row * n + col).copied().unwrap_or(0.0) / diag;
            if let Some(slot) = b.get_mut(row) {
                *slot -= factor * b_col;
            }
            for k in col..n {
                let a_ck = a.get(col * n + k).copied().unwrap_or(0.0);
                if let Some(cell) = a.get_mut(row * n + k) {
                    *cell -= factor * a_ck;
                }
            }
        }
    }

    let mut x = vec![0.0_f64; n];
    for i in (0..n).rev() {
        let mut sum = b.get(i).copied().unwrap_or(0.0);
        for j in i.wrapping_add(1)..n {
            let a_ij = a.get(i * n + j).copied().unwrap_or(0.0);
            let x_j = x.get(j).copied().unwrap_or(0.0);
            sum -= a_ij * x_j;
        }
        let diag = a.get(i * n + i).copied().unwrap_or(0.0);
        if diag.abs() <= 1.0e-14 {
            return None;
        }
        if let Some(slot) = x.get_mut(i) {
            *slot = sum / diag;
        }
    }
    Some(x)
}

#[allow(clippy::too_many_arguments)]
fn build_report(
    params: &[CalibrationParamSpec],
    theta: &[f64],
    observations: &[CalibrationObservation],
    model: &impl Fn(&[f64], &[f64]) -> Result<f64, DesignError>,
    dataset_label: &str,
    sources: &SourceTable,
    iterations: usize,
    converged: bool,
) -> Result<CalibrationReport, DesignError> {
    let mut sq_sum = 0.0;
    let mut max_abs_error = 0.0_f64;
    for obs in observations {
        let pred = model(&obs.inputs, theta)?;
        let err = (pred - obs.target).abs();
        sq_sum += err * err;
        max_abs_error = max_abs_error.max(err);
    }
    let rmse = (sq_sum / observations.len() as f64).sqrt();

    let mut fitted_params = Vec::with_capacity(params.len());
    for (idx, spec) in params.iter().enumerate() {
        let value = theta.get(idx).copied().unwrap_or(spec.initial);
        let sourced = Sourced {
            value,
            unit: spec.unit.clone(),
            basis: Basis::Fitted {
                to: dataset_label.to_owned(),
            },
            note: Some(format!("LM calibration RMSE={rmse:.6e}")),
        };
        sourced.check(&spec.name, sources)?;
        fitted_params.push((spec.name.clone(), sourced));
    }

    Ok(CalibrationReport {
        fitted_params,
        rmse,
        max_abs_error,
        iterations,
        converged,
    })
}
