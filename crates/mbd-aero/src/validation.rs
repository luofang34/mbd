use glam::DVec3;
use mbd_core::{AeroFlowState, MbdError};

pub(crate) fn invalid(field: &'static str, value: f64) -> MbdError {
    MbdError::InvalidAeroParameter { field, value }
}

pub(crate) fn check_positive(field: &'static str, value: f64) -> Result<(), MbdError> {
    if value.is_finite() && value > 0.0 {
        return Ok(());
    }
    Err(invalid(field, value))
}

pub(crate) fn check_nonnegative(field: &'static str, value: f64) -> Result<(), MbdError> {
    if value.is_finite() && value >= 0.0 {
        return Ok(());
    }
    Err(invalid(field, value))
}

pub(crate) fn check_finite(field: &'static str, value: f64) -> Result<(), MbdError> {
    if value.is_finite() {
        return Ok(());
    }
    Err(invalid(field, value))
}

pub(crate) fn check_finite_vec(field: &'static str, vec: DVec3) -> Result<(), MbdError> {
    if vec.is_finite() {
        return Ok(());
    }
    Err(invalid(field, vec.length()))
}

pub(crate) fn check_unit_vec(field: &'static str, vec: DVec3) -> Result<DVec3, MbdError> {
    if vec.is_finite() && (vec.length_squared() - 1.0).abs() <= 1e-6 {
        return Ok(vec.normalize());
    }
    Err(invalid(field, vec.length()))
}

pub(crate) fn control_slot(flow: &AeroFlowState, channel: Option<usize>) -> f64 {
    channel
        .and_then(|idx| flow.control_surfaces_rad.get(idx).copied())
        .unwrap_or(0.0)
}

pub(crate) fn smoothstep01(x: f64) -> f64 {
    let t = x.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
