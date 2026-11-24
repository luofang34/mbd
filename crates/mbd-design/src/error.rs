use mbd_core::MbdError;
use mbd_provenance::ProvenanceError;
use mbd_systems::SystemsError;
use syspack::SyspackError;

/// Errors produced by forward sizing, trim/power/thermal analysis, reverse parameter
/// identification, or syspack design contract auditing.
#[derive(Debug, thiserror::Error)]
pub enum DesignError {
    /// Core component-tree or inertial property error.
    #[error(transparent)]
    Mbd(#[from] MbdError),
    /// Physical subsystem modeling or conservation error.
    #[error(transparent)]
    Systems(#[from] SystemsError),
    /// Package loading or verification error.
    #[error(transparent)]
    Syspack(#[from] SyspackError),
    /// Evidence or fitted provenance validation error.
    #[error(transparent)]
    Provenance(#[from] ProvenanceError),
    /// Invalid scalar input parameter.
    #[error("design parameter {field} is invalid: {value}")]
    InvalidParameter {
        /// Offending field name.
        field: &'static str,
        /// Invalid numeric value.
        value: f64,
    },
    /// Invalid structural or scenario configuration.
    #[error("invalid design configuration: {detail}")]
    InvalidConfig {
        /// Reason the configuration was rejected.
        detail: String,
    },
    /// Inverse parameter identification solver failure.
    #[error("parameter calibration failed: {detail}")]
    OptimizationFailed {
        /// Solver failure detail.
        detail: String,
    },
}

pub(crate) fn check_positive(field: &'static str, value: f64) -> Result<f64, DesignError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(DesignError::InvalidParameter { field, value })
    }
}

pub(crate) fn check_non_negative(field: &'static str, value: f64) -> Result<f64, DesignError> {
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err(DesignError::InvalidParameter { field, value })
    }
}

pub(crate) fn check_finite(field: &'static str, value: f64) -> Result<f64, DesignError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(DesignError::InvalidParameter { field, value })
    }
}
