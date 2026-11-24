use alloc::string::String;
use glam::DVec3;
use mbd_core::MbdError;

/// Errors produced by unified fluid, propulsion, electrical, thermal, and payload subsystem models.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SystemsError {
    /// Underlying tree, component, gas, or port error from `mbd-core`.
    #[error(transparent)]
    Mbd(#[from] MbdError),
    /// Invalid scalar parameter (non-finite or outside physical bounds).
    #[error("systems parameter {field} is invalid: {value}")]
    InvalidParameter {
        /// Offending field name.
        field: &'static str,
        /// Invalid numeric value.
        value: f64,
    },
    /// Invalid vector parameter (non-finite or zero magnitude when a direction is required).
    #[error("systems vector parameter {field} is invalid")]
    InvalidVector {
        /// Offending vector field name.
        field: &'static str,
    },
    /// Invalid structural or logical configuration.
    #[error("invalid systems configuration: {detail}")]
    InvalidConfig {
        /// Description of the invalid configuration.
        detail: String,
    },
    /// Duplicate identifier registered in a subsystem network.
    #[error("duplicate {kind} identifier: {id}")]
    DuplicateId {
        /// Entity kind (e.g. "tank", "thermal node", "electrical source").
        kind: &'static str,
        /// Conflicting identifier.
        id: String,
    },
    /// Lookup referenced an unknown identifier.
    #[error("unknown {kind} identifier: {id}")]
    UnknownId {
        /// Entity kind.
        kind: &'static str,
        /// Missing identifier.
        id: String,
    },
    /// Transfer line attempted to connect tanks holding incompatible species.
    #[error("fluid species mismatch transferring from {from} to {to}")]
    SpeciesMismatch {
        /// Source tank identifier.
        from: String,
        /// Destination tank identifier.
        to: String,
    },
    /// Operation violated attachment or mass conservation preconditions.
    #[error("subsystem operation rejected: {detail}")]
    Rejected {
        /// Explanation of why the operation failed.
        detail: String,
    },
}

/// Validates that `value` is finite.
pub(crate) fn check_finite(field: &'static str, value: f64) -> Result<f64, SystemsError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(SystemsError::InvalidParameter { field, value })
    }
}

/// Validates that `value` is finite and `>= 0.0`.
pub(crate) fn check_non_negative(field: &'static str, value: f64) -> Result<f64, SystemsError> {
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err(SystemsError::InvalidParameter { field, value })
    }
}

/// Validates that `value` is finite and `> 0.0`.
pub(crate) fn check_positive(field: &'static str, value: f64) -> Result<f64, SystemsError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(SystemsError::InvalidParameter { field, value })
    }
}

/// Validates that `value` is finite and within `[0.0, 1.0]`.
pub(crate) fn check_unit_interval(field: &'static str, value: f64) -> Result<f64, SystemsError> {
    if value.is_finite() && (0.0..=1.0).contains(&value) {
        Ok(value)
    } else {
        Err(SystemsError::InvalidParameter { field, value })
    }
}

/// Validates that `vec` has all finite components.
pub(crate) fn check_vec_finite(field: &'static str, vec: DVec3) -> Result<DVec3, SystemsError> {
    if vec.is_finite() {
        Ok(vec)
    } else {
        Err(SystemsError::InvalidVector { field })
    }
}

/// Validates that `vec` is finite and normalizes it, failing on zero-length vectors.
pub(crate) fn normalize_axis(field: &'static str, vec: DVec3) -> Result<DVec3, SystemsError> {
    if !vec.is_finite() {
        return Err(SystemsError::InvalidVector { field });
    }
    let norm = vec.length();
    if norm <= 1.0e-12 {
        return Err(SystemsError::InvalidVector { field });
    }
    Ok(vec / norm)
}
