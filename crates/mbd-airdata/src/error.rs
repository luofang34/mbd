/// Why an air-data table or instrument configuration was rejected. Every
/// check runs at construction, so a mechanism that exists always steps.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum AirdataError {
    /// A value that must be a finite number is NaN or infinite.
    #[error("{field} must be finite")]
    NonFinite {
        /// The offending field.
        field: &'static str,
    },
    /// A value is outside the range the model is defined on.
    #[error("{field} is {value}, outside {min}..={max}")]
    OutOfRange {
        /// The offending field.
        field: &'static str,
        /// The value given.
        value: f64,
        /// Smallest accepted value.
        min: f64,
        /// Largest accepted value.
        max: f64,
    },
    /// A table has fewer points than it needs to define a mapping.
    #[error("{field} has {count} points; at least {min} are needed")]
    TooFewPoints {
        /// The table.
        field: &'static str,
        /// Points given.
        count: usize,
        /// Points needed.
        min: usize,
    },
    /// A table has more points than its fixed capacity.
    #[error("{field} has {count} points; the capacity is {capacity}")]
    TooManyPoints {
        /// The table.
        field: &'static str,
        /// Points given.
        count: usize,
        /// Points the table holds.
        capacity: usize,
    },
    /// A table's abscissae, or a dial's angles, do not strictly increase.
    #[error("{field} is not strictly increasing at point {index}")]
    NotIncreasing {
        /// The table.
        field: &'static str,
        /// Index of the first point that does not exceed its predecessor.
        index: usize,
    },
    /// A dial unit does not measure the instrument's quantity.
    #[error("{instrument} cannot display {unit}")]
    UnitMismatch {
        /// The instrument kind.
        instrument: &'static str,
        /// The unit's name.
        unit: &'static str,
    },
}

/// Fails with [`AirdataError::NonFinite`] unless `value` is finite.
pub(crate) fn finite(field: &'static str, value: f64) -> Result<f64, AirdataError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(AirdataError::NonFinite { field })
    }
}

/// Fails unless `value` is finite and within `min..=max`.
pub(crate) fn in_range(
    field: &'static str,
    value: f64,
    min: f64,
    max: f64,
) -> Result<f64, AirdataError> {
    finite(field, value)?;
    if value < min || value > max {
        return Err(AirdataError::OutOfRange {
            field,
            value,
            min,
            max,
        });
    }
    Ok(value)
}
