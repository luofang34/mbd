use serde::{Deserialize, Serialize};

use crate::{Basis, ProvenanceError, ProvenanceKind, SourceTable};

#[cfg(test)]
mod tests;

/// A value with its unit and how it was arrived at.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sourced<T> {
    /// The value, in `unit`.
    pub value: T,
    /// The unit the value is in; omitted for a dimensionless value, one that
    /// is not a quantity, or one whose unit the holding document fixes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    /// How the value was arrived at.
    pub basis: Basis,
    /// Conditions, conflicts between sources, or what the value stands in
    /// for.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

impl<T> Sourced<T> {
    /// The kind of the value's basis.
    pub fn kind(&self) -> ProvenanceKind {
        self.basis.kind()
    }

    /// Checks the value named `field` against the sources it may cite; see
    /// [`SourceTable::check`].
    pub fn check(&self, field: &str, sources: &SourceTable) -> Result<(), ProvenanceError> {
        if self.unit.as_deref().is_some_and(|u| u.trim().is_empty()) {
            return Err(ProvenanceError::EmptyUnit {
                field: field.to_owned(),
            });
        }
        sources.check(field, &self.basis)
    }
}
