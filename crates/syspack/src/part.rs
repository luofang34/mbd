//! The documents part packages carry: what an engine or a propeller is, from
//! which sources, and which of its numbers were read, fitted or estimated.
//!
//! A design tool sizes with these documents and a simulator installs the same
//! package, so both read one description. Quantities are SI and the field name
//! states the unit; what the source printed belongs in the value's note.

use std::collections::BTreeMap;

use mbd_provenance::{Basis, ProvenanceError, SourceTable, Sourced};

use crate::SyspackError;

mod engine;
#[cfg(test)]
pub(crate) mod fixtures;
mod propeller;

pub use engine::{EngineDocument, EngineRating, EngineSpec, InductionSpec, PistonEngineSpec};
pub use propeller::{ConstantSpeedSpec, PitchSpec, PropellerDocument, PropellerMapSpec};

/// Values a consumer's own model of the part needs beyond its description,
/// keyed by consumer (such as `jsbsim`) and then by name. The names are the
/// consumer's; each value still states where it comes from.
pub type ModelParameters = BTreeMap<String, BTreeMap<String, Sourced<serde_json::Value>>>;

/// The typed document of a part package.
#[derive(Debug, Clone, PartialEq)]
pub enum PartDocument {
    /// An engine's `engine.json`.
    Engine(Box<EngineDocument>),
    /// A propeller's `propeller.json`.
    Propeller(Box<PropellerDocument>),
}

impl PartDocument {
    /// The package id the document says it describes.
    pub fn id(&self) -> &str {
        match self {
            Self::Engine(document) => &document.id,
            Self::Propeller(document) => &document.id,
        }
    }
}

/// Checks one document's values against its own source table.
pub(crate) struct DocumentCheck<'a> {
    pub(crate) document: &'static str,
    pub(crate) sources: &'a SourceTable,
}

impl<'a> DocumentCheck<'a> {
    pub(crate) fn new(
        document: &'static str,
        sources: &'a SourceTable,
    ) -> Result<Self, SyspackError> {
        let check = Self { document, sources };
        sources
            .validate()
            .map_err(|source| check.provenance(source))?;
        Ok(check)
    }

    fn provenance(&self, source: ProvenanceError) -> SyspackError {
        SyspackError::PartProvenance {
            document: self.document,
            source,
        }
    }

    pub(crate) fn invalid(&self, field: &str, detail: impl Into<String>) -> SyspackError {
        SyspackError::InvalidPartDocument {
            document: self.document,
            field: field.to_owned(),
            detail: detail.into(),
        }
    }

    pub(crate) fn text(&self, field: &str, value: &str) -> Result<(), SyspackError> {
        if value.trim().is_empty() {
            return Err(self.invalid(field, "is empty"));
        }
        Ok(())
    }

    pub(crate) fn basis(&self, field: &str, basis: &Basis) -> Result<(), SyspackError> {
        self.sources
            .check(field, basis)
            .map_err(|source| self.provenance(source))
    }

    /// A value whose unit, if any, is stated beside it.
    pub(crate) fn stated<T>(&self, field: &str, value: &Sourced<T>) -> Result<(), SyspackError> {
        value
            .check(field, self.sources)
            .map_err(|source| self.provenance(source))
    }

    /// A value in the SI unit its field name fixes.
    pub(crate) fn fixed_unit<T>(
        &self,
        field: &str,
        value: &Sourced<T>,
    ) -> Result<(), SyspackError> {
        if value.unit.is_some() {
            return Err(self.invalid(
                field,
                "the field name fixes the unit; put the source's own figure in the note",
            ));
        }
        self.stated(field, value)
    }

    /// A positive, finite quantity in the SI unit its field name fixes.
    pub(crate) fn quantity(&self, field: &str, value: &Sourced<f64>) -> Result<(), SyspackError> {
        self.positive(field, value.value)?;
        self.fixed_unit(field, value)
    }

    pub(crate) fn positive(&self, field: &str, value: f64) -> Result<(), SyspackError> {
        if !(value.is_finite() && value > 0.0) {
            return Err(self.invalid(field, format!("{value} is not a positive number")));
        }
        Ok(())
    }

    /// An increasing pair of positive quantities, such as a governing range.
    pub(crate) fn range(&self, field: &str, value: &Sourced<[f64; 2]>) -> Result<(), SyspackError> {
        let [low, high] = value.value;
        self.positive(field, low)?;
        self.positive(field, high)?;
        if low >= high {
            return Err(self.invalid(field, format!("{low} to {high} is not a range")));
        }
        self.fixed_unit(field, value)
    }

    pub(crate) fn models(&self, models: &ModelParameters) -> Result<(), SyspackError> {
        for (consumer, values) in models {
            self.text("models", consumer)?;
            for (name, value) in values {
                self.text(&format!("models.{consumer}"), name)?;
                self.stated(&format!("models.{consumer}.{name}"), value)?;
            }
        }
        Ok(())
    }

    pub(crate) fn notes(&self, notes: &[String]) -> Result<(), SyspackError> {
        notes.iter().try_for_each(|note| self.text("notes", note))
    }
}
