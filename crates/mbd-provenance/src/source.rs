use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{Basis, ProvenanceError};

#[cfg(test)]
mod tests;

/// What kind of document a source is, which decides whether it may supply a
/// value or only cross-check one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceTier {
    /// An official flight or bench test report, with its measured conditions.
    TestReport,
    /// A flight or operating manual, technical order or handbook.
    OperatingManual,
    /// A manufacturer's specification, type certificate data sheet or parts
    /// record.
    ManufacturerSpecification,
    /// A research report, such as a wind-tunnel or propeller test series.
    ResearchReport,
    /// An engineering drawing.
    Drawing,
    /// A dated photograph of the article.
    Photograph,
    /// A book, article or encyclopaedia reporting primary material.
    Secondary,
    /// Game, simulator or hobby-community data: a hint to cross-check
    /// against, never the origin of a value.
    Community,
}

impl SourceTier {
    /// Whether a value may be read from a source of this tier.
    pub fn supplies_values(self) -> bool {
        !matches!(self, Self::Community)
    }
}

/// One cited document.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceRecord {
    /// Title, issuing body, report number and date: enough to find it again.
    pub document: String,
    /// What kind of document it is.
    pub tier: SourceTier,
    /// Where it was read, when it is online.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    /// Pages, figures or tables the values were read from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pages: Option<String>,
}

/// The documents a set of values may cite, by id.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SourceTable {
    sources: BTreeMap<String, SourceRecord>,
}

impl SourceTable {
    /// Builds a table, refusing an id that is not a slug or a record without a
    /// document.
    pub fn new(sources: BTreeMap<String, SourceRecord>) -> Result<Self, ProvenanceError> {
        let table = Self { sources };
        table.validate()?;
        Ok(table)
    }

    /// Checks every id and record. A deserialized table is unchecked until
    /// this has run.
    pub fn validate(&self) -> Result<(), ProvenanceError> {
        for (id, record) in &self.sources {
            if !is_slug(id) {
                return Err(ProvenanceError::InvalidSourceId { id: id.clone() });
            }
            let empty = |field| ProvenanceError::EmptySourceField {
                id: id.clone(),
                field,
            };
            if record.document.trim().is_empty() {
                return Err(empty("document"));
            }
            if record.url.as_deref().is_some_and(|u| u.trim().is_empty()) {
                return Err(empty("url"));
            }
            if record.pages.as_deref().is_some_and(|p| p.trim().is_empty()) {
                return Err(empty("pages"));
            }
        }
        Ok(())
    }

    /// The record with this id.
    pub fn get(&self, id: &str) -> Option<&SourceRecord> {
        self.sources.get(id)
    }

    /// Every source, in id order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &SourceRecord)> {
        self.sources
            .iter()
            .map(|(id, record)| (id.as_str(), record))
    }

    /// Checks the basis of the value named `field` against this table: every
    /// citation resolves, a value read from sources cites one that may supply
    /// values, and every other basis carries its statement.
    pub fn check(&self, field: &str, basis: &Basis) -> Result<(), ProvenanceError> {
        for id in basis.cited() {
            if !self.sources.contains_key(id) {
                return Err(ProvenanceError::UnknownSource {
                    field: field.to_owned(),
                    id: id.clone(),
                });
            }
        }
        match basis {
            Basis::Sourced(ids) => self.check_read_from(field, ids),
            Basis::Derived { method, .. } => stated(field, basis, "its method", method),
            Basis::Fitted { to } => stated(field, basis, "what it was fitted to", to),
            Basis::Estimated { method } => stated(field, basis, "its method", method),
            Basis::Assumed { reason } => stated(field, basis, "its reason", reason),
        }
    }

    fn check_read_from(&self, field: &str, ids: &[String]) -> Result<(), ProvenanceError> {
        let Some(first) = ids.first() else {
            return Err(ProvenanceError::NoSource {
                field: field.to_owned(),
            });
        };
        let supplied = ids
            .iter()
            .filter_map(|id| self.sources.get(id))
            .any(|record| record.tier.supplies_values());
        if supplied {
            Ok(())
        } else {
            Err(ProvenanceError::CrossCheckOnly {
                field: field.to_owned(),
                id: first.clone(),
            })
        }
    }
}

fn stated(
    field: &str,
    basis: &Basis,
    statement: &'static str,
    text: &str,
) -> Result<(), ProvenanceError> {
    if text.trim().is_empty() {
        return Err(ProvenanceError::MissingStatement {
            field: field.to_owned(),
            kind: basis.kind().as_str(),
            statement,
        });
    }
    Ok(())
}

fn is_slug(id: &str) -> bool {
    !id.is_empty()
        && id
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-' || c == '_')
}
