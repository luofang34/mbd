use serde::{Deserialize, Serialize};

#[cfg(test)]
mod tests;

/// How a value was arrived at.
///
/// The variants differ in what a passing test built on the value proves: a
/// model that reproduces a test it was [`Basis::Fitted`] to proves nothing
/// about that test.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Basis {
    /// Read from the cited sources, which state the value itself.
    Sourced(Vec<String>),
    /// Computed from other values by a stated method.
    Derived {
        /// Sources of the values it was computed from; none when those values
        /// are themselves not read from a source.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        from: Vec<String>,
        /// The calculation, precise enough to repeat.
        method: String,
    },
    /// Tuned until the model reproduced the named data.
    Fitted {
        /// The data it was fitted to, such as a report's table or a test
        /// card's rows.
        to: String,
    },
    /// An engineering estimate by a stated method, with no source for the
    /// value itself.
    Estimated {
        /// How it was estimated.
        method: String,
    },
    /// Chosen without evidence because the model needs a value.
    Assumed {
        /// Why this value.
        reason: String,
    },
}

impl Basis {
    /// The kind of basis, without its statement.
    pub fn kind(&self) -> ProvenanceKind {
        match self {
            Self::Sourced(_) => ProvenanceKind::Sourced,
            Self::Derived { .. } => ProvenanceKind::Derived,
            Self::Fitted { .. } => ProvenanceKind::Fitted,
            Self::Estimated { .. } => ProvenanceKind::Estimated,
            Self::Assumed { .. } => ProvenanceKind::Assumed,
        }
    }

    /// The source ids the basis cites.
    pub fn cited(&self) -> &[String] {
        match self {
            Self::Sourced(ids) | Self::Derived { from: ids, .. } => ids,
            Self::Fitted { .. } | Self::Estimated { .. } | Self::Assumed { .. } => &[],
        }
    }
}

/// The kinds of [`Basis`], for tallies and for mapping onto another tool's
/// labels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProvenanceKind {
    /// See [`Basis::Sourced`].
    Sourced,
    /// See [`Basis::Derived`].
    Derived,
    /// See [`Basis::Fitted`].
    Fitted,
    /// See [`Basis::Estimated`].
    Estimated,
    /// See [`Basis::Assumed`].
    Assumed,
}

impl ProvenanceKind {
    /// The kind's name as it is written in a document.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Sourced => "sourced",
            Self::Derived => "derived",
            Self::Fitted => "fitted",
            Self::Estimated => "estimated",
            Self::Assumed => "assumed",
        }
    }

    /// Whether a test that passes on such a value is evidence about the
    /// aircraft: false for a value tuned to a test or chosen without evidence.
    pub fn is_evidence(self) -> bool {
        matches!(self, Self::Sourced | Self::Derived)
    }
}
