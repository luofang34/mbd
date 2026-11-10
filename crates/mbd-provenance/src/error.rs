/// Why a value's stated origin, or a source table, was refused.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ProvenanceError {
    /// A source id is not lower-case letters, digits, `-` and `_`.
    #[error("source id {id:?} is not a slug")]
    InvalidSourceId {
        /// The offending id.
        id: String,
    },
    /// A source record leaves a required statement empty.
    #[error("source {id}: {field} is empty")]
    EmptySourceField {
        /// The source.
        id: String,
        /// The empty field.
        field: &'static str,
    },
    /// A value cites a source the table does not hold.
    #[error("{field}: cites source {id:?}, which the source table does not hold")]
    UnknownSource {
        /// The value being checked.
        field: String,
        /// The id that resolved to nothing.
        id: String,
    },
    /// A value says it was read from a source and names none.
    #[error("{field}: stated as read from a source, but cites none")]
    NoSource {
        /// The value being checked.
        field: String,
    },
    /// Every source a value was read from is one that may only cross-check.
    #[error(
        "{field}: rests only on {id:?}, a source that may cross-check a value but not supply it"
    )]
    CrossCheckOnly {
        /// The value being checked.
        field: String,
        /// One of the cited sources.
        id: String,
    },
    /// A basis leaves out the statement that makes it checkable, such as what
    /// a fitted value was fitted to.
    #[error("{field}: a value that is {kind} must state {statement}")]
    MissingStatement {
        /// The value being checked.
        field: String,
        /// The basis kind, as [`crate::ProvenanceKind::as_str`] names it.
        kind: &'static str,
        /// What is missing.
        statement: &'static str,
    },
    /// A unit is present and empty; a dimensionless value omits the unit.
    #[error("{field}: unit is empty")]
    EmptyUnit {
        /// The value being checked.
        field: String,
    },
}
