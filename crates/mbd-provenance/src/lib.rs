//! Where a design value comes from.
//!
//! A model built from documents mixes numbers read from a source, numbers
//! computed from those, numbers tuned until the model reproduced a test, and
//! numbers nobody could find. A reader who cannot tell them apart will cite a
//! tuned value as evidence. Here every value carries its [`Basis`], and a
//! [`SourceTable`] holds the documents a basis may cite.
//!
//! Checking fails closed: a citation that resolves to no source, a value read
//! from nothing, a value resting only on a source that may not back numbers,
//! or a fit that does not say what it was fitted to is a typed
//! [`ProvenanceError`].
//!
//! The crate knows no vehicle, simulator or design tool; documents that hold
//! [`Sourced`] values decide when to check them.

mod basis;
mod error;
mod source;
mod sourced;

pub use basis::{Basis, ProvenanceKind};
pub use error::ProvenanceError;
pub use source::{SourceRecord, SourceTable, SourceTier};
pub use sourced::Sourced;
