//! Part documents the tests load: one engine and one propeller, written
//! from published sources.

/// A two-speed supercharged radial with four stated ratings.
pub(crate) const KASEI_11_ENGINE: &str = include_str!("fixtures/mitsubishi-kasei-11.engine.json");

/// A constant-speed propeller whose geometry is mostly estimated.
pub(crate) const HAMILTON_PROPELLER: &str =
    include_str!("fixtures/hamilton-standard-cs-134in-3b.propeller.json");
