//! How an instrument's dial is lettered: which pinned typeface its numerals
//! and legends are drawn in, and in what style.

use serde::{Deserialize, Serialize};

use super::{InstrumentPackDocument, InstrumentSpec};
use crate::SyspackError;

#[cfg(test)]
mod tests;

/// The faces a dial is lettered in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LetteringSpec {
    /// The scale's numerals.
    pub numerals: FaceStyleSpec,
    /// The printed legends; the numerals' face and style when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub legends: Option<FaceStyleSpec>,
}

/// One typeface, pinned by the pack, in one style.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FaceStyleSpec {
    /// The id of a typeface the pack's `typefaces` pins.
    pub face: String,
    /// Horizontal scale at constant pen, `0.25..=4`; 1 when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub condense: Option<f64>,
    /// Pen width as a fraction of the cap height, `(0, 0.5]`; the face's
    /// own when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pen: Option<f64>,
    /// Shear per unit of height, `-1..=1`, positive leaning right.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub slant: Option<f64>,
    /// Glyph alternates enabled by name, such as `one.flag`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub alternates: Vec<String>,
}

fn invalid(spec: &InstrumentSpec, detail: &str) -> SyspackError {
    SyspackError::InvalidInstrument {
        instrument: spec.id.clone(),
        field: "lettering",
        detail: detail.to_owned(),
    }
}

impl FaceStyleSpec {
    fn validate(
        &self,
        spec: &InstrumentSpec,
        document: &InstrumentPackDocument,
    ) -> Result<(), SyspackError> {
        if !document.typefaces.iter().any(|pin| pin.id == self.face) {
            return Err(invalid(
                spec,
                "lettering names a typeface the pack does not pin",
            ));
        }
        let within = |v: Option<f64>, ok: fn(f64) -> bool| v.is_none_or(|v| v.is_finite() && ok(v));
        if !within(self.condense, |v| (0.25..=4.0).contains(&v)) {
            return Err(invalid(spec, "condense must lie in 0.25..=4"));
        }
        if !within(self.pen, |v| v > 0.0 && v <= 0.5) {
            return Err(invalid(spec, "pen must lie in (0, 0.5]"));
        }
        if !within(self.slant, |v| (-1.0..=1.0).contains(&v)) {
            return Err(invalid(spec, "slant must lie in -1..=1"));
        }
        if self
            .alternates
            .iter()
            .any(|a| a.is_empty() || !a.is_ascii())
        {
            return Err(invalid(spec, "an alternate needs an ASCII name"));
        }
        Ok(())
    }
}

impl LetteringSpec {
    /// Checks every face is pinned and every style in range.
    pub(super) fn validate(
        &self,
        spec: &InstrumentSpec,
        document: &InstrumentPackDocument,
    ) -> Result<(), SyspackError> {
        self.numerals.validate(spec, document)?;
        self.legends
            .as_ref()
            .map_or(Ok(()), |legends| legends.validate(spec, document))
    }

    /// The legends' face and style.
    #[must_use]
    pub fn legends(&self) -> &FaceStyleSpec {
        self.legends.as_ref().unwrap_or(&self.numerals)
    }
}
