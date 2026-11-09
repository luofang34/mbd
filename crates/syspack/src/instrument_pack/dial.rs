//! How an instrument's dial face is printed: what a renderer needs beyond the
//! scale its mechanism reads. The mechanism's `scale` says which value sits
//! at which needle angle from the dial's reference mark; this says where that
//! mark sits on the face, how the face is graduated and lettered, and how the
//! pointers look. Every field is optional, and a renderer supplies its own
//! house style for whatever a pack leaves out.

use serde::{Deserialize, Serialize};

use super::InstrumentSpec;
use crate::SyspackError;

/// The printed face of one dial. Positions and sizes are fractions of the
/// face radius, `x` right and `y` down from the face centre as the pilot sees
/// it; angles are degrees clockwise from twelve o'clock.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DialFaceSpec {
    /// Where the scale's reference mark (needle angle zero) sits on the face.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reference_deg: Option<f64>,
    /// Diameter of the visible face inside the bezel, m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face_diameter_m: Option<f64>,
    /// Centre of this scale and its pointer's pivot `[x, y]`, fractions of
    /// the face radius: off centre for one of several sub-dials sharing a
    /// face. The face centre when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ring_centre: Option<[f64; 2]>,
    /// Radius of this scale, fraction of the face radius; the whole face
    /// when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ring_radius: Option<f64>,
    /// Runs of evenly spaced graduations, in dial units. An altimeter's are in
    /// units of its long hand, one turn per 1 000.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub graduations: Vec<GraduationSpec>,
    /// Printed numbers on the scale.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub numerals: Vec<NumeralSpec>,
    /// Printed words, such as `AIRSPEED` and `MPH X100`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub legends: Vec<LegendSpec>,
    /// Whether graduations, numerals and pointers carry luminous paint, which
    /// glows under the cockpit's ultraviolet lamps.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub luminous: Option<bool>,
    /// Pointer shapes in hand order: the long hand first, then an altimeter's
    /// thousands hand and ten-thousands pointer.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pointers: Vec<PointerFaceSpec>,
    /// Where the pointers go when the instrument has no input.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub on_failure: Option<FailureSpec>,
}

/// A run of evenly spaced graduations.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraduationSpec {
    /// First graduated value, dial units.
    pub from: f64,
    /// Last graduated value, dial units; at least `from`.
    pub to: f64,
    /// Spacing, dial units.
    pub every: f64,
    /// Whether these are the long major marks rather than short minor ones.
    #[serde(default)]
    pub major: bool,
}

/// A printed number. The text is what the face prints, which is often not the
/// value: `2` at 200 mph on a dial marked in hundreds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NumeralSpec {
    /// Where it sits, dial units.
    pub value: f64,
    /// What is printed.
    pub text: String,
}

/// A printed word or phrase.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LegendSpec {
    /// What is printed.
    pub text: String,
    /// Its centre `[x, y]`, fractions of the face radius.
    pub at: [f64; 2],
    /// Its height, fraction of the face radius.
    pub size: f64,
}

/// The silhouette family of a pointer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PointerShapeSpec {
    /// A long tapered needle.
    Lance,
    /// A short broad hand, such as an altimeter's thousands hand.
    Sword,
    /// A straight bar with a square end.
    Baton,
    /// A thin shaft carrying a triangle near the rim.
    Triangle,
}

/// How one pointer looks, in fractions of the face radius.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PointerFaceSpec {
    /// Silhouette.
    pub shape: PointerShapeSpec,
    /// Pivot to tip.
    pub length: f64,
    /// Widest part.
    pub width: f64,
    /// Counterweight tail behind the pivot.
    #[serde(default)]
    pub tail: f64,
}

/// What the pointers do when the instrument has no input.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum FailureSpec {
    /// Rest at this face angle, degrees clockwise from twelve o'clock.
    Park {
        /// The resting angle.
        angle_deg: f64,
    },
    /// Not drawn.
    Hide,
}

/// The most pointers a dial face describes: the altimeter's three hands and a
/// spare.
const MAX_POINTERS: usize = 4;

fn invalid(spec: &InstrumentSpec, detail: &str) -> SyspackError {
    SyspackError::InvalidInstrument {
        instrument: spec.id.clone(),
        field: "dial",
        detail: detail.to_owned(),
    }
}

/// Printable ASCII with no leading or trailing space: what a dial's glyph set
/// can letter.
fn printable(text: &str) -> bool {
    !text.is_empty()
        && text.trim() == text
        && text.chars().all(|c| c.is_ascii_graphic() || c == ' ')
}

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

impl DialFaceSpec {
    /// Checks the face against its instrument, failing on the first problem.
    pub(super) fn validate(&self, spec: &InstrumentSpec) -> Result<(), SyspackError> {
        if self.reference_deg.is_some_and(|deg| !deg.is_finite()) {
            return Err(invalid(spec, "reference_deg must be finite"));
        }
        if self
            .ring_centre
            .is_some_and(|c| !c.iter().all(|v| v.is_finite() && v.abs() <= 1.0))
        {
            return Err(invalid(spec, "ring_centre must lie on the face"));
        }
        if self.ring_radius.is_some_and(|r| !(positive(r) && r <= 1.0)) {
            return Err(invalid(spec, "ring_radius must be positive and at most 1"));
        }
        if let Some(diameter) = self.face_diameter_m
            && !(positive(diameter) && diameter <= spec.bezel.diameter_m())
        {
            return Err(invalid(
                spec,
                "face_diameter_m must be positive and within the bezel",
            ));
        }
        for g in &self.graduations {
            let ok = g.from.is_finite() && g.to.is_finite() && g.from <= g.to && positive(g.every);
            if !ok {
                return Err(invalid(
                    spec,
                    "a graduation run needs finite from <= to and every > 0",
                ));
            }
        }
        for numeral in &self.numerals {
            if !(numeral.value.is_finite() && printable(&numeral.text)) {
                return Err(invalid(
                    spec,
                    "a numeral needs a finite value and printable text",
                ));
            }
        }
        for legend in &self.legends {
            let placed = legend.at.iter().all(|v| v.is_finite()) && positive(legend.size);
            if !(placed && printable(&legend.text)) {
                return Err(invalid(
                    spec,
                    "a legend needs printable text, a finite place and a size",
                ));
            }
        }
        if self.pointers.len() > MAX_POINTERS {
            return Err(invalid(spec, "a dial describes at most four pointers"));
        }
        for pointer in &self.pointers {
            let ok = positive(pointer.length)
                && positive(pointer.width)
                && pointer.tail.is_finite()
                && pointer.tail >= 0.0;
            if !ok {
                return Err(invalid(spec, "a pointer needs a positive length and width"));
            }
        }
        if let Some(FailureSpec::Park { angle_deg }) = self.on_failure
            && !angle_deg.is_finite()
        {
            return Err(invalid(spec, "a parked pointer needs a finite angle"));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
