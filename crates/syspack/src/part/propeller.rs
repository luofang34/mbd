use mbd_provenance::{Basis, SourceTable, Sourced};
use serde::{Deserialize, Serialize};

use super::{DocumentCheck, ModelParameters};
use crate::{PROPELLER_DOCUMENT_ASSET, SyspackError};

#[cfg(test)]
mod tests;

/// `propeller.json`: a propeller as its sources describe it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerDocument {
    /// The id of the part package the document belongs to.
    pub id: String,
    /// Maker, type and size.
    pub name: String,
    /// The documents the values below cite.
    pub sources: SourceTable,
    /// Number of blades.
    pub blades: Sourced<u32>,
    /// Diameter, m.
    pub diameter_m: Sourced<f64>,
    /// How blade pitch is set.
    pub pitch: PitchSpec,
    /// Propeller speed over crankshaft speed of the installation the part's
    /// tables are built for; omitted for direct drive.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reduction_gear: Option<Sourced<f64>>,
    /// Where the thrust and power coefficient maps come from.
    pub map: PropellerMapSpec,
    /// Values a consumer's own model needs that no source gives.
    #[serde(default, skip_serializing_if = "ModelParameters::is_empty")]
    pub models: ModelParameters,
    /// Conflicts between sources, and what was looked for and not found.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
}

/// How a propeller's blade pitch is set.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum PitchSpec {
    /// Blades fixed to the hub.
    Fixed {
        /// Geometric pitch, m of advance per revolution.
        geometric_pitch_m: Sourced<f64>,
    },
    /// A governor holds a selected speed by changing blade angle.
    ConstantSpeed(Box<ConstantSpeedSpec>),
}

/// The governed range of a constant-speed propeller.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConstantSpeedSpec {
    /// Low and high blade-angle stops at 0.75 of the radius, degrees.
    pub blade_angle_range_deg: Sourced<[f64; 2]>,
    /// Lowest and highest crankshaft speed the governor holds, rpm.
    pub governor_speed_range_rpm: Sourced<[f64; 2]>,
    /// Whether the blades can be feathered.
    pub feathering: Sourced<bool>,
}

/// Origin of a propeller's thrust and power coefficient maps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerMapSpec {
    /// Whether the maps were measured (and in which report), computed from a
    /// stated blade, or blended from both.
    pub basis: Basis,
    /// The inputs the maps were generated from, in the generating tool's own
    /// terms, so the tables can be rebuilt; omitted for measured maps.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inputs: Option<serde_json::Value>,
    /// Known steps or gaps in the maps, and what they stand in for.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

impl PropellerDocument {
    /// Parses and validates `propeller.json` content, failing on the first
    /// problem.
    pub fn parse(bytes: &[u8]) -> Result<Self, SyspackError> {
        let document: Self =
            serde_json::from_slice(bytes).map_err(|source| SyspackError::PartDocumentParse {
                document: PROPELLER_DOCUMENT_ASSET,
                source,
            })?;
        document.validate()?;
        Ok(document)
    }

    fn validate(&self) -> Result<(), SyspackError> {
        let check = DocumentCheck::new(PROPELLER_DOCUMENT_ASSET, &self.sources)?;
        check.text("id", &self.id)?;
        check.text("name", &self.name)?;
        if self.blades.value == 0 {
            return Err(check.invalid("blades", "a propeller has at least one blade"));
        }
        check.fixed_unit("blades", &self.blades)?;
        check.quantity("diameter_m", &self.diameter_m)?;
        if let Some(gear) = &self.reduction_gear {
            check.quantity("reduction_gear", gear)?;
        }
        self.pitch.validate(&check)?;
        check.basis("map", &self.map.basis)?;
        if let Some(note) = &self.map.note {
            check.text("map.note", note)?;
        }
        check.models(&self.models)?;
        check.notes(&self.notes)
    }

    /// The documents the values cite.
    pub fn sources(&self) -> &SourceTable {
        &self.sources
    }
}

impl PitchSpec {
    fn validate(&self, check: &DocumentCheck<'_>) -> Result<(), SyspackError> {
        match self {
            Self::Fixed { geometric_pitch_m } => {
                check.quantity("pitch.fixed.geometric_pitch_m", geometric_pitch_m)
            }
            Self::ConstantSpeed(spec) => {
                check.range(
                    "pitch.constant_speed.blade_angle_range_deg",
                    &spec.blade_angle_range_deg,
                )?;
                check.range(
                    "pitch.constant_speed.governor_speed_range_rpm",
                    &spec.governor_speed_range_rpm,
                )?;
                check.fixed_unit("pitch.constant_speed.feathering", &spec.feathering)
            }
        }
    }
}
