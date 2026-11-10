use std::collections::BTreeSet;

use mbd_provenance::{Basis, SourceTable, Sourced};
use serde::{Deserialize, Serialize};

use super::{DocumentCheck, ModelParameters};
use crate::{ENGINE_DOCUMENT_ASSET, SyspackError};

#[cfg(test)]
mod tests;

/// `engine.json`: an engine as its sources describe it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EngineDocument {
    /// The id of the part package the document belongs to.
    pub id: String,
    /// Maker and designation.
    pub name: String,
    /// The documents the values below cite.
    pub sources: SourceTable,
    /// What kind of engine it is, with that kind's description.
    pub engine: EngineSpec,
    /// Values a consumer's own model needs that no source gives.
    #[serde(default, skip_serializing_if = "ModelParameters::is_empty")]
    pub models: ModelParameters,
    /// Conflicts between sources, and what was looked for and not found.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
}

/// The kinds of engine a part can be.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum EngineSpec {
    /// A reciprocating engine driving a shaft.
    Piston(PistonEngineSpec),
}

/// A piston engine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PistonEngineSpec {
    /// Arrangement and cooling, such as `14-cylinder two-row air-cooled radial`.
    pub layout: Sourced<String>,
    /// Number of cylinders.
    pub cylinders: Sourced<u32>,
    /// Cylinder bore, m.
    pub bore_m: Sourced<f64>,
    /// Piston stroke, m.
    pub stroke_m: Sourced<f64>,
    /// Swept volume of all cylinders, m^3.
    pub displacement_m3: Sourced<f64>,
    /// Compression ratio.
    pub compression_ratio: Sourced<f64>,
    /// Dry mass, kg.
    pub dry_mass_kg: Sourced<f64>,
    /// Propeller shaft speed over crankshaft speed; omitted for direct drive.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reduction_gear: Option<Sourced<f64>>,
    /// How the engine is supplied with charge.
    pub induction: InductionSpec,
    /// Fuel grade.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fuel: Option<Sourced<String>>,
    /// The power ratings the sources state; at least one.
    pub ratings: Vec<EngineRating>,
}

/// Induction system of a piston engine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InductionSpec {
    /// The system in words, such as `single-stage, two-speed, gear-driven
    /// supercharger` or `naturally aspirated, fuel injected`.
    pub description: Sourced<String>,
    /// Impeller speed over crankshaft speed of a gear-driven supercharger,
    /// one entry per speed in ascending order; omitted when there is none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub supercharger_gear_ratios: Option<Sourced<Vec<f64>>>,
    /// Supercharger impeller diameter, m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub impeller_diameter_m: Option<Sourced<f64>>,
    /// Carburettor or injection system.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub carburettor: Option<Sourced<String>>,
}

/// One stated power rating.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EngineRating {
    /// The rating's name, such as `take-off (1 min)`; unique in the document.
    pub name: String,
    /// Brake power, W.
    pub power_w: f64,
    /// Crankshaft speed, rpm.
    pub speed_rpm: f64,
    /// Absolute manifold pressure, Pa; omitted when the source gives none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manifold_pressure_pa: Option<f64>,
    /// Pressure altitude the rating holds at, m.
    pub altitude_m: f64,
    /// Which supercharger speed it is rated in, counted from 1; omitted for
    /// an engine without a multi-speed supercharger.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub supercharger_speed: Option<u32>,
    /// How long the rating may be held, s; omitted when unlimited.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub time_limit_s: Option<f64>,
    /// Where the rating comes from.
    pub basis: Basis,
    /// The source's own figures and units, and conditions.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

impl EngineDocument {
    /// Parses and validates `engine.json` content, failing on the first
    /// problem.
    pub fn parse(bytes: &[u8]) -> Result<Self, SyspackError> {
        let document: Self =
            serde_json::from_slice(bytes).map_err(|source| SyspackError::PartDocumentParse {
                document: ENGINE_DOCUMENT_ASSET,
                source,
            })?;
        document.validate()?;
        Ok(document)
    }

    fn validate(&self) -> Result<(), SyspackError> {
        let check = DocumentCheck::new(ENGINE_DOCUMENT_ASSET, &self.sources)?;
        check.text("id", &self.id)?;
        check.text("name", &self.name)?;
        match &self.engine {
            EngineSpec::Piston(piston) => piston.validate(&check)?,
        }
        check.models(&self.models)?;
        check.notes(&self.notes)
    }

    /// The documents the values cite.
    pub fn sources(&self) -> &SourceTable {
        &self.sources
    }
}

impl PistonEngineSpec {
    fn validate(&self, check: &DocumentCheck<'_>) -> Result<(), SyspackError> {
        check.fixed_unit("layout", &self.layout)?;
        check.text("layout", &self.layout.value)?;
        if self.cylinders.value == 0 {
            return Err(check.invalid("cylinders", "an engine has at least one cylinder"));
        }
        check.fixed_unit("cylinders", &self.cylinders)?;
        for (field, value) in [
            ("bore_m", &self.bore_m),
            ("stroke_m", &self.stroke_m),
            ("displacement_m3", &self.displacement_m3),
            ("compression_ratio", &self.compression_ratio),
            ("dry_mass_kg", &self.dry_mass_kg),
        ] {
            check.quantity(field, value)?;
        }
        if let Some(gear) = &self.reduction_gear {
            check.quantity("reduction_gear", gear)?;
        }
        if let Some(fuel) = &self.fuel {
            check.fixed_unit("fuel", fuel)?;
            check.text("fuel", &fuel.value)?;
        }
        let speeds = self.induction.validate(check)?;
        self.validate_ratings(check, speeds)
    }

    fn validate_ratings(
        &self,
        check: &DocumentCheck<'_>,
        supercharger_speeds: usize,
    ) -> Result<(), SyspackError> {
        if self.ratings.is_empty() {
            return Err(check.invalid("ratings", "an engine states at least one rating"));
        }
        let mut names = BTreeSet::new();
        for rating in &self.ratings {
            check.text("ratings.name", &rating.name)?;
            let field = format!("ratings[{}]", rating.name);
            if !names.insert(rating.name.as_str()) {
                return Err(check.invalid(&field, "is stated twice"));
            }
            rating.validate(check, &field, supercharger_speeds)?;
        }
        Ok(())
    }
}

impl InductionSpec {
    /// Checks the induction system and returns how many supercharger speeds
    /// it declares.
    fn validate(&self, check: &DocumentCheck<'_>) -> Result<usize, SyspackError> {
        check.fixed_unit("induction.description", &self.description)?;
        check.text("induction.description", &self.description.value)?;
        if let Some(impeller) = &self.impeller_diameter_m {
            check.quantity("induction.impeller_diameter_m", impeller)?;
        }
        if let Some(carburettor) = &self.carburettor {
            check.fixed_unit("induction.carburettor", carburettor)?;
            check.text("induction.carburettor", &carburettor.value)?;
        }
        let Some(ratios) = &self.supercharger_gear_ratios else {
            return Ok(0);
        };
        let field = "induction.supercharger_gear_ratios";
        check.fixed_unit(field, ratios)?;
        if ratios.value.is_empty() {
            return Err(check.invalid(field, "omit the field when there is no supercharger"));
        }
        let mut previous = 0.0;
        for ratio in &ratios.value {
            check.positive(field, *ratio)?;
            if *ratio <= previous {
                return Err(check.invalid(field, "speeds must be in ascending order"));
            }
            previous = *ratio;
        }
        Ok(ratios.value.len())
    }
}

impl EngineRating {
    fn validate(
        &self,
        check: &DocumentCheck<'_>,
        field: &str,
        supercharger_speeds: usize,
    ) -> Result<(), SyspackError> {
        check.positive(&format!("{field}.power_w"), self.power_w)?;
        check.positive(&format!("{field}.speed_rpm"), self.speed_rpm)?;
        if let Some(pressure) = self.manifold_pressure_pa {
            check.positive(&format!("{field}.manifold_pressure_pa"), pressure)?;
        }
        if let Some(limit) = self.time_limit_s {
            check.positive(&format!("{field}.time_limit_s"), limit)?;
        }
        if !self.altitude_m.is_finite() {
            return Err(check.invalid(&format!("{field}.altitude_m"), "is not finite"));
        }
        if let Some(speed) = self.supercharger_speed {
            let declared = u32::try_from(supercharger_speeds).unwrap_or(u32::MAX);
            if speed == 0 || speed > declared {
                return Err(check.invalid(
                    &format!("{field}.supercharger_speed"),
                    format!("{speed} is not one of the {declared} declared speeds"),
                ));
            }
        }
        if let Some(note) = &self.note {
            check.text(&format!("{field}.note"), note)?;
        }
        check.basis(field, &self.basis)
    }
}
