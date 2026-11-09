use std::collections::BTreeSet;

use super::{BezelSize, InstrumentKindSpec, InstrumentPackDocument, InstrumentSpec, MarkingSpec};
use crate::SyspackError;
use crate::manifest::validate::is_slug;

/// Where an instrument's input comes from, parsed from its `source` string.
/// The grammar is closed so a typo fails at load rather than leaving a dead
/// instrument; the vehicle resolves the names it contains.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SourceBinding {
    /// `pitot_static.<device id>`: the vehicle's
    /// [`crate::DeviceKindSpec::PitotStatic`] device with that id.
    PitotStatic {
        /// The device id.
        device: String,
    },
    /// `engine[<index>].<quantity>`, such as `engine[0].manifold_pressure`.
    Engine {
        /// Engine number, from zero.
        index: u8,
        /// What is measured, a slug such as `rpm` or `oil_pressure`.
        quantity: String,
    },
    /// `system.<path>`: any other quantity the vehicle publishes, such as
    /// `system.vacuum.suction`.
    System {
        /// Dot-separated slugs.
        path: String,
    },
}

impl SourceBinding {
    /// Parses a binding, or `None` if `source` fits no form.
    pub fn parse(source: &str) -> Option<Self> {
        if let Some(device) = source.strip_prefix("pitot_static.") {
            return is_slug(device).then(|| Self::PitotStatic {
                device: device.to_owned(),
            });
        }
        if let Some(rest) = source.strip_prefix("engine[") {
            let (index, quantity) = rest.split_once("].")?;
            let all_digits = !index.is_empty() && index.chars().all(|c| c.is_ascii_digit());
            let index = index.parse().ok().filter(|_| all_digits)?;
            return is_slug(quantity).then(|| Self::Engine {
                index,
                quantity: quantity.to_owned(),
            });
        }
        let path = source.strip_prefix("system.")?;
        path.split('.').all(is_slug).then(|| Self::System {
            path: path.to_owned(),
        })
    }
}

fn invalid(spec: &InstrumentSpec, field: &'static str, detail: &str) -> SyspackError {
    SyspackError::InvalidInstrument {
        instrument: spec.id.clone(),
        field,
        detail: detail.to_owned(),
    }
}

fn check_markings(spec: &InstrumentSpec) -> Result<(), SyspackError> {
    for marking in &spec.markings {
        match *marking {
            MarkingSpec::Arc { from, to, .. } => {
                if !(from.is_finite() && to.is_finite() && from < to) {
                    return Err(invalid(spec, "markings", "an arc needs finite from < to"));
                }
            }
            MarkingSpec::Radial { at, .. } => {
                if !at.is_finite() {
                    return Err(invalid(spec, "markings", "a radial needs a finite value"));
                }
            }
        }
    }
    Ok(())
}

fn check_instrument(spec: &InstrumentSpec) -> Result<(), SyspackError> {
    if !is_slug(&spec.id) {
        return Err(invalid(spec, "id", "must be a lowercase slug"));
    }
    if spec.name.is_empty() {
        return Err(invalid(spec, "name", "must not be empty"));
    }
    let diameter_m = spec.bezel.diameter_m();
    if !(diameter_m.is_finite() && diameter_m > 0.0) {
        return Err(invalid(
            spec,
            "bezel",
            "diameter must be finite and positive",
        ));
    }
    if matches!(spec.bezel, BezelSize::Custom { .. }) && diameter_m > 1.0 {
        return Err(invalid(spec, "bezel", "diameter above 1 m"));
    }
    let binding = SourceBinding::parse(&spec.source).ok_or_else(|| {
        invalid(
            spec,
            "source",
            "must be pitot_static.<id>, engine[<n>].<quantity> or system.<path>",
        )
    })?;
    let pneumatic = matches!(
        spec.kind,
        InstrumentKindSpec::Airspeed { .. }
            | InstrumentKindSpec::Altimeter { .. }
            | InstrumentKindSpec::VerticalSpeed { .. }
    );
    if pneumatic && !matches!(binding, SourceBinding::PitotStatic { .. }) {
        return Err(invalid(
            spec,
            "source",
            "a pitot-static instrument needs pitot_static.<id>",
        ));
    }
    if !pneumatic && matches!(binding, SourceBinding::PitotStatic { .. }) {
        return Err(invalid(
            spec,
            "source",
            "only pitot-static instruments read pitot_static",
        ));
    }
    check_markings(spec)?;
    spec.mechanism().map(|_| ())
}

impl InstrumentPackDocument {
    /// Checks every instrument and the layout.
    pub(super) fn validate(&self) -> Result<(), SyspackError> {
        let mut ids = BTreeSet::new();
        for spec in &self.instruments {
            check_instrument(spec)?;
            if !ids.insert(spec.id.as_str()) {
                return Err(SyspackError::DuplicateInstrument {
                    id: spec.id.clone(),
                });
            }
        }
        let mut slots = BTreeSet::new();
        for slot in &self.layout {
            if !is_slug(&slot.slot) || !slots.insert(slot.slot.as_str()) {
                return Err(SyspackError::InvalidLayoutSlot {
                    slot: slot.slot.clone(),
                    detail: "slot names must be unique lowercase slugs",
                });
            }
            if !ids.contains(slot.instrument.as_str()) {
                return Err(SyspackError::UnknownLayoutInstrument {
                    slot: slot.slot.clone(),
                    instrument: slot.instrument.clone(),
                });
            }
            if !slot.offset_m.iter().all(|v| v.is_finite()) {
                return Err(SyspackError::InvalidLayoutSlot {
                    slot: slot.slot.clone(),
                    detail: "offset must be finite",
                });
            }
        }
        Ok(())
    }
}
