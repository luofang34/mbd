use std::collections::BTreeSet;

use super::{BezelSize, InstrumentKindSpec, InstrumentPackDocument, InstrumentSpec, MarkingSpec};
use crate::SyspackError;
use crate::manifest::validate::is_slug;

/// Where an instrument's input comes from, parsed from its `source` string.
/// The grammar is closed so a typo fails at load rather than leaving a dead
/// instrument; the vehicle resolves the names it contains.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SourceBinding {
    /// `pitot_static.<device id>`, with any device id free of `.`, `[` and
    /// `]`: the vehicle's
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
            let usable = !device.is_empty() && !device.contains(['.', '[', ']']);
            return usable.then(|| Self::PitotStatic {
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
    check_source(spec, &spec.source, &spec.kind)?;
    check_markings(spec)?;
    if let Some(dial) = &spec.dial {
        dial.validate(spec)?;
    }
    check_pointers(spec)?;
    spec.mechanism()?;
    spec.pointer_mechanisms().map(|_| ())
}

/// The most further pointers a case holds beside its first.
const MAX_FURTHER_POINTERS: usize = 3;

fn check_pointers(spec: &InstrumentSpec) -> Result<(), SyspackError> {
    if spec.pointers.is_empty() {
        return Ok(());
    }
    if spec.pointers.len() > MAX_FURTHER_POINTERS {
        return Err(invalid(
            spec,
            "pointers",
            "a case holds at most four pointers",
        ));
    }
    let altimeter =
        |kind: &InstrumentKindSpec| matches!(kind, InstrumentKindSpec::Altimeter { .. });
    if altimeter(&spec.kind) {
        return Err(invalid(
            spec,
            "pointers",
            "an altimeter's hands are its own",
        ));
    }
    let mut ids = BTreeSet::new();
    for pointer in &spec.pointers {
        if !is_slug(&pointer.id) || !ids.insert(pointer.id.as_str()) {
            return Err(invalid(
                spec,
                "pointers",
                "pointer ids must be unique lowercase slugs",
            ));
        }
        if altimeter(&pointer.kind) {
            return Err(invalid(
                spec,
                "pointers",
                "an altimeter is never a further pointer",
            ));
        }
        check_source(spec, &pointer.source, &pointer.kind)?;
        if let Some(dial) = &pointer.dial {
            dial.validate(spec)?;
        }
    }
    Ok(())
}

/// Checks that `source` parses and suits `kind`: pitot-static instruments,
/// and only they, read a pitot-static system.
fn check_source(
    spec: &InstrumentSpec,
    source: &str,
    kind: &InstrumentKindSpec,
) -> Result<(), SyspackError> {
    let binding = SourceBinding::parse(source).ok_or_else(|| {
        invalid(
            spec,
            "source",
            "must be pitot_static.<id>, engine[<n>].<quantity> or system.<path>",
        )
    })?;
    let pneumatic = matches!(
        kind,
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
    Ok(())
}

impl InstrumentPackDocument {
    /// Checks every instrument and the layout.
    pub(super) fn validate(&self) -> Result<(), SyspackError> {
        let mut faces = BTreeSet::new();
        for pin in &self.typefaces {
            pin.validate()?;
            if !faces.insert(pin.id.as_str()) {
                return Err(SyspackError::TypefacePackShape {
                    detail: "a pack pins each typeface once",
                });
            }
        }
        let mut ids = BTreeSet::new();
        for spec in &self.instruments {
            check_instrument(spec)?;
            if let Some(lettering) = &spec.lettering {
                lettering.validate(spec, self)?;
            }
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
