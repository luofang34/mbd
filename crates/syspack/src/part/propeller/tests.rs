use mbd_provenance::{ProvenanceError, ProvenanceKind};
use serde_json::{Value, json};

use super::*;
use crate::part::fixtures::HAMILTON_PROPELLER;

fn document() -> Value {
    serde_json::from_str(HAMILTON_PROPELLER).unwrap()
}

fn parse(document: &Value) -> Result<PropellerDocument, SyspackError> {
    PropellerDocument::parse(document.to_string().as_bytes())
}

fn invalid_field(result: Result<PropellerDocument, SyspackError>) -> String {
    match result {
        Err(SyspackError::InvalidPartDocument { field, .. }) => field,
        other => panic!("expected an invalid field, got {other:?}"),
    }
}

#[test]
fn a_propeller_parses_and_says_which_of_its_geometry_is_estimated() {
    let propeller = parse(&document()).unwrap();
    assert_eq!(propeller.blades.value, 3);
    assert_eq!(propeller.blades.kind(), ProvenanceKind::Sourced);
    assert_eq!(propeller.diameter_m.kind(), ProvenanceKind::Estimated);
    assert_eq!(propeller.map.basis.kind(), ProvenanceKind::Estimated);
    let PitchSpec::ConstantSpeed(governed) = &propeller.pitch else {
        panic!("the fixture is a constant-speed propeller");
    };
    assert_eq!(governed.blade_angle_range_deg.value, [16.0, 36.0]);
    assert_eq!(governed.feathering.kind(), ProvenanceKind::Assumed);
    let written = serde_json::to_vec(&propeller).unwrap();
    assert_eq!(PropellerDocument::parse(&written).unwrap(), propeller);
}

#[test]
fn a_fixed_pitch_propeller_states_its_geometric_pitch() {
    let mut fixed = document();
    fixed["pitch"] = json!({"fixed": {"geometric_pitch_m": {
        "value": 1.524, "basis": {"sourced": ["navy-test-table"]}, "note": "60 in"
    }}});
    let propeller = parse(&fixed).unwrap();
    assert!(matches!(propeller.pitch, PitchSpec::Fixed { .. }));
    fixed["pitch"]["fixed"]["geometric_pitch_m"]["value"] = json!(0.0);
    assert_eq!(
        invalid_field(parse(&fixed)),
        "pitch.fixed.geometric_pitch_m"
    );
}

#[test]
fn an_unknown_field_or_pitch_kind_does_not_parse() {
    let mut top = document();
    top["tip_mach_losses"] = json!("estimated");
    let mut pitch = document();
    pitch["pitch"] = json!({"variable": {}});
    let mut map = document();
    map["map"]["tables"] = json!("propeller.xml");
    for document in [top, pitch, map] {
        assert!(matches!(
            parse(&document),
            Err(SyspackError::PartDocumentParse {
                document: "propeller.json",
                ..
            })
        ));
    }
}

#[test]
fn ranges_must_rise_and_the_map_must_state_its_origin() {
    let mut inverted = document();
    inverted["pitch"]["constant_speed"]["blade_angle_range_deg"]["value"] = json!([36.0, 16.0]);
    assert_eq!(
        invalid_field(parse(&inverted)),
        "pitch.constant_speed.blade_angle_range_deg"
    );
    let mut bladeless = document();
    bladeless["blades"]["value"] = json!(0);
    assert_eq!(invalid_field(parse(&bladeless)), "blades");
    let mut measured = document();
    measured["map"]["basis"] = json!({"sourced": ["naca-tr-640"]});
    assert!(matches!(
        parse(&measured),
        Err(SyspackError::PartProvenance {
            source: ProvenanceError::UnknownSource { .. },
            ..
        })
    ));
    let mut unstated = document();
    unstated["map"]["basis"] = json!({"estimated": {"method": ""}});
    assert!(matches!(
        parse(&unstated),
        Err(SyspackError::PartProvenance {
            source: ProvenanceError::MissingStatement { .. },
            ..
        })
    ));
}
