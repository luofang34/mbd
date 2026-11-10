use mbd_provenance::{ProvenanceError, ProvenanceKind};
use serde_json::{Value, json};

use super::*;
use crate::part::fixtures::KASEI_11_ENGINE;

fn document() -> Value {
    serde_json::from_str(KASEI_11_ENGINE).unwrap()
}

fn parse(document: &Value) -> Result<EngineDocument, SyspackError> {
    EngineDocument::parse(document.to_string().as_bytes())
}

fn piston(document: &mut Value) -> &mut Value {
    &mut document["engine"]["piston"]
}

fn invalid_field(result: Result<EngineDocument, SyspackError>) -> String {
    match result {
        Err(SyspackError::InvalidPartDocument { field, .. }) => field,
        other => panic!("expected an invalid field, got {other:?}"),
    }
}

fn provenance(result: Result<EngineDocument, SyspackError>) -> ProvenanceError {
    match result {
        Err(SyspackError::PartProvenance { source, .. }) => source,
        other => panic!("expected a provenance error, got {other:?}"),
    }
}

#[test]
fn an_engine_parses_with_its_ratings_and_their_origins() {
    let engine = parse(&document()).unwrap();
    let EngineSpec::Piston(piston) = &engine.engine;
    assert_eq!(piston.cylinders.value, 14);
    assert_eq!(piston.bore_m.value, 0.150);
    assert_eq!(piston.ratings.len(), 4);
    let second = &piston.ratings[2];
    assert_eq!(second.name, "rated, second speed");
    assert_eq!(second.supercharger_speed, Some(2));
    assert_eq!(second.altitude_m, 4100.0);
    assert_eq!(second.basis.kind(), ProvenanceKind::Sourced);
    assert_eq!(
        engine.models["jsbsim"]["bsfc_indicated"].kind(),
        ProvenanceKind::Fitted
    );
    assert!(engine.sources().get("taic-1").is_some());
    let written = serde_json::to_vec(&engine).unwrap();
    assert_eq!(EngineDocument::parse(&written).unwrap(), engine);
}

#[test]
fn an_unknown_field_anywhere_does_not_parse() {
    let mut top = document();
    top["installed_in"] = json!("G4M1");
    let mut spec = document();
    piston(&mut spec)["length"] = json!({"value": 1.705, "basis": {"sourced": ["taic-1"]}});
    let mut rating = document();
    piston(&mut rating)["ratings"][0]["power_hp"] = json!(1505.0);
    let mut value = document();
    piston(&mut value)["bore_m"]["source"] = json!("navy-manual-1941");
    for document in [top, spec, rating, value] {
        assert!(matches!(
            parse(&document),
            Err(SyspackError::PartDocumentParse {
                document: "engine.json",
                ..
            })
        ));
    }
}

#[test]
fn every_value_must_state_an_origin_the_document_can_back() {
    let mut unknown = document();
    piston(&mut unknown)["stroke_m"]["basis"] = json!({"sourced": ["jane-s-1945"]});
    assert_eq!(
        provenance(parse(&unknown)),
        ProvenanceError::UnknownSource {
            field: "stroke_m".to_owned(),
            id: "jane-s-1945".to_owned()
        }
    );
    let mut rating = document();
    piston(&mut rating)["ratings"][3]["basis"] = json!({"sourced": []});
    assert_eq!(
        provenance(parse(&rating)),
        ProvenanceError::NoSource {
            field: "ratings[military, sea level]".to_owned()
        }
    );
    let mut model = document();
    model["models"]["jsbsim"]["bsfc_indicated"]["basis"] = json!({"fitted": {"to": ""}});
    assert!(matches!(
        provenance(parse(&model)),
        ProvenanceError::MissingStatement { kind: "fitted", .. }
    ));
    let mut community = document();
    community["sources"]["taic-1"]["tier"] = json!("community");
    assert_eq!(
        provenance(parse(&community)),
        ProvenanceError::CrossCheckOnly {
            field: "ratings[military, sea level]".to_owned(),
            id: "taic-1".to_owned()
        }
    );
}

#[test]
fn quantities_are_si_under_the_name_of_their_field() {
    let mut millimetres = document();
    piston(&mut millimetres)["bore_m"] = json!({
        "value": 150.0, "unit": "mm", "basis": {"sourced": ["navy-manual-1941"]}
    });
    assert_eq!(invalid_field(parse(&millimetres)), "bore_m");
    let mut negative = document();
    piston(&mut negative)["dry_mass_kg"]["value"] = json!(-725.0);
    assert_eq!(invalid_field(parse(&negative)), "dry_mass_kg");
    let mut model_unit = document();
    model_unit["models"]["jsbsim"]["bsfc_indicated"]["unit"] = json!("kg/kWh");
    parse(&model_unit).unwrap();
}

#[test]
fn ratings_must_be_distinct_physical_and_in_a_declared_supercharger_speed() {
    let mut none = document();
    piston(&mut none)["ratings"] = json!([]);
    assert_eq!(invalid_field(parse(&none)), "ratings");
    let mut twice = document();
    piston(&mut twice)["ratings"][1]["name"] = json!("take-off (1 min)");
    assert_eq!(invalid_field(parse(&twice)), "ratings[take-off (1 min)]");
    let mut power = document();
    piston(&mut power)["ratings"][0]["power_w"] = json!(0.0);
    assert_eq!(
        invalid_field(parse(&power)),
        "ratings[take-off (1 min)].power_w"
    );
    let mut third = document();
    piston(&mut third)["ratings"][2]["supercharger_speed"] = json!(3);
    assert_eq!(
        invalid_field(parse(&third)),
        "ratings[rated, second speed].supercharger_speed"
    );
    let mut unsupercharged = document();
    piston(&mut unsupercharged)["induction"]
        .as_object_mut()
        .unwrap()
        .remove("supercharger_gear_ratios");
    assert_eq!(
        invalid_field(parse(&unsupercharged)),
        "ratings[take-off (1 min)].supercharger_speed"
    );
    let mut descending = document();
    piston(&mut descending)["induction"]["supercharger_gear_ratios"]["value"] = json!([9.118, 7.4]);
    assert_eq!(
        invalid_field(parse(&descending)),
        "induction.supercharger_gear_ratios"
    );
}
