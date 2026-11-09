use serde_json::{Value, json};

use crate::instrument_pack::tests::ga_panel;
use crate::{DialFaceSpec, FailureSpec, InstrumentPackDocument, PointerShapeSpec, SyspackError};

fn with_face(face: Value) -> Value {
    let mut panel = ga_panel();
    panel["instruments"][0]["dial"] = face;
    panel
}

fn parse(value: &Value) -> Result<InstrumentPackDocument, SyspackError> {
    InstrumentPackDocument::parse(value.to_string().as_bytes())
}

fn printed_face() -> Value {
    json!({
        "reference_deg": 0.0,
        "face_diameter_m": 0.068,
        "graduations": [
            {"from": 40.0, "to": 200.0, "every": 5.0},
            {"from": 40.0, "to": 200.0, "every": 10.0, "major": true},
        ],
        "numerals": [{"value": 40.0, "text": "40"}, {"value": 100.0, "text": "100"}],
        "legends": [{"text": "KNOTS", "at": [0.0, 0.3], "size": 0.09}],
        "luminous": true,
        "pointers": [{"shape": "Lance", "length": 0.9, "width": 0.06, "tail": 0.2}],
        "on_failure": {"Park": {"angle_deg": 0.0}},
    })
}

#[test]
fn a_printed_face_parses_and_round_trips() {
    let document = parse(&with_face(printed_face())).unwrap();
    let face = document.instruments[0].dial.as_ref().unwrap();
    assert_eq!(face.reference_deg, Some(0.0));
    assert_eq!(face.graduations.len(), 2);
    assert!(face.graduations[1].major && !face.graduations[0].major);
    assert_eq!(face.numerals[1].text, "100");
    assert_eq!(face.pointers[0].shape, PointerShapeSpec::Lance);
    assert_eq!(face.on_failure, Some(FailureSpec::Park { angle_deg: 0.0 }));
    let text = serde_json::to_string(&document).unwrap();
    assert_eq!(
        InstrumentPackDocument::parse(text.as_bytes()).unwrap(),
        document
    );
}

#[test]
fn every_face_field_is_optional() {
    let document = parse(&with_face(json!({}))).unwrap();
    assert_eq!(document.instruments[0].dial, Some(DialFaceSpec::default()));
    assert_eq!(parse(&ga_panel()).unwrap().instruments[0].dial, None);
}

#[test]
fn rejects_faces_that_cannot_be_printed() {
    let broken = [
        ("face_diameter_m", json!(0.2)),
        (
            "graduations",
            json!([{"from": 10.0, "to": 0.0, "every": 1.0}]),
        ),
        (
            "graduations",
            json!([{"from": 0.0, "to": 10.0, "every": 0.0}]),
        ),
        ("numerals", json!([{"value": 10.0, "text": ""}])),
        ("numerals", json!([{"value": 10.0, "text": "1\u{00b0}"}])),
        (
            "legends",
            json!([{"text": "MPH", "at": [0.0, 0.3], "size": 0.0}]),
        ),
        (
            "pointers",
            json!([{"shape": "Lance", "length": 0.9, "width": -0.06}]),
        ),
        (
            "pointers",
            json!([
                {"shape": "Lance", "length": 0.9, "width": 0.06},
                {"shape": "Lance", "length": 0.9, "width": 0.06},
                {"shape": "Lance", "length": 0.9, "width": 0.06},
                {"shape": "Lance", "length": 0.9, "width": 0.06},
                {"shape": "Lance", "length": 0.9, "width": 0.06},
            ]),
        ),
    ];
    for (field, value) in broken {
        let mut face = printed_face();
        face[field] = value;
        match parse(&with_face(face)) {
            Err(SyspackError::InvalidInstrument { field: "dial", .. }) => {}
            other => panic!("{field}: expected an invalid dial, got {other:?}"),
        }
    }
    let mut face = printed_face();
    face["colour"] = json!("black");
    assert!(matches!(
        parse(&with_face(face)),
        Err(SyspackError::InstrumentsParse(_))
    ));
}
