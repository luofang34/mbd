use serde_json::{Value, json};

use crate::instrument_pack::tests::ga_panel;
use crate::{InstrumentPackDocument, SyspackError};

const SHA: &str = "1111111111111111111111111111111111111111111111111111111111111111";

fn lettered(lettering: Value) -> Value {
    let mut panel = ga_panel();
    panel["typefaces"] = json!([{"id": "us-ms33558", "version": "1.0.0", "manifest_sha256": SHA}]);
    panel["instruments"][1]["lettering"] = lettering;
    panel
}

fn parse(value: &Value) -> Result<InstrumentPackDocument, SyspackError> {
    InstrumentPackDocument::parse(value.to_string().as_bytes())
}

#[test]
fn a_dial_is_lettered_in_a_pinned_face_and_style() {
    let document = parse(&lettered(json!({
        "numerals": {"face": "us-ms33558", "condense": 0.75, "pen": 0.1667, "alternates": ["one.flag"]},
    })))
    .unwrap();
    let lettering = document.instruments[1].lettering.as_ref().unwrap();
    assert_eq!(lettering.numerals.condense, Some(0.75));
    assert_eq!(lettering.legends(), &lettering.numerals);
    let text = serde_json::to_string(&document).unwrap();
    assert_eq!(
        InstrumentPackDocument::parse(text.as_bytes()).unwrap(),
        document
    );
}

#[test]
fn rejects_lettering_that_cannot_be_drawn() {
    for (case, lettering) in [
        ("unpinned", json!({"numerals": {"face": "din-1451"}})),
        (
            "condense",
            json!({"numerals": {"face": "us-ms33558", "condense": 0.1}}),
        ),
        (
            "pen",
            json!({"numerals": {"face": "us-ms33558", "pen": 0.0}}),
        ),
        (
            "slant",
            json!({"numerals": {"face": "us-ms33558", "slant": 2.0}}),
        ),
    ] {
        assert!(parse(&lettered(lettering)).is_err(), "{case}");
    }
    let mut twice = lettered(json!({"numerals": {"face": "us-ms33558"}}));
    twice["typefaces"] = json!([
        {"id": "us-ms33558", "version": "1.0.0", "manifest_sha256": SHA},
        {"id": "us-ms33558", "version": "1.0.1", "manifest_sha256": SHA},
    ]);
    assert!(parse(&twice).is_err());
}
