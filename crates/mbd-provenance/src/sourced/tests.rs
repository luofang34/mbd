use super::*;

fn sources() -> SourceTable {
    serde_json::from_str(
        r#"{"navy-manual-1941": {"document": "Kasei 10-series handling manual, Jan 1941", "tier": "operating_manual"}}"#,
    )
    .unwrap()
}

#[test]
fn a_value_parses_with_its_unit_basis_and_note() {
    let bore: Sourced<f64> = serde_json::from_str(
        r#"{"value": 150.0, "unit": "mm", "basis": {"sourced": ["navy-manual-1941"]}}"#,
    )
    .unwrap();
    assert_eq!(bore.value, 150.0);
    assert_eq!(bore.unit.as_deref(), Some("mm"));
    assert_eq!(bore.kind(), ProvenanceKind::Sourced);
    bore.check("geometry.bore", &sources()).unwrap();

    let ratios: Sourced<Vec<f64>> = serde_json::from_str(
        r#"{"value": [7.4, 9.118], "unit": "impeller/crankshaft",
            "basis": {"sourced": ["navy-manual-1941"]}, "note": "two speeds"}"#,
    )
    .unwrap();
    assert_eq!(ratios.value, [7.4, 9.118]);
    assert_eq!(ratios.note.as_deref(), Some("two speeds"));
}

#[test]
fn a_value_round_trips_without_its_absent_fields() {
    let feathering = Sourced {
        value: false,
        unit: None,
        basis: Basis::Assumed {
            reason: "not documented for the Model 11".to_owned(),
        },
        note: None,
    };
    let json = serde_json::to_string(&feathering).unwrap();
    assert_eq!(
        json,
        r#"{"value":false,"basis":{"assumed":{"reason":"not documented for the Model 11"}}}"#
    );
    assert_eq!(
        serde_json::from_str::<Sourced<bool>>(&json).unwrap(),
        feathering
    );
}

#[test]
fn a_value_without_a_basis_or_with_an_unknown_field_does_not_parse() {
    for json in [
        r#"{"value": 150.0, "unit": "mm"}"#,
        r#"{"value": 150.0, "basis": {"sourced": ["a"]}, "source": "a"}"#,
        r#"{"unit": "mm", "basis": {"sourced": ["a"]}}"#,
    ] {
        assert!(
            serde_json::from_str::<Sourced<f64>>(json).is_err(),
            "{json}"
        );
    }
}

#[test]
fn checking_a_value_checks_its_unit_and_its_basis() {
    let mut bsfc = Sourced {
        value: 0.3337,
        unit: Some(String::new()),
        basis: Basis::Fitted { to: String::new() },
        note: None,
    };
    assert_eq!(
        bsfc.check("bsfc", &sources()),
        Err(ProvenanceError::EmptyUnit {
            field: "bsfc".to_owned()
        })
    );
    bsfc.unit = Some("lb/hp/hr".to_owned());
    assert!(matches!(
        bsfc.check("bsfc", &sources()),
        Err(ProvenanceError::MissingStatement { kind: "fitted", .. })
    ));
    bsfc.basis = Basis::Sourced(vec!["taic-1".to_owned()]);
    assert_eq!(
        bsfc.check("bsfc", &sources()),
        Err(ProvenanceError::UnknownSource {
            field: "bsfc".to_owned(),
            id: "taic-1".to_owned()
        })
    );
}
