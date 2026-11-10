use super::*;

#[test]
fn each_basis_is_written_under_its_kind_name() {
    let cases = [
        (
            Basis::Sourced(vec!["navy-manual-1941".to_owned()]),
            r#"{"sourced":["navy-manual-1941"]}"#,
        ),
        (
            Basis::Derived {
                from: vec!["taic-1".to_owned()],
                method: "rated power over boost".to_owned(),
            },
            r#"{"derived":{"from":["taic-1"],"method":"rated power over boost"}}"#,
        ),
        (
            Basis::Fitted {
                to: "the rated first-speed power".to_owned(),
            },
            r#"{"fitted":{"to":"the rated first-speed power"}}"#,
        ),
        (
            Basis::Estimated {
                method: "tail-volume coefficients".to_owned(),
            },
            r#"{"estimated":{"method":"tail-volume coefficients"}}"#,
        ),
        (
            Basis::Assumed {
                reason: "not documented".to_owned(),
            },
            r#"{"assumed":{"reason":"not documented"}}"#,
        ),
    ];
    for (basis, json) in cases {
        assert_eq!(serde_json::to_string(&basis).unwrap(), json);
        assert_eq!(serde_json::from_str::<Basis>(json).unwrap(), basis);
        let name = json.split('"').nth(1).unwrap();
        assert_eq!(basis.kind().as_str(), name);
    }
}

#[test]
fn a_basis_with_an_unknown_kind_or_field_does_not_parse() {
    for json in [
        r#"{"guessed":{"reason":"x"}}"#,
        r#"{"fitted":{"to":"x","tolerance":0.1}}"#,
        r#"{"fitted":{}}"#,
    ] {
        assert!(serde_json::from_str::<Basis>(json).is_err(), "{json}");
    }
}

#[test]
fn only_read_and_derived_values_count_as_evidence() {
    use ProvenanceKind::{Assumed, Derived, Estimated, Fitted, Sourced};
    let evidence: Vec<_> = [Sourced, Derived, Fitted, Estimated, Assumed]
        .into_iter()
        .filter(|kind| kind.is_evidence())
        .collect();
    assert_eq!(evidence, [Sourced, Derived]);
}

#[test]
fn a_derived_value_cites_the_sources_of_its_inputs() {
    let derived = Basis::Derived {
        from: vec!["a".to_owned(), "b".to_owned()],
        method: "m".to_owned(),
    };
    assert_eq!(derived.cited(), ["a", "b"]);
    assert!(Basis::Fitted { to: "x".to_owned() }.cited().is_empty());
}
