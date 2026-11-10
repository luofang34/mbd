use super::*;

fn table() -> SourceTable {
    serde_json::from_str(
        r#"{
            "taic-1": {
                "document": "TAIC Manual No. 1, Japanese Aircraft Performance and Characteristics, Dec 1944",
                "tier": "test_report",
                "url": "https://archive.org/details/japanese-aircraft-performance-characteristics-taic-manual",
                "pages": "pp.861-862"
            },
            "forum-table": { "document": "Simulator community data sheet", "tier": "community" }
        }"#,
    )
    .unwrap()
}

fn read(ids: &[&str]) -> Basis {
    Basis::Sourced(ids.iter().map(|id| (*id).to_owned()).collect())
}

#[test]
fn a_table_round_trips_and_validates() {
    let table = table();
    table.validate().unwrap();
    let json = serde_json::to_string(&table).unwrap();
    assert_eq!(serde_json::from_str::<SourceTable>(&json).unwrap(), table);
    assert_eq!(table.get("taic-1").unwrap().tier, SourceTier::TestReport);
    assert_eq!(
        table.iter().map(|(id, _)| id).collect::<Vec<_>>(),
        ["forum-table", "taic-1"]
    );
}

#[test]
fn a_record_with_an_unknown_field_or_tier_does_not_parse() {
    for json in [
        r#"{"a": {"document": "d", "tier": "test_report", "confidence": "high"}}"#,
        r#"{"a": {"document": "d", "tier": "hearsay"}}"#,
        r#"{"a": {"tier": "drawing"}}"#,
    ] {
        assert!(serde_json::from_str::<SourceTable>(json).is_err(), "{json}");
    }
}

#[test]
fn a_bad_id_or_an_empty_statement_is_refused() {
    let record = |document: &str, pages: Option<&str>| SourceRecord {
        document: document.to_owned(),
        tier: SourceTier::Drawing,
        url: None,
        pages: pages.map(str::to_owned),
    };
    let build = |id: &str, r: SourceRecord| SourceTable::new(BTreeMap::from([(id.to_owned(), r)]));
    assert_eq!(
        build("TAIC 1", record("d", None)),
        Err(ProvenanceError::InvalidSourceId {
            id: "TAIC 1".to_owned()
        })
    );
    assert_eq!(
        build("a", record("  ", None)),
        Err(ProvenanceError::EmptySourceField {
            id: "a".to_owned(),
            field: "document"
        })
    );
    assert_eq!(
        build("a", record("d", Some(""))),
        Err(ProvenanceError::EmptySourceField {
            id: "a".to_owned(),
            field: "pages"
        })
    );
    assert!(build("a", record("d", Some("p.3"))).is_ok());
}

#[test]
fn a_citation_must_resolve() {
    let table = table();
    assert_eq!(
        table.check("bore", &read(&["navy-manual-1941"])),
        Err(ProvenanceError::UnknownSource {
            field: "bore".to_owned(),
            id: "navy-manual-1941".to_owned()
        })
    );
    let derived = Basis::Derived {
        from: vec!["missing".to_owned()],
        method: "m".to_owned(),
    };
    assert!(matches!(
        table.check("x", &derived),
        Err(ProvenanceError::UnknownSource { .. })
    ));
}

#[test]
fn a_value_read_from_sources_needs_one_that_may_supply_it() {
    let table = table();
    assert_eq!(
        table.check("bore", &read(&[])),
        Err(ProvenanceError::NoSource {
            field: "bore".to_owned()
        })
    );
    assert_eq!(
        table.check("top speed", &read(&["forum-table"])),
        Err(ProvenanceError::CrossCheckOnly {
            field: "top speed".to_owned(),
            id: "forum-table".to_owned()
        })
    );
    table.check("bore", &read(&["taic-1"])).unwrap();
    table
        .check("top speed", &read(&["forum-table", "taic-1"]))
        .unwrap();
}

#[test]
fn every_other_basis_must_carry_its_statement() {
    let table = table();
    let cases = [
        (
            Basis::Derived {
                from: Vec::new(),
                method: " ".to_owned(),
            },
            "derived",
            "its method",
        ),
        (
            Basis::Fitted { to: String::new() },
            "fitted",
            "what it was fitted to",
        ),
        (
            Basis::Estimated {
                method: String::new(),
            },
            "estimated",
            "its method",
        ),
        (
            Basis::Assumed {
                reason: String::new(),
            },
            "assumed",
            "its reason",
        ),
    ];
    for (basis, kind, statement) in cases {
        assert_eq!(
            table.check("cd0", &basis),
            Err(ProvenanceError::MissingStatement {
                field: "cd0".to_owned(),
                kind,
                statement
            })
        );
    }
    let fitted = Basis::Fitted {
        to: "Kasei 11 level speeds".to_owned(),
    };
    table.check("cd0", &fitted).unwrap();
    let derived = Basis::Derived {
        from: Vec::new(),
        method: "fuel flow over brake power at the rated point".to_owned(),
    };
    table.check("bsfc", &derived).unwrap();
}
