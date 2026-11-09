use serde_json::{Value, json};

use super::*;
use mbd_airdata::AirdataError;

fn scale(points: &[(f64, f64)]) -> Value {
    points
        .iter()
        .map(|&(value, angle_deg)| json!({"value": value, "angle_deg": angle_deg}))
        .collect()
}

fn needle() -> Value {
    json!({"natural_frequency_rad_s": 14.0, "damping_ratio": 0.7})
}

fn pitot_static_instruments() -> Value {
    json!([
        {
            "id": "asi",
            "name": "Airspeed indicator, knots",
            "bezel": "ThreeAndOneEighthInch",
            "source": "pitot_static.main",
            "needle": needle(),
            "markings": [
                {"Arc": {"from": 33.0, "to": 85.0, "colour": "White"}},
                {"Arc": {"from": 44.0, "to": 129.0, "colour": "Green"}},
                {"Arc": {"from": 129.0, "to": 163.0, "colour": "Yellow"}},
                {"Radial": {"at": 163.0, "colour": "Red"}},
            ],
            "kind": {"Airspeed": {
                "units": "Knots",
                "scale": scale(&[(0.0, 0.0), (40.0, 25.0), (100.0, 170.0), (200.0, 340.0)]),
                "instrument_error": [[40.0, 1.0], [160.0, -1.0]],
            }},
        },
        {
            "id": "altimeter",
            "name": "Kollsman Type C-12 sensitive altimeter",
            "bezel": "ThreeAndOneEighthInch",
            "source": "pitot_static.main",
            "needle": needle(),
            "kind": {"Altimeter": {
                "units": "Feet",
                "kollsman": {"units": "InchesOfMercury", "min": 28.10, "max": 31.00},
                "aneroid_tau_s": 0.1,
                "range": [-1000.0, 20000.0],
            }},
        },
        {
            "id": "vsi",
            "name": "Vertical speed indicator, feet per minute",
            "bezel": "ThreeAndOneEighthInch",
            "source": "pitot_static.main",
            "needle": needle(),
            "kind": {"VerticalSpeed": {
                "units": "FeetPerMinute",
                "scale": scale(&[(-2000.0, -170.0), (0.0, 0.0), (2000.0, 170.0)]),
                "leak_tau_s": 2.5,
            }},
        },
    ])
}

/// A GA analog panel: airspeed, altimeter, vertical speed, tachometer and
/// oil pressure.
pub(crate) fn ga_panel() -> Value {
    let mut instruments = pitot_static_instruments();
    let engine = [
        json!({
            "id": "tach",
            "name": "Recording tachometer",
            "bezel": "ThreeAndOneEighthInch",
            "source": "engine[0].rpm",
            "needle": needle(),
            "kind": {"Tachometer": {
                "units": "RevolutionsPerMinute",
                "scale": scale(&[(0.0, 0.0), (3500.0, 270.0)]),
                "drag_cup_tau_s": 0.4,
            }},
        }),
        json!({
            "id": "oil_pressure",
            "name": "Oil pressure gauge",
            "bezel": {"Custom": {"diameter_m": 0.05}},
            "source": "engine[0].oil_pressure",
            "needle": needle(),
            "kind": {"PressureGauge": {
                "units": "PoundsPerSquareInch",
                "scale": scale(&[(0.0, 0.0), (115.0, 90.0)]),
                "line_tau_s": 0.0,
            }},
        }),
    ];
    if let Some(list) = instruments.as_array_mut() {
        list.extend(engine);
    }
    json!({
        "instruments": instruments,
        "layout": [
            {"slot": "top_left", "instrument": "asi", "offset_m": [-0.09, 0.05]},
            {"slot": "top_right", "instrument": "altimeter", "offset_m": [0.09, 0.05]},
            {"slot": "bottom_right", "instrument": "vsi", "offset_m": [0.09, -0.05]},
            {"slot": "engine_left", "instrument": "tach", "offset_m": [0.2, 0.0]},
            {"slot": "engine_right", "instrument": "oil_pressure", "offset_m": [0.27, 0.0]},
        ],
    })
}

fn parse(value: &Value) -> Result<InstrumentPackDocument, SyspackError> {
    InstrumentPackDocument::parse(value.to_string().as_bytes())
}

fn assert_invalid(value: &Value, field: &str) {
    match parse(value) {
        Err(SyspackError::InvalidInstrument { field: found, .. }) => assert_eq!(found, field),
        other => panic!("expected an invalid {field}, got {other:?}"),
    }
}

#[test]
fn a_panel_parses_and_builds_every_mechanism() {
    let document = parse(&ga_panel()).unwrap();
    assert_eq!(document.instruments.len(), 5);
    let mechanisms: Vec<_> = document
        .instruments
        .iter()
        .map(|spec| spec.mechanism().unwrap())
        .collect();
    assert!(matches!(mechanisms[0], InstrumentMechanism::Airspeed(_)));
    assert!(matches!(
        mechanisms[2],
        InstrumentMechanism::VerticalSpeed(_)
    ));
    assert!(matches!(mechanisms[3], InstrumentMechanism::Tachometer(_)));
    assert!(matches!(
        mechanisms[4],
        InstrumentMechanism::PressureGauge(_)
    ));
    let InstrumentMechanism::Altimeter(altimeter) = &mechanisms[1] else {
        panic!("not an altimeter");
    };
    assert_eq!(altimeter.kollsman_pa(), mbd_airdata::ISA_P0_PA);
    let bezel = document.instrument("altimeter").unwrap().bezel;
    assert_eq!(bezel.diameter_m(), 0.079_375);
}

#[test]
fn a_panel_round_trips_through_json() {
    let document = parse(&ga_panel()).unwrap();
    let text = serde_json::to_string(&document).unwrap();
    assert_eq!(
        InstrumentPackDocument::parse(text.as_bytes()).unwrap(),
        document
    );
}

#[test]
fn rejects_unknown_fields_and_malformed_json() {
    let mut value = ga_panel();
    value["instruments"][0]["colour"] = json!("black");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InstrumentsParse(_))
    ));
    assert!(matches!(
        InstrumentPackDocument::parse(b"{"),
        Err(SyspackError::InstrumentsParse(_))
    ));
}

#[test]
fn rejects_duplicate_and_dangling_ids() {
    let mut value = ga_panel();
    value["instruments"][1]["id"] = json!("asi");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::DuplicateInstrument { id }) if id == "asi"
    ));
    let mut value = ga_panel();
    value["layout"][0]["instrument"] = json!("clock");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::UnknownLayoutInstrument { instrument, .. }) if instrument == "clock"
    ));
    let mut value = ga_panel();
    value["layout"][1]["slot"] = json!("top_left");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InvalidLayoutSlot { .. })
    ));
}

#[test]
fn rejects_a_non_monotone_scale() {
    let mut value = ga_panel();
    value["instruments"][0]["kind"]["Airspeed"]["scale"] =
        scale(&[(0.0, 0.0), (100.0, 200.0), (200.0, 150.0)]);
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InstrumentMechanism {
            instrument,
            source: AirdataError::NotIncreasing { index: 2, .. },
        }) if instrument == "asi"
    ));
}

#[test]
fn rejects_unphysical_mechanisms() {
    let mut value = ga_panel();
    value["instruments"][2]["kind"]["VerticalSpeed"]["leak_tau_s"] = json!(0.0);
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InstrumentMechanism { .. })
    ));
    let mut value = ga_panel();
    value["instruments"][1]["kind"]["Altimeter"]["kollsman"]["units"] = json!("Feet");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InstrumentMechanism {
            source: AirdataError::UnitMismatch { .. },
            ..
        })
    ));
    let mut value = ga_panel();
    value["instruments"][0]["kind"]["Airspeed"]["units"] = json!("InchesOfMercury");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InstrumentMechanism { .. })
    ));
    let mut value = ga_panel();
    value["instruments"][0]["needle"]["damping_ratio"] = json!(-1.0);
    assert!(matches!(
        parse(&value),
        Err(SyspackError::InstrumentMechanism { .. })
    ));
}

#[test]
fn rejects_bad_sources_markings_and_bezels() {
    let cases = [
        ("/instruments/0/source", json!("engine[0].rpm"), "source"),
        (
            "/instruments/3/source",
            json!("pitot_static.main"),
            "source",
        ),
        ("/instruments/3/source", json!("engine[x].rpm"), "source"),
        ("/instruments/0/markings/0/Arc/to", json!(10.0), "markings"),
        (
            "/instruments/4/bezel",
            json!({"Custom": {"diameter_m": 0.0}}),
            "bezel",
        ),
        ("/instruments/4/name", json!(""), "name"),
        ("/instruments/4/id", json!("Oil"), "id"),
    ];
    for (pointer, replacement, field) in cases {
        let mut value = ga_panel();
        *value.pointer_mut(pointer).unwrap() = replacement;
        assert_invalid(&value, field);
    }
}

#[test]
fn source_bindings_have_a_closed_grammar() {
    assert_eq!(
        SourceBinding::parse("pitot_static.left_boom"),
        Some(SourceBinding::PitotStatic {
            device: "left_boom".to_owned()
        })
    );
    assert_eq!(
        SourceBinding::parse("pitot_static.Main-Pitot"),
        Some(SourceBinding::PitotStatic {
            device: "Main-Pitot".to_owned()
        })
    );
    assert_eq!(
        SourceBinding::parse("engine[1].manifold_pressure"),
        Some(SourceBinding::Engine {
            index: 1,
            quantity: "manifold_pressure".to_owned()
        })
    );
    assert_eq!(
        SourceBinding::parse("system.vacuum.suction"),
        Some(SourceBinding::System {
            path: "vacuum.suction".to_owned()
        })
    );
    for bad in [
        "",
        "pitot_static.",
        "pitot_static.main.static",
        "engine[].rpm",
        "engine[+1].rpm",
        "engine[256].rpm",
        "engine[0]rpm",
        "system.vacuum..suction",
        "gps.altitude",
    ] {
        assert_eq!(SourceBinding::parse(bad), None, "{bad}");
    }
}
