use serde_json::{Value, json};

use crate::instrument_pack::tests::ga_panel;
use crate::{InstrumentMechanism, InstrumentPackDocument, SyspackError};

fn scale(points: &[(f64, f64)]) -> Value {
    points
        .iter()
        .map(|&(value, angle_deg)| json!({"value": value, "angle_deg": angle_deg}))
        .collect()
}

/// A P-38G engine gage unit: oil temperature with oil and fuel pressure.
fn engine_gage_unit() -> Value {
    json!({
        "id": "engine_gage_unit_0",
        "name": "Engine gage unit, left engine",
        "bezel": "ThreeAndOneEighthInch",
        "source": "engine[0].oil_temperature",
        "needle": {"natural_frequency_rad_s": 8.0, "damping_ratio": 0.8},
        "kind": {"Thermometer": {
            "units": "DegreesCelsius",
            "scale": scale(&[(0.0, -60.0), (100.0, 60.0)]),
            "bulb_tau_s": 6.0,
        }},
        "dial": {"ring_centre": [0.0, -0.3], "ring_radius": 0.6},
        "pointers": [
            {
                "id": "oil_pressure",
                "source": "engine[0].oil_pressure",
                "kind": {"PressureGauge": {
                    "units": "PoundsPerSquareInch",
                    "scale": scale(&[(0.0, 120.0), (200.0, 240.0)]),
                    "line_tau_s": 0.3,
                }},
                "dial": {"ring_centre": [-0.4, 0.3], "ring_radius": 0.5},
            },
            {
                "id": "fuel_pressure",
                "source": "engine[0].fuel_pressure",
                "kind": {"PressureGauge": {
                    "units": "PoundsPerSquareInch",
                    "scale": scale(&[(0.0, 150.0), (25.0, 210.0)]),
                    "line_tau_s": 0.3,
                }},
            },
        ],
    })
}

fn with(instrument: Value) -> Value {
    let mut panel = ga_panel();
    if let Some(list) = panel["instruments"].as_array_mut() {
        list.push(instrument);
    }
    panel
}

fn parse(value: &Value) -> Result<InstrumentPackDocument, SyspackError> {
    InstrumentPackDocument::parse(value.to_string().as_bytes())
}

#[test]
fn a_case_with_three_pointers_builds_a_mechanism_for_each() {
    let document = parse(&with(engine_gage_unit())).unwrap();
    let unit = document.instrument("engine_gage_unit_0").unwrap();
    assert!(matches!(
        unit.mechanism().unwrap(),
        InstrumentMechanism::Thermometer(_)
    ));
    let further = unit.pointer_mechanisms().unwrap();
    assert_eq!(further.len(), 2);
    assert!(
        further
            .iter()
            .all(|m| matches!(m, InstrumentMechanism::PressureGauge(_)))
    );
    let InstrumentMechanism::Thermometer(oil) = unit.mechanism().unwrap() else {
        panic!("not a thermometer");
    };
    let mut oil = *oil;
    oil.reset(273.15 + 70.0);
    assert!((oil.step(0.01, 273.15 + 70.0).indicated - 70.0).abs() < 1e-6);
}

#[test]
fn a_dual_fuel_gauge_reads_gallons_per_tank() {
    let tank = |id: &str, source: &str| {
        json!({"id": id, "source": source, "kind": {"FuelQuantity": {
            "units": "UsGallons",
            "scale": scale(&[(0.0, -50.0), (90.0, 50.0)]),
            "transmitter_tau_s": 2.0,
        }}})
    };
    let mut fuel = tank("fuel_front", "system.fuel.front_left");
    fuel["name"] = json!("Fuel quantity, front tanks");
    fuel["bezel"] = json!("TwoAndOneQuarterInch");
    fuel["needle"] = json!({"natural_frequency_rad_s": 6.0, "damping_ratio": 0.9});
    fuel["pointers"] = json!([tank("right", "system.fuel.front_right")]);
    let document = parse(&with(fuel)).unwrap();
    let gauge = document.instrument("fuel_front").unwrap();
    assert!(matches!(
        gauge.pointer_mechanisms().unwrap()[0],
        InstrumentMechanism::FuelQuantity(_)
    ));
}

#[test]
fn rejects_pointers_a_case_cannot_hold() {
    let pointer = |id: &str| {
        engine_gage_unit()["pointers"][0]
            .clone()
            .as_object()
            .map(|o| {
                let mut o = o.clone();
                o.insert("id".to_owned(), json!(id));
                Value::Object(o)
            })
    };
    let mut too_many = engine_gage_unit();
    too_many["pointers"] = json!([pointer("a"), pointer("b"), pointer("c"), pointer("d")]);
    let mut duplicate = engine_gage_unit();
    duplicate["pointers"] = json!([pointer("a"), pointer("a")]);
    let mut pitot = engine_gage_unit();
    pitot["pointers"][0]["source"] = json!("pitot_static.main");
    let mut wrong_unit = engine_gage_unit();
    wrong_unit["kind"]["Thermometer"]["units"] = json!("Knots");
    let mut off_face = engine_gage_unit();
    off_face["dial"]["ring_centre"] = json!([1.5, 0.0]);
    for (case, value) in [
        ("too many", too_many),
        ("duplicate", duplicate),
        ("pitot", pitot),
        ("wrong unit", wrong_unit),
        ("off face", off_face),
    ] {
        assert!(parse(&with(value)).is_err(), "{case}");
    }
    let mut altimeter = ga_panel();
    altimeter["instruments"][1]["pointers"] = json!([pointer("a")]);
    assert!(parse(&altimeter).is_err());
}

/// An instrument drawn from the state directly validates, and its mechanism
/// says so instead of modelling one.
#[test]
fn a_direct_instrument_has_no_modelled_mechanism() {
    let horizon = json!({
        "id": "gyro_horizon",
        "name": "Sperry gyro horizon",
        "bezel": "ThreeAndOneEighthInch",
        "source": "system.attitude",
        "needle": {"natural_frequency_rad_s": 8.0, "damping_ratio": 0.8},
        "kind": {"Direct": {"instrument": "GyroHorizon"}},
    });
    let document = parse(&with(horizon)).unwrap();
    let horizon = document.instrument("gyro_horizon").unwrap();
    assert_eq!(
        horizon.mechanism().unwrap(),
        InstrumentMechanism::Direct(crate::DirectInstrument::GyroHorizon)
    );
}
