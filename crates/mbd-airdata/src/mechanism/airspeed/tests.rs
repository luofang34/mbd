use super::*;
use crate::isa::isa_atmosphere;
use crate::pitot::impact_pressure_from_cas_pa;
use crate::{PositionErrorCurve, PositionErrorTable};

const KNOT_M_S: f64 = 1852.0 / 3600.0;

fn indicator(error: PiecewiseLinear) -> AirspeedIndicator {
    let scale = DialScale::new(&[
        (0.0, 0.0),
        (40.0, 30f64.to_radians()),
        (100.0, 180f64.to_radians()),
        (200.0, 330f64.to_radians()),
    ])
    .unwrap();
    let needle = NeedleConfig {
        natural_frequency_rad_s: 15.0,
        damping_ratio: 0.8,
        min_stop: 0.0,
        max_stop: 340f64.to_radians(),
    };
    AirspeedIndicator::new(AirspeedConfig {
        unit: DialUnit::Knots,
        scale,
        instrument_error: error,
        needle,
    })
    .unwrap()
}

fn settle(asi: &mut AirspeedIndicator, total: f64, static_pa: f64) -> DialReading {
    let mut reading = asi.step(1.0 / 30.0, total, static_pa);
    for _ in 0..120 {
        reading = asi.step(1.0 / 30.0, total, static_pa);
    }
    reading
}

/// The capsule reads CAS at any altitude: 100 KCAS at 10 000 ft reads 100.
#[test]
fn reads_calibrated_airspeed_at_altitude() {
    let mut asi = indicator(PiecewiseLinear::constant(0.0));
    let static_pa = isa_atmosphere(3_048.0, 0.0).pressure_pa;
    let total = static_pa + impact_pressure_from_cas_pa(100.0 * KNOT_M_S);
    let reading = settle(&mut asi, total, static_pa);
    assert!((reading.indicated - 100.0).abs() < 1e-6, "{reading:?}");
    assert!((reading.needle_angle_rad - 180f64.to_radians()).abs() < 1e-6);
}

#[test]
fn adds_the_bench_instrument_error() {
    let error = PiecewiseLinear::new("e", &[(50.0, 1.0), (150.0, 3.0)]).unwrap();
    let mut asi = indicator(error);
    let total = 101_325.0 + impact_pressure_from_cas_pa(100.0 * KNOT_M_S);
    assert!((settle(&mut asi, total, 101_325.0).indicated - 102.0).abs() < 1e-6);
}

/// Static ports reading high make the needle read the card's IAS.
#[test]
fn shows_position_error_from_the_static_ports() {
    let card = [(50.0, 56.0), (80.0, 81.0)].map(|(i, c)| (i * KNOT_M_S, c * KNOT_M_S));
    let table =
        PositionErrorTable::new(&[PositionErrorCurve::from_ias_cas_pairs(0.0, &card).unwrap()])
            .unwrap();
    let qc = impact_pressure_from_cas_pa(56.0 * KNOT_M_S);
    let sensed_static = table.sensed_static_pa(101_325.0, qc, 0.0);
    let mut asi = indicator(PiecewiseLinear::constant(0.0));
    let reading = settle(&mut asi, 101_325.0 + qc, sensed_static);
    assert!((reading.indicated - 50.0).abs() < 1e-6, "{reading:?}");
}

#[test]
fn reverse_flow_rests_on_the_stop_and_a_length_unit_is_rejected() {
    let mut asi = indicator(PiecewiseLinear::constant(0.0));
    let reading = settle(&mut asi, 100_000.0, 101_325.0);
    assert_eq!(reading.indicated, 0.0);
    assert_eq!(reading.commanded, 0.0);
    let config = AirspeedConfig {
        unit: DialUnit::Feet,
        scale: *asi.scale(),
        instrument_error: PiecewiseLinear::constant(0.0),
        needle: NeedleConfig {
            natural_frequency_rad_s: 1.0,
            damping_ratio: 1.0,
            min_stop: 0.0,
            max_stop: 1.0,
        },
    };
    assert!(matches!(
        AirspeedIndicator::new(config),
        Err(AirdataError::UnitMismatch { .. })
    ));
}

#[test]
fn snapshot_and_rewind_replay_bit_identically() {
    let mut asi = indicator(PiecewiseLinear::constant(0.0));
    asi.reset(101_325.0 + 1_000.0, 101_325.0);
    let snapshot = asi;
    let run = |asi: &mut AirspeedIndicator| {
        (0..90)
            .map(|k| {
                asi.step(
                    1.0 / 30.0,
                    101_325.0 + 1_000.0 + 20.0 * f64::from(k),
                    101_325.0,
                )
            })
            .last()
    };
    let first = run(&mut asi);
    let after = asi;
    asi = snapshot;
    assert_eq!(run(&mut asi), first);
    assert_eq!(asi, after);
}
