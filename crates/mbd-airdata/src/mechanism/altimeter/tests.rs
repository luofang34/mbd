use super::*;
use crate::isa::{ISA_P0_PA, isa_atmosphere};

const FOOT_M: f64 = 0.3048;

fn altimeter(kollsman_pa: f64, static_pa: f64, aneroid_tau_s: f64) -> Altimeter {
    let config = AltimeterConfig {
        unit: DialUnit::Feet,
        kollsman: KollsmanWindow::INCHES_OF_MERCURY,
        aneroid_tau_s,
        needle: NeedleConfig {
            natural_frequency_rad_s: 12.0,
            damping_ratio: 0.9,
            min_stop: -1_500.0,
            max_stop: 35_000.0,
        },
    };
    Altimeter::new(config, kollsman_pa, static_pa).unwrap()
}

fn settle(altimeter: &mut Altimeter, static_pa: f64) -> AltimeterReading {
    let mut reading = altimeter.step(1.0 / 30.0, static_pa);
    for _ in 0..600 {
        reading = altimeter.step(1.0 / 30.0, static_pa);
    }
    reading
}

/// The NOAA/NWS altimeter-setting reduction (ASOS; Smithsonian Meteorological
/// Tables): `AS = (p^0.190284 + 8.4184960528e-5 · H)^(1/0.190284)`, hPa and
/// metres. An altimeter set to it at the station must read field elevation.
#[test]
fn reads_field_elevation_when_set_to_qnh() {
    for (station_hpa, elevation_m) in [(950.0_f64, 500.0_f64), (850.0, 1_500.0), (1_000.0, 100.0)] {
        let setting_hpa =
            (station_hpa.powf(0.190_284) + 8.418_496_052_8e-5 * elevation_m).powf(1.0 / 0.190_284);
        let mut altimeter = altimeter(setting_hpa * 100.0, station_hpa * 100.0, 0.0);
        let reading = settle(&mut altimeter, station_hpa * 100.0);
        let expected_ft = elevation_m / FOOT_M;
        assert!(
            (reading.indicated - expected_ft).abs() < 1.0,
            "{reading:?} vs {expected_ft}"
        );
    }
}

/// Set to 29.92 inHg (1013.25 hPa) it reads pressure altitude: 10 000 ft at
/// 69.68 kPa.
#[test]
fn reads_pressure_altitude_at_standard_setting() {
    let static_pa = isa_atmosphere(3_048.0, 0.0).pressure_pa;
    let mut altimeter = altimeter(ISA_P0_PA, static_pa, 0.0);
    let reading = settle(&mut altimeter, static_pa);
    assert!((reading.indicated - 10_000.0).abs() < 1e-6);
}

#[test]
fn the_hands_turn_at_their_gear_ratios() {
    let static_pa = isa_atmosphere(12_345.0 * FOOT_M, 0.0).pressure_pa;
    let mut altimeter = altimeter(ISA_P0_PA, static_pa, 0.0);
    let reading = settle(&mut altimeter, static_pa);
    let turns = |rad: f64| rad / TAU;
    assert!((turns(reading.hundreds_hand_rad) - 0.345).abs() < 1e-6);
    assert!((turns(reading.thousands_hand_rad) - 0.2345).abs() < 1e-7);
    assert!((turns(reading.ten_thousands_pointer_rad) - 0.123_45).abs() < 1e-8);
    // Below sea level the long hand sits short of twelve o'clock.
    assert!((hand_angle(-100.0, 1_000.0) / TAU - 0.9).abs() < 1e-12);
}

#[test]
fn the_kollsman_knob_stops_at_the_window() {
    let mut altimeter = altimeter(ISA_P0_PA, ISA_P0_PA, 0.0);
    let low = altimeter.set_kollsman(90_000.0);
    assert!((low / 3_386.389 - 28.10).abs() < 1e-12);
    let high = altimeter.set_kollsman(110_000.0);
    assert!((high / 3_386.389 - 31.00).abs() < 1e-12);
    assert_eq!(altimeter.set_kollsman(f64::NAN), high);
    // Raising the setting by an inch raises the reading by about 900 ft.
    altimeter.set_kollsman(30.92 * 3_386.389);
    let up = settle(&mut altimeter, 29.92 * 3_386.389).indicated;
    assert!((880.0..950.0).contains(&up), "{up}");
}

#[test]
fn the_aneroid_lags_a_pressure_step() {
    let start = isa_atmosphere(0.0, 0.0).pressure_pa;
    let higher = isa_atmosphere(304.8, 0.0).pressure_pa;
    let mut altimeter = altimeter(ISA_P0_PA, start, 1.0);
    let first = altimeter.step(1.0 / 30.0, higher);
    assert!(first.indicated < 100.0, "{first:?} {altimeter:?}");
    let settled = settle(&mut altimeter, higher).indicated;
    assert!((settled - 1_000.0).abs() < 0.5, "{settled}");
}

#[test]
fn rejects_a_speed_unit() {
    let config = AltimeterConfig {
        unit: DialUnit::Knots,
        kollsman: KollsmanWindow::HECTOPASCALS,
        aneroid_tau_s: 0.0,
        needle: NeedleConfig {
            natural_frequency_rad_s: 1.0,
            damping_ratio: 1.0,
            min_stop: 0.0,
            max_stop: 1.0,
        },
    };
    assert!(Altimeter::new(config, ISA_P0_PA, ISA_P0_PA).is_err());
}
