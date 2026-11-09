use super::*;

fn needle(min: f64, max: f64) -> NeedleConfig {
    NeedleConfig {
        natural_frequency_rad_s: 20.0,
        damping_ratio: 1.0,
        min_stop: min,
        max_stop: max,
    }
}

#[test]
fn a_manifold_pressure_gauge_reads_ambient_with_the_engine_stopped() {
    let scale = DialScale::new(&[(10.0, 0.0), (35.0, 4.0)]).unwrap();
    let config = PressureGaugeConfig {
        unit: DialUnit::InchesOfMercury,
        scale,
        line_tau_s: 0.2,
        needle: needle(0.0, 4.0),
    };
    let mut gauge = PressureGauge::new(config, 101_325.0).unwrap();
    let reading = gauge.step(1.0 / 30.0, 101_325.0);
    assert!((reading.indicated - 29.921).abs() < 1e-3, "{reading:?}");
    let wrong = PressureGaugeConfig {
        unit: DialUnit::Knots,
        ..config
    };
    assert!(PressureGauge::new(wrong, 0.0).is_err());
}

#[test]
fn an_oil_pressure_gauge_reads_psi() {
    let scale = DialScale::new(&[(0.0, 0.0), (115.0, 4.0)]).unwrap();
    let config = PressureGaugeConfig {
        unit: DialUnit::PoundsPerSquareInch,
        scale,
        line_tau_s: 0.0,
        needle: needle(0.0, 4.0),
    };
    let mut gauge = PressureGauge::new(config, 0.0).unwrap();
    let mut reading = gauge.step(0.1, 60.0 * 6_894.757);
    for _ in 0..60 {
        reading = gauge.step(0.1, 60.0 * 6_894.757);
    }
    assert!((reading.indicated - 60.0).abs() < 1e-3);
}

#[test]
fn the_drag_cup_lags_engine_speed() {
    let scale = DialScale::new(&[(0.0, 0.0), (3_500.0, 4.5)]).unwrap();
    let config = TachometerConfig {
        unit: DialUnit::RevolutionsPerMinute,
        scale,
        drag_cup_tau_s: 0.5,
        needle: needle(0.0, 4.5),
    };
    let mut tachometer = Tachometer::new(config, 0.0).unwrap();
    let mut reading = tachometer.step(0.05, 2_400.0);
    for _ in 1..10 {
        reading = tachometer.step(0.05, 2_400.0);
    }
    // One time constant after a step: 1 − 1/e of the way, given the drive
    // ramps over the first step.
    let held = 2_400.0 * (1.0 - (-0.45f64 / 0.5).exp());
    assert!(reading.commanded > held && reading.commanded < 2_400.0 * 0.7);
    for _ in 0..100 {
        reading = tachometer.step(0.05, 2_400.0);
    }
    assert!(
        (reading.indicated - 2_400.0).abs() < 0.1,
        "{reading:?} {tachometer:?}"
    );
}
