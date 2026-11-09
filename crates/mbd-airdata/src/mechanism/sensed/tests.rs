use super::*;

fn config(unit: DialUnit, points: &[(f64, f64)], tau: f64) -> SensedGaugeConfig {
    let scale = DialScale::new(points).unwrap();
    let (min_stop, max_stop) = scale.angle_range_rad();
    SensedGaugeConfig {
        unit,
        scale,
        element_tau_s: tau,
        needle: NeedleConfig {
            natural_frequency_rad_s: 20.0,
            damping_ratio: 1.0,
            min_stop,
            max_stop,
        },
    }
}

#[test]
fn a_celsius_thermometer_reads_kelvin_through_its_bulb() {
    let c = config(DialUnit::DegreesCelsius, &[(0.0, 0.0), (100.0, 5.0)], 4.0);
    let mut oil = Thermometer::new(c, 273.15 + 20.0).unwrap();
    let settled = oil.step(0.01, 273.15 + 20.0);
    assert!((settled.indicated - 20.0).abs() < 1e-6, "{settled:?}");
    // A step to 80 °C reaches the dial only through the bulb's lag.
    let early = oil.step(1.0, 273.15 + 80.0);
    assert!(
        early.indicated > 20.0 && early.indicated < 50.0,
        "{early:?}"
    );
    for _ in 0..600 {
        oil.step(0.1, 273.15 + 80.0);
    }
    assert!((oil.step(0.1, 273.15 + 80.0).indicated - 80.0).abs() < 0.01);
    assert!(
        Thermometer::new(config(DialUnit::Knots, &[(0.0, 0.0), (1.0, 1.0)], 1.0), 0.0).is_err()
    );
}

#[test]
fn a_fuel_gauge_reads_gallons() {
    let c = config(DialUnit::UsGallons, &[(0.0, 0.0), (100.0, 4.0)], 0.0);
    let mut tank = FuelQuantityGauge::new(c, 0.0).unwrap();
    let full = 90.0 * 0.003_785_411_784;
    tank.reset(full);
    assert!((tank.step(0.01, full).indicated - 90.0).abs() < 1e-6);
    assert!(
        FuelQuantityGauge::new(config(DialUnit::Feet, &[(0.0, 0.0), (1.0, 1.0)], 0.0), 0.0)
            .is_err()
    );
}
