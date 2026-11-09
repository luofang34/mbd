use super::*;

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} differs from {expected} by more than {tolerance}"
    );
}

/// Pressure, temperature and density at geopotential altitudes, from
/// U.S. Standard Atmosphere 1976 (NOAA-S/T 76-1562), Table I.
#[test]
fn matches_us_1976_table_points() {
    let points = [
        // (altitude m, pressure Pa, temperature K, density kg/m³)
        (0.0, 101_325.0, 288.15, 1.2250),
        (3_048.0, 69_682.0, 268.338, 0.904_637),
        (5_000.0, 54_019.9, 255.65, 0.736_116),
        (11_000.0, 22_632.1, 216.65, 0.363_918),
        (20_000.0, 5_474.89, 216.65, 0.088_034_8),
        (32_000.0, 868.019, 228.65, 0.013_225),
        (47_000.0, 110.906, 270.65, 0.001_427_5),
    ];
    for (altitude_m, pressure_pa, temperature_k, density) in points {
        let air = isa_atmosphere(altitude_m, 0.0);
        assert_close(air.pressure_pa, pressure_pa, pressure_pa * 2e-5);
        assert_close(air.temperature_k, temperature_k, 1e-3);
        assert_close(air.density_kg_m3, density, density * 1e-4);
    }
}

#[test]
fn sea_level_speed_of_sound_is_the_cas_reference() {
    assert_close(speed_of_sound_m_s(ISA_T0_K), ISA_SPEED_OF_SOUND0_M_S, 1e-3);
    // 295.07 m/s in the stratosphere, U.S. Standard Atmosphere 1976.
    assert_close(
        isa_atmosphere(11_000.0, 0.0).speed_of_sound_m_s,
        295.07,
        0.01,
    );
}

#[test]
fn pressure_altitude_inverts_the_standard_in_every_layer() {
    for altitude_m in [
        -500.0, 0.0, 3_048.0, 10_999.0, 15_000.0, 25_000.0, 40_000.0, 50_000.0,
    ] {
        let pressure_pa = isa_atmosphere(altitude_m, 0.0).pressure_pa;
        assert_close(pressure_altitude_m(pressure_pa), altitude_m, 1e-6);
    }
}

#[test]
fn pressure_is_continuous_at_layer_boundaries() {
    for boundary_m in [11_000.0, 20_000.0, 32_000.0, 47_000.0] {
        let below = isa_atmosphere(boundary_m - 1e-6, 0.0).pressure_pa;
        let above = isa_atmosphere(boundary_m + 1e-6, 0.0).pressure_pa;
        assert_close(below, above, below * 1e-9);
    }
}

#[test]
fn a_hot_day_keeps_pressure_and_thins_the_air() {
    let hot = isa_atmosphere(0.0, 15.0);
    assert_eq!(hot.pressure_pa, ISA_P0_PA);
    assert_close(hot.temperature_k, 303.15, 1e-12);
    assert_close(hot.density_kg_m3, 1.164_38, 1e-5);
}

#[test]
fn the_pressure_gradient_is_hydrostatic() {
    // dp/dh = -rho g0: -12.013 Pa/m at sea level.
    assert_close(
        pressure_lapse_pa_m(ISA_P0_PA),
        -ISA_RHO0_KG_M3 * ISA_G0_M_S2,
        1e-3,
    );
    let air = isa_atmosphere(6_000.0, 0.0);
    let step_m = 0.5;
    let numeric = (isa_atmosphere(6_000.0 + step_m, 0.0).pressure_pa
        - isa_atmosphere(6_000.0 - step_m, 0.0).pressure_pa)
        / (2.0 * step_m);
    assert_close(pressure_lapse_pa_m(air.pressure_pa), numeric, 1e-6);
}

#[test]
fn geopotential_altitude_is_slightly_below_geometric() {
    // 11 km geopotential is 11 019 m geometric in the 1976 standard.
    assert_close(geopotential_altitude_m(11_019.1), 11_000.0, 0.1);
}
