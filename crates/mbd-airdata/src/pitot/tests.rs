use super::*;
use crate::isa::isa_atmosphere;

const KNOT_M_S: f64 = 1852.0 / 3600.0;

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} differs from {expected} by more than {tolerance}"
    );
}

/// Pitot-to-static ratios from NACA Report 1135 (Ames Research Staff, 1953):
/// isentropic `p/p0 = 0.8430` at Mach 0.5, normal-shock `p02/p1 = 5.640` at
/// Mach 2.
#[test]
fn matches_naca_1135_pressure_ratios() {
    assert_close(impact_pressure_pa(1.0, 0.5) + 1.0, 1.0 / 0.8430, 2e-4);
    assert_close(impact_pressure_pa(1.0, 2.0) + 1.0, 5.640, 1e-3);
}

/// The CAS law: 100 kt is 1630 Pa of impact pressure, and a CAS equal to the
/// sea-level speed of sound is `qc = (1.2^3.5 − 1) p0 ≈ 0.8929 p0`.
#[test]
fn matches_the_published_cas_law() {
    assert_close(impact_pressure_from_cas_pa(100.0 * KNOT_M_S), 1630.3, 1.0);
    assert_close(
        impact_pressure_from_cas_pa(ISA_SPEED_OF_SOUND0_M_S) / ISA_P0_PA,
        0.8929,
        1e-4,
    );
}

#[test]
fn the_two_laws_meet_at_mach_one() {
    let below = impact_pressure_pa(ISA_P0_PA, 1.0 - 1e-9);
    let above = impact_pressure_pa(ISA_P0_PA, 1.0 + 1e-9);
    assert_close(below, above, 1e-3);
    let qc = impact_pressure_pa(ISA_P0_PA, 1.0);
    assert_close(
        mach_from_impact_pressure(qc * (1.0 - 1e-9), ISA_P0_PA),
        1.0,
        1e-8,
    );
    assert_close(
        mach_from_impact_pressure(qc * (1.0 + 1e-9), ISA_P0_PA),
        1.0,
        1e-8,
    );
}

#[test]
fn mach_round_trips_in_both_regimes() {
    let static_pa = isa_atmosphere(9_000.0, 0.0).pressure_pa;
    for mach in [0.05, 0.3, 0.8, 0.99, 1.01, 1.5, 2.5, 5.0, 10.0] {
        let qc = impact_pressure_pa(static_pa, mach);
        assert_close(mach_from_impact_pressure(qc, static_pa), mach, mach * 1e-12);
    }
    assert_eq!(mach_from_impact_pressure(0.0, static_pa), 0.0);
    assert_eq!(mach_from_impact_pressure(-5.0, static_pa), 0.0);
    assert_eq!(mach_from_impact_pressure(5.0, 0.0), 0.0);
    assert_eq!(mach_from_impact_pressure(f64::NAN, static_pa), 0.0);
}

#[test]
fn cas_round_trips_and_equals_tas_at_sea_level() {
    for cas_kt in [40.0, 120.0, 400.0, 661.5, 900.0] {
        let cas = cas_kt * KNOT_M_S;
        assert_close(
            cas_from_impact_pressure_m_s(impact_pressure_from_cas_pa(cas)),
            cas,
            1e-9,
        );
    }
    let mach = 0.4;
    let qc = impact_pressure_pa(ISA_P0_PA, mach);
    let tas = tas_m_s(mach, 288.15);
    assert_close(cas_from_impact_pressure_m_s(qc), tas, 1e-3);
    assert_close(eas_m_s(mach, ISA_P0_PA), tas, 1e-3);
}

/// At altitude CAS exceeds EAS by the compressibility correction that an
/// EAS-only model omits, and the excess grows with Mach number.
#[test]
fn cas_exceeds_eas_at_altitude() {
    let air = isa_atmosphere(9_144.0, 0.0);
    let excess_kt = |mach: f64| {
        let qc = impact_pressure_pa(air.pressure_pa, mach);
        (cas_from_impact_pressure_m_s(qc) - eas_m_s(mach, air.pressure_pa)) / KNOT_M_S
    };
    assert!(excess_kt(0.3) > 0.0);
    assert!(excess_kt(0.8) > 10.0 * excess_kt(0.3).min(1.0));
    assert!(excess_kt(0.8) > excess_kt(0.5));
}
