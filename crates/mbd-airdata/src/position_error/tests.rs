use super::*;
use crate::isa::ISA_P0_PA;
use crate::pitot::impact_pressure_pa;

const KNOT_M_S: f64 = 1852.0 / 3600.0;

/// A calibration card in the style of a light-aircraft POH: at low speed the
/// static ports read high, so IAS is below CAS.
fn card() -> PositionErrorCurve {
    let pairs = [(50.0, 56.0), (80.0, 81.0), (120.0, 119.0)]
        .map(|(ias, cas)| (ias * KNOT_M_S, cas * KNOT_M_S));
    PositionErrorCurve::from_ias_cas_pairs(0.0, &pairs).unwrap()
}

#[test]
fn ias_below_cas_means_the_ports_read_high() {
    let curve = card();
    assert!(curve.ratio(56.0 * KNOT_M_S) > 0.0);
    assert!(curve.ratio(119.0 * KNOT_M_S) < 0.0);
}

/// Feeding the sensed static pressure through the CAS law reproduces the
/// card's IAS: the sign convention closes the loop.
#[test]
fn sensed_pressures_reproduce_the_card() {
    let table = PositionErrorTable::new(&[card()]).unwrap();
    for (ias_kt, cas_kt) in [(50.0, 56.0), (80.0, 81.0), (120.0, 119.0)] {
        let qc = crate::pitot::impact_pressure_from_cas_pa(cas_kt * KNOT_M_S);
        let total = ISA_P0_PA + qc;
        let sensed = table.sensed_static_pa(ISA_P0_PA, qc, 0.0);
        let ias = cas_from_impact_pressure_m_s(total - sensed) / KNOT_M_S;
        assert!((ias - ias_kt).abs() < 1e-9, "{ias} vs {ias_kt}");
    }
}

#[test]
fn interpolates_between_flap_settings_and_holds_beyond() {
    let up = PositionErrorCurve::new(0.0, &[(0.0, 0.01)]).unwrap();
    let down = PositionErrorCurve::new(30.0, &[(0.0, 0.05)]).unwrap();
    let table = PositionErrorTable::new(&[up, down]).unwrap();
    assert!((table.ratio(15.0, 30.0) - 0.03).abs() < 1e-15);
    assert_eq!(table.ratio(-5.0, 30.0), 0.01);
    assert_eq!(table.ratio(40.0, 30.0), 0.05);
    assert!(matches!(
        PositionErrorTable::new(&[down, up]),
        Err(AirdataError::NotIncreasing { index: 1, .. })
    ));
}

#[test]
fn no_error_senses_free_stream() {
    let qc = impact_pressure_pa(ISA_P0_PA, 0.3);
    assert_eq!(
        PositionErrorTable::none().sensed_static_pa(ISA_P0_PA, qc, 10.0),
        ISA_P0_PA
    );
}
