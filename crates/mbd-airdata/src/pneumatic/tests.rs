use super::*;

#[test]
fn reaches_one_minus_one_over_e_after_one_time_constant_of_a_held_input() {
    let mut held = PneumaticLag::new(2.0, 0.0);
    held.input = 1.0;
    let after = held.step(2.0, 1.0);
    assert!((after - (1.0 - (-1.0f64).exp())).abs() < 1e-15);
}

#[test]
fn step_size_does_not_change_a_ramp() {
    let mut coarse = PneumaticLag::new(0.7, 100.0);
    let mut fine = coarse;
    coarse.step(0.3, 50.0);
    for k in 1..=30 {
        fine.step(0.01, 100.0 - 50.0 * f64::from(k) / 30.0);
    }
    assert!((coarse.value() - fine.value()).abs() < 1e-11);
}

#[test]
fn a_steady_ramp_lags_by_exactly_tau() {
    let tau = 2.5;
    let slope = -12.0;
    let mut lag = PneumaticLag::new(tau, 0.0);
    // Settled on the ramp: the lagged end trails by tau * slope.
    lag.value = -tau * slope;
    for k in 1..=100 {
        let input = slope * f64::from(k) / 30.0;
        let value = lag.step(1.0 / 30.0, input);
        assert!((input - value - tau * slope).abs() < 1e-11);
    }
}

#[test]
fn no_lag_passes_through_and_zero_dt_holds() {
    let mut direct = PneumaticLag::new(0.0, 1.0);
    assert_eq!(direct.step(0.01, 7.0), 7.0);
    assert_eq!(PneumaticLag::new(f64::NAN, 0.0).tau_s(), 0.0);
    let mut lag = PneumaticLag::new(1.0, 3.0);
    assert_eq!(lag.step(0.0, 9.0), 3.0);
    assert_eq!(lag.step(-1.0, 9.0), 3.0);
    assert_eq!(lag.step(f64::NAN, 9.0), 3.0);
}
