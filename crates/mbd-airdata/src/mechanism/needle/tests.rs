use super::*;

fn needle(omega: f64, zeta: f64) -> NeedleDynamics {
    let config = NeedleConfig {
        natural_frequency_rad_s: omega,
        damping_ratio: zeta,
        min_stop: -10.0,
        max_stop: 10.0,
    };
    NeedleDynamics::new(config, 0.0).unwrap()
}

/// Peak of a unit step response.
fn peak(omega: f64, zeta: f64, dt: f64, steps: u32) -> f64 {
    let mut needle = needle(omega, zeta);
    (0..steps).map(|_| needle.step(dt, 1.0)).fold(0.0, f64::max)
}

/// Overshoot of a second-order step response is `exp(−πζ/√(1−ζ²))`.
#[test]
fn overshoot_follows_the_damping_ratio() {
    for zeta in [0.2_f64, 0.5, 0.7] {
        let expected = (-core::f64::consts::PI * zeta / (1.0 - zeta * zeta).sqrt()).exp();
        let overshoot = peak(20.0, zeta, 1.0 / 600.0, 1_200) - 1.0;
        assert!((overshoot - expected).abs() < 2e-3, "ζ {zeta}: {overshoot}");
    }
    assert!(peak(20.0, 1.0, 1.0 / 600.0, 1_200) <= 1.0 + 1e-9);
}

#[test]
fn a_stiff_light_needle_is_stable_at_thirty_hertz() {
    let mut needle = needle(400.0, 0.3);
    for _ in 0..300 {
        let position = needle.step(1.0 / 30.0, 1.0);
        assert!(position.abs() < 2.0);
    }
    assert!((needle.position() - 1.0).abs() < 1e-6);
    assert!(needle.rate().abs() < 1e-3);
}

#[test]
fn the_stops_hold_the_needle() {
    let mut needle = needle(20.0, 0.1);
    for _ in 0..100 {
        assert!(needle.step(0.01, 50.0) <= 10.0);
    }
    assert_eq!(needle.position(), 10.0);
    assert!(needle.rate() <= 0.0);
    needle.reset(-99.0);
    assert_eq!(needle.position(), -10.0);
}

#[test]
fn bad_inputs_leave_the_needle_alone() {
    let mut needle = needle(20.0, 0.5);
    needle.step(0.1, 3.0);
    let before = needle;
    needle.step(f64::NAN, 1.0);
    needle.step(0.1, f64::INFINITY);
    needle.step(-0.1, 1.0);
    needle.reset(f64::NAN);
    assert_eq!(needle, before);
}

#[test]
fn rejects_bad_configs() {
    let good = NeedleConfig {
        natural_frequency_rad_s: 10.0,
        damping_ratio: 0.7,
        min_stop: 0.0,
        max_stop: 1.0,
    };
    for bad in [
        NeedleConfig {
            natural_frequency_rad_s: 0.0,
            ..good
        },
        NeedleConfig {
            damping_ratio: -0.1,
            ..good
        },
        NeedleConfig {
            max_stop: -1.0,
            ..good
        },
        NeedleConfig {
            min_stop: f64::NAN,
            ..good
        },
    ] {
        assert!(NeedleDynamics::new(bad, 0.0).is_err(), "{bad:?}");
    }
}

#[test]
fn identical_inputs_give_bit_identical_state() {
    let mut a = needle(35.0, 0.4);
    let mut b = needle(35.0, 0.4);
    for k in 0..500 {
        let command = (f64::from(k) * 0.05).sin();
        a.step(1.0 / 30.0, command);
        b.step(1.0 / 30.0, command);
    }
    assert_eq!(a.position().to_bits(), b.position().to_bits());
    assert_eq!(a, b);
}
