use super::*;
use crate::isa::isa_atmosphere;

const FPM_M_S: f64 = 0.3048 / 60.0;
const TAU_S: f64 = 2.5;
const DT: f64 = 1.0 / 30.0;

fn vsi(static_pa: f64) -> VerticalSpeedIndicator {
    // Expanded near zero, as real VSI dials are.
    let scale = DialScale::new(&[
        (-2_000.0, (-170f64).to_radians()),
        (-1_000.0, (-110f64).to_radians()),
        (0.0, 0.0),
        (1_000.0, 110f64.to_radians()),
        (2_000.0, 170f64.to_radians()),
    ])
    .unwrap();
    let config = VerticalSpeedConfig {
        unit: DialUnit::FeetPerMinute,
        scale,
        leak_tau_s: TAU_S,
        needle: NeedleConfig {
            natural_frequency_rad_s: 10.0,
            damping_ratio: 0.8,
            min_stop: (-175f64).to_radians(),
            max_stop: 175f64.to_radians(),
        },
    };
    VerticalSpeedIndicator::new(config, static_pa).unwrap()
}

/// Flies a constant-rate climb in the standard atmosphere and returns the
/// readings, one per step.
fn climb(rate_fpm: f64, seconds: f64) -> Vec<DialReading> {
    let start_m = 1_000.0;
    let mut indicator = vsi(isa_atmosphere(start_m, 0.0).pressure_pa);
    let steps = (seconds / DT).round() as u32;
    (1..=steps)
        .map(|k| {
            let altitude_m = start_m + rate_fpm * FPM_M_S * f64::from(k) * DT;
            indicator.step(DT, isa_atmosphere(altitude_m, 0.0).pressure_pa)
        })
        .collect()
}

#[test]
fn reads_the_true_rate_once_settled() {
    for rate_fpm in [500.0, 1_000.0, -1_500.0] {
        let readings = climb(rate_fpm, 30.0);
        let last = readings.last().unwrap();
        assert!(
            (last.indicated - rate_fpm).abs() < 0.003 * rate_fpm.abs(),
            "{rate_fpm}: {last:?}"
        );
    }
}

/// The capsule's differential builds as `1 − exp(−t/τ)` after a climb
/// begins, so the needle needs about three time constants (7.5 s here) to
/// reach 95 % of the rate.
#[test]
fn lags_with_the_leak_time_constant() {
    let readings = climb(1_000.0, 10.0);
    let at = |seconds: f64| readings[(seconds / DT).round() as usize - 1].commanded;
    let expected = |seconds: f64| 1_000.0 * (1.0 - (-seconds / TAU_S).exp());
    for seconds in [1.0, TAU_S, 5.0, 3.0 * TAU_S] {
        assert!(
            (at(seconds) - expected(seconds)).abs() < 2.0,
            "{seconds}: {}",
            at(seconds)
        );
    }
    assert!(at(3.0 * TAU_S) > 940.0);
}

/// A sudden static drop that is not a climb, such as opening the alternate
/// static source inside a cabin at lower pressure, swings the needle into a
/// climb and lets it decay back to level.
#[test]
fn a_static_step_swings_the_needle_and_decays() {
    let static_pa = isa_atmosphere(1_000.0, 0.0).pressure_pa;
    let mut indicator = vsi(static_pa);
    let dropped = static_pa - 50.0;
    let swing: Vec<f64> = (0..300)
        .map(|_| indicator.step(DT, dropped).indicated)
        .collect();
    let peak = swing.iter().copied().fold(f64::MIN, f64::max);
    assert!(peak > 250.0, "{peak}");
    assert!(swing.last().unwrap().abs() < 10.0);
}

#[test]
fn identical_runs_are_bit_identical() {
    let a = climb(800.0, 5.0);
    let b = climb(800.0, 5.0);
    assert_eq!(a, b);
}
