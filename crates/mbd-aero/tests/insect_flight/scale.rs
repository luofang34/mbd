//! Scale check: the same nondimensional flapping system a hundred times
//! longer, with frequency, viscosity and gravity scaled to keep wing speed,
//! Reynolds number, mass ratios and `g/(f²R)`, flies the same
//! nondimensional trajectory under the same nondimensional lift.

use std::io::Write;

use mbd_multibody::Rk4;

use super::fly::FlySpec;
use super::free_flight::fly_free;

#[test]
fn a_hundredfold_fly_repeats_the_nondimensional_flight() {
    let small = FlySpec::drosophila();
    let large = small.scaled(100.0);
    let beats = 4;
    let a = fly_free(small, &mut Rk4::new(), beats, true);
    let b = fly_free(large, &mut Rk4::new(), beats, true);
    let lift_unit =
        |s: &FlySpec| s.air_density_kg_m3 * s.frequency_hz.powi(2) * s.wing_length_m.powi(4);
    let (ua, ub) = (lift_unit(&small), lift_unit(&large));
    let peak = a.lift.iter().fold(0.0_f64, |m, l| m.max((l / ua).abs()));
    let lift_gap = a
        .lift
        .iter()
        .zip(&b.lift)
        .map(|(x, y)| (x / ua - y / ub).abs())
        .fold(0.0_f64, f64::max);
    let path_gap = a
        .com
        .iter()
        .zip(&b.com)
        .map(|(x, y)| (*x / small.wing_length_m - *y / large.wing_length_m).length())
        .fold(0.0_f64, f64::max);
    writeln!(
        std::io::stderr(),
        "scale ×100: max nondimensional lift gap {:.2e} of peak {peak:.3}, path gap {:.2e} wing lengths",
        lift_gap / peak,
        path_gap,
    )
    .ok();
    // Similarity is exact; the two runs differ only by rounding of
    // numbers a factor 10⁶–10¹² apart in magnitude: 1e-12.
    assert!(lift_gap < 1e-12 * peak, "{lift_gap}");
    assert!(path_gap < 1e-12, "{path_gap}");
}
