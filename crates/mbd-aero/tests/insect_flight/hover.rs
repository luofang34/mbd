//! Tethered hovering: the body held still, the wings on their prescribed
//! angles, the cycle-averaged circulatory force and power.
//!
//! The entrained-air (added-mass) force has zero cycle mean here: for a
//! still body and periodic wing motion the air's impulse is periodic, and
//! the force is its rate of change (Newman 1977, §4.13).

use std::io::Write;

use mbd_multibody::{FloatingBaseState, Workspace};

use super::fly::{Fly, FlySpec, FlyWorld};
use super::probe::{self, WingSample};

/// Samples per wingbeat; the integrand is periodic and smooth, so the
/// midpoint rule converges spectrally and this is far beyond the tolerance.
const SAMPLES: u32 = 2_000;

/// Cycle means of the circulatory wing loads over one wingbeat.
pub fn cycle_mean(spec: FlySpec) -> WingSample {
    let fly = Fly::new(spec);
    let model = probe::model(&fly);
    let elements = probe::elements(&fly, true, Vec::new());
    let world = FlyWorld::new(&spec);
    let mut mean = WingSample::default();
    let mut work = Workspace::new();
    for k in 0..SAMPLES {
        let t = (f64::from(k) + 0.5) / f64::from(SAMPLES) * spec.period_s();
        let base = FloatingBaseState::AT_REST;
        let state = probe::state_at(&fly, &model, &elements, &world, &base, t);
        let kin = model.kinematics(&state, &mut work).unwrap();
        let s = probe::wings(&fly, &kin, &world);
        mean.force += s.force;
        mean.translational += s.translational;
        mean.moment_origin += s.moment_origin;
        mean.power_w += s.power_w;
        mean.reynolds += s.reynolds;
    }
    let n = f64::from(SAMPLES);
    WingSample {
        force: mean.force / n,
        translational: mean.translational / n,
        moment_origin: mean.moment_origin / n,
        power_w: mean.power_w / n,
        reynolds: mean.reynolds / n,
        ..WingSample::default()
    }
}

#[test]
fn hovering_wings_support_the_body_at_the_published_power() {
    let spec = FlySpec::drosophila();
    let mean = cycle_mean(spec);
    let weight = spec.weight_n();
    let lift_ratio = mean.force.z / weight;
    let translational_ratio = mean.translational.z / weight;
    let power = mean.power_w / spec.total_mass_kg();
    writeln!(
        std::io::stderr(),
        "hover: lift {:.2} µN, lift/weight {lift_ratio:.3}, translational/weight \
         {translational_ratio:.3}, power {power:.1} W/kg, element-mean Re {:.0}, \
         horizontal {:?} N",
        mean.force.z * 1e6,
        mean.reynolds,
        mean.force.truncate()
    )
    .ok();
    // Fry et al. 2005 (p. 2310): for the measured kinematics of hovering
    // flies the translational quasi-steady term gives 89 % of the force
    // they produce, which in hover equals their weight. These idealised
    // kinematics (sinusoidal stroke, a 45° mid-stroke incidence near the
    // C_L peak, no wake loss) predict 1.37 times the weight instead: the
    // literature ratio of about 0.8–1.0 is not reproduced, and this bound
    // only records that the prediction supports the weight without
    // exceeding it by half.
    assert!((1.0..=1.5).contains(&lift_ratio), "{lift_ratio}");
    // Re from the mean wingtip speed, 2ΦfR, and mean chord: for Fry et
    // al.'s mean kinematics this definition gives their tabulated
    // 153 ± 17 (Table 1), so the operating point matches theirs.
    let mean_chord = spec.wing_area_m2 / spec.wing_length_m;
    let tip_speed = 2.0 * spec.stroke_amplitude_rad * spec.frequency_hz * spec.wing_length_m;
    let re = spec.air_density_kg_m3 * tip_speed * mean_chord / spec.air_viscosity_pa_s;
    assert!((136.0..=170.0).contains(&re), "{re}");
    // Fry et al. 2005 Table 1: aerodynamic power 97 ± 27 W/kg of flight
    // muscle, range 50–130 over six flies, with muscle 30 % of body mass:
    // 15–39 W/kg of body mass.
    assert!((15.0..=39.0).contains(&power), "{power}");
    // Mirror-image wings and mirror-image half-strokes: no net side or
    // fore-aft force over the cycle, to rounding.
    assert!(mean.force.y.abs() < 1e-9 * weight, "{:?}", mean.force);
    assert!(mean.force.x.abs() < 1e-9 * weight, "{:?}", mean.force);
}

#[test]
fn advanced_rotation_lifts_more_than_delayed_rotation() {
    // Dickinson et al. 1999: rotating ahead of stroke reversal raises mean
    // lift and rotating after it lowers it. Here that comes
    // from the rotational term alone; the lead is 8 % of the cycle.
    let lead = 0.08 * 2.0 * std::f64::consts::PI;
    let at = |rotation_lead_rad| {
        cycle_mean(FlySpec {
            rotation_lead_rad,
            ..FlySpec::drosophila()
        })
        .force
        .z
    };
    let (advanced, symmetric, delayed) = (at(lead), at(0.0), at(-lead));
    writeln!(
        std::io::stderr(),
        "rotation timing: advanced {advanced:.3e}, symmetric {symmetric:.3e}, delayed {delayed:.3e} N"
    )
    .ok();
    assert!(advanced > symmetric && symmetric > delayed);
}
