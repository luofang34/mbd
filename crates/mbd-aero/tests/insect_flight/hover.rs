//! Tethered hovering: the body held still, the wings on their prescribed
//! angles, the cycle-averaged force split into its quasi-steady terms.
//!
//! The entrained-air (added-mass) force has zero cycle mean here: for a
//! still body and periodic wing motion the air's impulse is periodic, and
//! the force is its rate of change (Newman 1977, §4.13).
//!
//! # What the lift is compared with
//!
//! The rig's kinematics are idealised (sinusoidal stroke and pitch, a
//! horizontal stroke plane, no stroke deviation, 45° incidence at
//! mid-stroke), so its lift is checked against what the quasi-steady model
//! predicts for exactly these kinematics, and against robotic-wing
//! measurements of the same idealisation, not against the weight of a fly
//! flapping differently:
//!
//! - Force coefficients are stroke means in the normalisation of Sane &
//!   Dickinson 2001 (J. Exp. Biol. 204:2607, eq. 3),
//!   `C̄ = 8F̄ / (ρ Φ² n² R² S r̂₂² ⟨(dφ̂/dt̂)²⟩)` with `φ̂ = 2φ/Φ`, which is the
//!   blade-element integral of `½ρ c (φ̇ r)² C` over a wing whose second
//!   moment of area is `S R² r̂₂²`. The translational coefficients of
//!   Dickinson, Lehmann & Sane 1999 were measured in that normalisation
//!   (Sane & Dickinson 2001, eqs 3, 5, 6: 180° sweeps at fixed incidence of
//!   a three-dimensional model wing), so integrating them element by element
//!   with the local speed counts nothing twice.
//! - Measured on the robotic wing with a horizontal stroke plane, no
//!   deviation and 45–50° mid-stroke incidence, the mean lift coefficient
//!   runs from 1.39 (Φ = 60°, α = 50°) to 1.87 (Φ = 180°, α = 50°) (Sane &
//!   Dickinson 2001, Fig. 3C, D) and 1.9 for a fast symmetric flip at
//!   Φ = 180°, α = 45° (ibid., p. 2615). That wing swept a smoothed
//!   triangular stroke at constant incidence through its own wake, so the
//!   range brackets these kinematics for plausibility only.
//! - Freely hovering flies need only `C̄_L = 1.36 ± 0.21` (Fry, Sayaman &
//!   Dickinson 2005, J. Exp. Biol. 208:2303, Table 1) because their
//!   kinematics differ: a U-shaped tip path with more than 25° of stroke
//!   deviation (ibid., p. 2309) that raises the aerodynamic incidence early
//!   in each half stroke (ibid., p. 2310). On the robotic wing, ±25° of
//!   deviation lowers the mean lift coefficient from 1.88 to 1.27–1.62
//!   (Sane & Dickinson 2001, Fig. 8), which spans the difference. The rig
//!   has no deviation and so carries more than the weight.
//! - Fry et al.'s statement that the translational term gives 89 % of the
//!   measured force (p. 2310) is for the measured kinematics of one flight
//!   sequence, so it says nothing about these.

use std::f64::consts::PI;
use std::io::Write;

use glam::DVec3;
use mbd_multibody::{FloatingBaseState, Workspace};

use super::fly::{Fly, FlySpec, FlyWorld};
use super::probe::{self, WingSample};

/// Samples per wingbeat; the integrand is periodic and smooth, so the
/// midpoint rule converges spectrally and this is far beyond the tolerance.
const SAMPLES: u32 = 2_000;

/// Non-dimensional second moment of area of the rig's continuous wing
/// shape (see `FlySpec::chords_m`).
const R2_SQ: f64 = (1.75 * 2.75) / (3.25 * 4.25);

/// Wing read-outs of the fly at rest at time `t`.
fn sample(spec: FlySpec, t: f64) -> WingSample {
    let fly = Fly::new(spec);
    let model = probe::model(&fly);
    let elements = probe::elements(&fly, true, Vec::new());
    let world = FlyWorld::new(&spec);
    let base = FloatingBaseState::AT_REST;
    let state = probe::state_at(&fly, &model, &elements, &world, &base, t);
    let mut work = Workspace::new();
    let kin = model.kinematics(&state, &mut work).unwrap();
    probe::wings(&fly, &kin, &world)
}

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
        mean.lift += s.lift;
        mean.drag += s.drag;
        mean.rotational += s.rotational;
        mean.moment_origin += s.moment_origin;
        mean.power_w += s.power_w;
        mean.reynolds += s.reynolds;
    }
    let n = f64::from(SAMPLES);
    WingSample {
        force: mean.force / n,
        translational: mean.translational / n,
        lift: mean.lift / n,
        drag: mean.drag / n,
        rotational: mean.rotational / n,
        moment_origin: mean.moment_origin / n,
        power_w: mean.power_w / n,
        reynolds: mean.reynolds / n,
        ..WingSample::default()
    }
}

/// Cycle-mean force of the entrained air on the wings: minus the change of
/// its impulse over one beat, divided by the period.
fn added_mass_mean(spec: FlySpec) -> DVec3 {
    let p0 = sample(spec, 0.0).fluid_linear;
    let p1 = sample(spec, spec.period_s()).fluid_linear;
    -(p1 - p0) / spec.period_s()
}

/// Mean force coefficient of one wing in the normalisation of Sane &
/// Dickinson 2001 eq. 3. For the sinusoidal stroke `φ̂ = cos 2πt̂`, so
/// `⟨(dφ̂/dt̂)²⟩ = 2π²`.
fn mean_coefficient(spec: &FlySpec, force_per_wing_n: f64) -> f64 {
    let (phi, n, r) = (
        spec.stroke_amplitude_rad,
        spec.frequency_hz,
        spec.wing_length_m,
    );
    let speed_sq = 2.0 * PI * PI;
    let area = spec.wing_area_m2;
    8.0 * force_per_wing_n
        / (spec.air_density_kg_m3 * phi * phi * n * n * r * r * area * R2_SQ * speed_sq)
}

/// The quasi-steady translational lift coefficient for the rig's
/// kinematics, worked out apart from the element. The pitch axis is the
/// span and every element moves horizontally, so all elements share the
/// incidence `90° − |ψ(t)|` and the stroke mean is the `φ̇²`-weighted mean
/// of `C_L`, with the fit of Dickinson, Lehmann & Sane (1999) as quoted by
/// Sane & Dickinson 2001, eq. 5.
fn quasi_steady_lift_coefficient(spec: &FlySpec) -> f64 {
    let (mut weighted, mut weight) = (0.0, 0.0);
    for k in 0..SAMPLES {
        let t = (f64::from(k) + 0.5) / f64::from(SAMPLES) * spec.period_s();
        let [(_, stroke_rate, _), (pitch, _, _)] = super::fly::wing_angles(spec, t);
        let alpha_deg = 90.0 - pitch.abs().to_degrees();
        let cl = 0.225 + 1.58 * (2.13 * alpha_deg - 7.20).to_radians().sin();
        weighted += stroke_rate * stroke_rate * cl;
        weight += stroke_rate * stroke_rate;
    }
    weighted / weight
}

#[test]
fn hover_lift_matches_the_quasi_steady_prediction_for_its_kinematics() {
    let spec = FlySpec::drosophila();
    let mean = cycle_mean(spec);
    let weight = spec.weight_n();
    let (lift, drag, rotational) = (mean.lift.z, mean.drag.z, mean.rotational.z);
    let added = added_mass_mean(spec).z;
    let c_lift = mean_coefficient(&spec, 0.5 * lift);
    let c_total = mean_coefficient(&spec, 0.5 * mean.force.z);
    let c_quasi = quasi_steady_lift_coefficient(&spec);
    writeln!(
        std::io::stderr(),
        "hover per weight: translational lift {:.4}, translational drag {:.2e}, \
         rotational {:.4}, added mass {:.2e}, total {:.4}; C̄_L translational \
         {c_lift:.4}, total {c_total:.4}, quadrature {c_quasi:.4}",
        lift / weight,
        drag / weight,
        rotational / weight,
        added / weight,
        mean.force.z / weight,
    )
    .ok();
    // The element's blade-element sum reproduces the quadrature exactly
    // once the strips' own second moment (0.3494 for 20 strips, against
    // 0.3484 for the continuous shape) replaces the continuous one: the
    // speeds, incidences, coefficients and lift directions all agree.
    let chords = spec.chords_m();
    let width = spec.wing_length_m / chords.len() as f64;
    let second_moment: f64 = chords
        .iter()
        .enumerate()
        .map(|(k, c)| c * width * ((k as f64 + 0.5) * width).powi(2))
        .sum();
    let r2_strips = second_moment / (spec.wing_area_m2 * spec.wing_length_m.powi(2));
    let c_strips = c_quasi * r2_strips / R2_SQ;
    assert!(
        (c_lift / c_strips - 1.0).abs() < 1e-12,
        "{c_lift} vs {c_strips}"
    );
    // Horizontal stroke plane: every element moves horizontally, so drag
    // has no vertical part.
    assert!(drag.abs() < 1e-12 * weight, "{drag}");
    // Periodic impulse: the identity holds to rounding.
    assert!(added.abs() < 1e-9 * weight, "{added}");
    assert!((lift + drag + rotational - mean.force.z).abs() < 1e-12 * weight);
    // A plausibility bracket, not a validation: the robotic-wing range for
    // similar idealised kinematics (module comment).
    assert!((1.39..=1.90).contains(&c_total), "{c_total}");
}

#[test]
fn hovering_wings_support_the_body_at_the_published_power() {
    let spec = FlySpec::drosophila();
    let mean = cycle_mean(spec);
    let weight = spec.weight_n();
    let lift_ratio = mean.force.z / weight;
    let power = mean.power_w / spec.total_mass_kg();
    writeln!(
        std::io::stderr(),
        "hover: lift {:.2} µN, lift/weight {lift_ratio:.3}, power {power:.1} W/kg, \
         element-mean Re {:.0}, horizontal {:?} N",
        mean.force.z * 1e6,
        mean.reynolds,
        mean.force.truncate()
    )
    .ok();
    // A recorded value, not a validation: these kinematics give the mean
    // lift coefficient checked in the test above, more than real flies need
    // with theirs (module comment), so the wings carry more than the weight.
    assert!((1.370..=1.380).contains(&lift_ratio), "{lift_ratio}");
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
    // lift and rotating after it lowers it. The lead is 8 % of the cycle.
    // Either shift moves the incidence at mid-stroke off 45° and costs the
    // same translational lift; the ordering comes from the rotational term,
    // equal and opposite for the two.
    let lead = 0.08 * 2.0 * std::f64::consts::PI;
    let at = |rotation_lead_rad| {
        let m = cycle_mean(FlySpec {
            rotation_lead_rad,
            ..FlySpec::drosophila()
        });
        (m.force.z, m.lift.z, m.rotational.z)
    };
    let (advanced, symmetric, delayed) = (at(lead), at(0.0), at(-lead));
    writeln!(
        std::io::stderr(),
        "rotation timing (total, translational lift, rotational) N: \
         advanced {advanced:?}, symmetric {symmetric:?}, delayed {delayed:?}"
    )
    .ok();
    assert!(advanced.0 > symmetric.0 && symmetric.0 > delayed.0);
    assert!((advanced.1 - delayed.1).abs() < 1e-12 * symmetric.0);
    assert!((advanced.2 + delayed.2).abs() < 1e-12 * symmetric.0);
    assert!(advanced.2 > 0.0);
}
