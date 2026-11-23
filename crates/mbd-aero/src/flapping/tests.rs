use std::f64::consts::PI;

use glam::DVec3;
use mbd_core::AmbientGas;
use mbd_multibody::SpatialMotion;

use super::*;

const CHORD_M: f64 = 1e-3;
const WIDTH_M: f64 = 2e-3;

/// One strip, span along `-z`, chord (toward the leading edge) along `+x`,
/// so the plate normal is `+y`.
fn plate(axis_fraction: f64) -> FlappingWingAero {
    FlappingWingAero::new(FlappingWingParams {
        root_link_m: DVec3::ZERO,
        span_axis_link: -DVec3::Z,
        chord_axis_link: DVec3::X,
        length_m: WIDTH_M,
        chords_m: vec![CHORD_M],
        rotation_axis_chord_fraction: axis_fraction,
        center_of_pressure_chord_fraction: 0.25,
    })
    .unwrap()
}

fn air() -> AmbientGas {
    AmbientGas::earth_air(101_325.0, 293.15).unwrap()
}

fn flow(v: DVec3, w: DVec3) -> LinkFlow<'static> {
    LinkFlow {
        velocity_air_link_m_s: v,
        angular_rate_link_rad_s: w,
        ambient: air(),
        controls: &[],
    }
}

#[test]
fn fits_are_transcribed_with_their_peak_and_broadside_values() {
    // A transcription check of the two formulas, not a validation: the
    // fits peak in lift near 45° at about 1.8 and reach about 3.46 in drag
    // broadside.
    let (cl45, _) = translational_coefficients(45.0);
    let (_, cd90) = translational_coefficients(90.0);
    assert!((cl45 - 1.8).abs() < 0.01, "{cl45}");
    assert!((cd90 - 3.46).abs() < 0.01, "{cd90}");
}

#[test]
fn a_plate_translating_at_incidence_lifts_away_from_its_normal_motion() {
    let wing = plate(0.25);
    let speed = 2.0;
    let alpha = 40f64.to_radians();
    // Leading edge first, moving down: air strikes the lower face.
    let v = DVec3::new(alpha.cos(), -alpha.sin(), 0.0) * speed;
    let out = wing.evaluate(&flow(v, DVec3::ZERO)).unwrap();
    let f = out.translational_force_link_n;
    let (cl, cd) = translational_coefficients(40.0);
    let q_s = 0.5 * air().density_kg_m3 * speed * speed * CHORD_M * WIDTH_M;
    let lift_dir = DVec3::new(alpha.sin(), alpha.cos(), 0.0);
    // Exact formula evaluation; rounding only.
    assert!((f.dot(lift_dir) - cl * q_s).abs() < 1e-12 * cl * q_s);
    assert!((f.dot(-v / speed) - cd * q_s).abs() < 1e-12 * cd * q_s);
    assert!(
        out.aerodynamic_power_w > 0.0,
        "drag takes power from the wing"
    );
}

#[test]
fn rotational_force_follows_the_kutta_coefficient_of_the_axis() {
    let speed = 2.0;
    let pitch_rate = 300.0;
    for x0 in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let wing = plate(x0);
        // Nose-up rotation about the span (+z for this plate).
        let out = wing
            .evaluate(&flow(DVec3::X * speed, DVec3::Z * pitch_rate))
            .unwrap();
        let c_rot = PI * (0.75 - x0);
        let expected = c_rot * air().density_kg_m3 * speed * pitch_rate * CHORD_M * CHORD_M;
        let f = out.rotational_force_link_n;
        // Sane & Dickinson 2002 eq. 11 for one strip, along the normal.
        assert!((f.y - expected * WIDTH_M).abs() <= 1e-12 * expected.abs().max(1e-30));
        assert_eq!(f.x, 0.0);
    }
}

#[test]
fn added_inertia_matches_the_quarter_chord_coefficients() {
    // Sane & Dickinson 2002 eq. 3 with the axis at the quarter chord:
    // ρπc²/4 per span for normal acceleration, ρπc³/16 per span for
    // rotational acceleration about the axis.
    let wing = plate(0.25);
    let rho = air().density_kg_m3;
    let added = wing
        .evaluate(&flow(DVec3::ZERO, DVec3::ZERO))
        .unwrap()
        .added_inertia;
    let heave = added.mul_motion(SpatialMotion::new(DVec3::ZERO, DVec3::Y));
    let expected_heave = rho * PI * CHORD_M * CHORD_M / 4.0 * WIDTH_M;
    assert!((heave.force.y - expected_heave).abs() < 1e-12 * expected_heave);
    let pitch = added.mul_motion(SpatialMotion::new(DVec3::Z, DVec3::ZERO));
    let expected_pitch = rho * PI * CHORD_M.powi(3) / 16.0 * WIDTH_M;
    // Mid-chord lies aft of the axis, so a nose-up acceleration moves it
    // down: the entrained air pushes back up.
    assert!((pitch.force.y + expected_pitch).abs() < 1e-12 * expected_pitch);
    assert!(heave.force.x == 0.0 && pitch.force.x == 0.0);
}

#[test]
fn rejects_geometry_it_cannot_represent() {
    let good = plate(0.25).params().clone();
    let bad = [
        FlappingWingParams {
            chords_m: vec![],
            ..good.clone()
        },
        FlappingWingParams {
            chord_axis_link: DVec3::new(0.6, 0.0, 0.8),
            ..good.clone()
        },
        FlappingWingParams {
            rotation_axis_chord_fraction: 1.5,
            ..good
        },
    ];
    for params in bad {
        assert!(FlappingWingAero::new(params).is_err());
    }
}
