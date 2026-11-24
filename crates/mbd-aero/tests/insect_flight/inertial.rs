//! Inertial reaction: flapping in vacuum without gravity, the body must
//! recoil against its wings exactly as momentum conservation demands.

use std::io::Write;

use glam::DVec3;
use mbd_multibody::{FloatingBaseState, Rk4, Workspace};

use super::fly::{BODY, Fly, FlySpec, FlyWorld, WINGS};
use super::free_flight::STEPS_PER_BEAT;
use super::probe;

/// Half the peak-to-peak range of samples spanning exactly one period,
/// after removing the straight line through the first and last: the
/// system drifts at the constant velocity its start gave it.
fn amplitude(xs: &[f64]) -> f64 {
    let (first, last) = (xs[0], xs[xs.len() - 1]);
    let n = (xs.len() - 1) as f64;
    let (lo, hi) =
        xs.iter()
            .enumerate()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (k, &x)| {
                let d = x - (first + (last - first) * k as f64 / n);
                (lo.min(d), hi.max(d))
            });
    0.5 * (hi - lo)
}

/// Fore-aft sway of the body and of the right wing's centre of mass over
/// the fourth wingbeat, and the body's pitch rocking, flapping without
/// gravity or pitch control, with or without air.
fn sway(spec: FlySpec, air: bool) -> (f64, f64, f64) {
    let mut fly = Fly::new(spec);
    fly.servo.pitch_gains = None;
    let model = probe::model(&fly);
    let elements = probe::elements(&fly, air, Vec::new());
    let world = FlyWorld::new(&spec);
    let base = FloatingBaseState::AT_REST;
    let mut state = probe::state_at(&fly, &model, &elements, &world, &base, 0.0);
    let mut work = Workspace::new();
    let wing_cg = fly.masses[1].cg_m;
    let h = spec.period_s() / f64::from(STEPS_PER_BEAT);
    let beats = 4;
    let (mut body_x, mut wing_x, mut pitch) = (Vec::new(), Vec::new(), Vec::new());
    for k in 0..beats * STEPS_PER_BEAT {
        let t = f64::from(k) * h;
        model
            .advance(
                &mut Rk4::new(),
                &elements,
                &world,
                &mut state,
                t,
                h,
                &mut work,
            )
            .unwrap();
        if k + 1 < (beats - 1) * STEPS_PER_BEAT {
            continue;
        }
        let kin = model.kinematics(&state, &mut work).unwrap();
        body_x.push(kin.point(BODY, DVec3::ZERO).unwrap().position_world_m.x);
        wing_x.push(kin.point(WINGS[0], wing_cg).unwrap().position_world_m.x);
        let nose = state.base().rotation_world_from_root * DVec3::X;
        pitch.push((-nose.z).atan2(nose.x));
    }
    (amplitude(&body_x), amplitude(&wing_x), amplitude(&pitch))
}

#[test]
fn flapping_wings_rock_the_body_by_the_momentum_ratio() {
    let spec = FlySpec {
        gravity_m_s2: 0.0,
        ..FlySpec::drosophila()
    };
    let (a_body, a_wing, rocking) = sway(spec, false);
    let (air_body, _, air_rocking) = sway(spec, true);
    let ratio = spec.wing_mass_kg / spec.body_mass_kg;
    // The task's estimate per wing, doubled for the pair; momentum
    // conservation makes it exact for the centre-of-mass motion.
    let estimate = 2.0 * ratio * a_wing;
    writeln!(
        std::io::stderr(),
        "inertial reaction: body sway {:.3} µm vs 2 m_w/m_b × wing CG travel {:.3} µm \
         (wing CG amplitude {:.3} mm), body pitch rocking {:.3} deg; with air: sway \
         {:.3} µm, rocking {:.3} deg",
        a_body * 1e6,
        estimate * 1e6,
        a_wing * 1e3,
        rocking.to_degrees(),
        air_body * 1e6,
        air_rocking.to_degrees(),
    )
    .ok();
    // The body origin is its centre of mass and the stroke links are
    // massless, so momentum conservation makes the relation exact; what is
    // left is RK4 error and the straight-line detrending: 1e-3.
    assert!(
        (a_body - estimate).abs() < 1e-3 * estimate,
        "{a_body} vs {estimate}"
    );
    assert!(a_body > 1e-3 * spec.wing_length_m, "the body visibly moves");
}
