//! Landing: the fly, wings held still, drops onto a floor on six legs
//! whose contacts are sized for micrometre penetration under its weight.

use std::io::Write;
use std::time::Instant;

use glam::DVec3;
use mbd_multibody::{
    ContactPointSpec, FloatingBaseState, ModelState, MultibodyModel, Rk4, SupportedLoad, Workspace,
};

use super::fly::{BODY, Fly, FlySpec, FlyWorld};
use super::free_flight::STEPS_PER_BEAT;
use super::probe;

/// Static leg penetration as a fraction of the wing length: about 2.4 µm.
const PENETRATION_FRACTION: f64 = 1e-3;

fn legs(fly: &Fly) -> Vec<ContactPointSpec> {
    let spec = &fly.spec;
    let r = spec.wing_length_m;
    let load = SupportedLoad {
        mass_kg: spec.total_mass_kg() / 6.0,
        size_m: r,
        gravity_m_s2: spec.gravity_m_s2,
        penetration_fraction: PENETRATION_FRACTION,
        damping_ratio: 0.7,
    };
    [-0.25, 0.0, 0.25]
        .into_iter()
        .flat_map(|x| [-0.15, 0.15].map(|y| DVec3::new(x, y, -0.25) * r))
        .map(|at| ContactPointSpec::sized_for(BODY, at, &load, 0.8, 0.6))
        .collect()
}

/// Kinetic and potential energy of the bodies, kinetic energy of the
/// entrained air, and the energy stored in the legs' normal springs, J.
fn energy(
    fly: &Fly,
    model: &MultibodyModel,
    elements: &probe::FlyElements,
    world: &FlyWorld,
    state: &ModelState,
) -> f64 {
    let mut work = Workspace::new();
    let kin = model.kinematics(state, &mut work).unwrap();
    let springs: f64 = elements
        .3
        .iter()
        .map(|c| {
            let foot = kin.point(BODY, c.offset_local_m).unwrap().position_world_m;
            let depth = (-foot.z).max(0.0);
            0.5 * c.stiffness_n_m * depth * depth
        })
        .sum();
    probe::body_energy(fly, &kin) + probe::wings(fly, &kin, world).fluid_energy_j + springs
}

/// The fly with its wings held still, on six legs, in air.
fn rig() -> (Fly, MultibodyModel, probe::FlyElements, FlyWorld) {
    let spec = FlySpec::drosophila();
    let mut fly = Fly::new(spec);
    fly.servo.hold = true;
    fly.servo.pitch_gains = None;
    let model = probe::model(&fly);
    let elements = probe::elements(&fly, true, legs(&fly));
    (fly, model, elements, FlyWorld::new(&spec))
}

/// Largest energy increase from one step to the next.
fn worst_step_gain(energies: &[f64]) -> f64 {
    energies
        .windows(2)
        .map(|w| w[1] - w[0])
        .fold(f64::NEG_INFINITY, f64::max)
}

#[test]
fn the_fly_settles_on_its_legs_without_gaining_energy() {
    let (fly, model, elements, world) = rig();
    let spec = fly.spec;
    let r = spec.wing_length_m;
    // Feet a fifth of a wing length above the floor, at rest.
    let at = DVec3::new(0.0, 0.0, 0.45 * r);
    let drop = FloatingBaseState {
        position_world_m: at,
        ..FloatingBaseState::AT_REST
    };
    let mut state = model.state(&fly.tree, &drop, &elements, &world).unwrap();
    let mut work = Workspace::new();
    let e0 = energy(&fly, &model, &elements, &world, &state);
    let h = spec.period_s() / f64::from(STEPS_PER_BEAT);
    let (mut peak, mut evals, mut energies, mut touched_at) = (e0, 0_u64, Vec::new(), None);
    let beats = 40;
    let start = Instant::now();
    for k in 0..beats * STEPS_PER_BEAT {
        let (t, mut rk4) = (f64::from(k) * h, Rk4::new());
        let report =
            (model.advance(&mut rk4, &elements, &world, &mut state, t, h, &mut work)).unwrap();
        evals += u64::from(report.derivative_evals);
        let e = energy(&fly, &model, &elements, &world, &state);
        peak = peak.max(e);
        energies.push(e);
        let lowest = state.base().position_world_m.z - 0.25 * r;
        if touched_at.is_none() && lowest < 0.0 {
            touched_at = Some(energies.len() - 1);
        }
    }
    let wall = start.elapsed().as_secs_f64();
    let base = state.base();
    let e_end = energy(&fly, &model, &elements, &world, &state);
    let depth = -(base.position_world_m.z - 0.25 * r);
    let static_depth = PENETRATION_FRACTION * r;
    let speed = base.linear_vel_world_m_s.length();
    writeln!(
        std::io::stderr(),
        "landing: final depth {:.3} µm (static {:.3} µm), speed {:.2e} m/s, energy \
         {:.3e} → {:.3e} J, peak {:.3e} J, {} evals in {:.3} s for {:.3} s simulated",
        depth * 1e6,
        static_depth * 1e6,
        speed,
        e0,
        e_end,
        peak,
        evals,
        wall,
        f64::from(beats) * spec.period_s(),
    )
    .ok();
    // Contacts, servo and air only dissipate: energy never rises above
    // where it started, beyond RK4 rounding of the spring energy (1e-6).
    assert!(peak <= e0 * (1.0 + 1e-6), "{peak} vs {e0}");
    // From first touch on, no step gains energy beyond rounding (1e-9 of
    // the impact energy; the bristle springs are left out of the sum and
    // store too little to show).
    let impact = e0 - e_end;
    let worst_gain = worst_step_gain(&energies[touched_at.expect("the fly lands")..]);
    writeln!(
        std::io::stderr(),
        "landing: worst step gain {:.2e} of the impact energy",
        worst_gain / impact
    )
    .ok();
    assert!(worst_gain <= 1e-9 * impact, "{worst_gain} vs {impact}");
    // At rest: the legs are placed symmetrically about the centre of mass,
    // so each carries a sixth of the weight and sinks to the static depth
    // it was sized for (1e-3 covers the residual creep of the damped
    // bristles), still to a millionth of the free-fall speed.
    assert!(
        (depth - static_depth).abs() < 1e-3 * static_depth,
        "{depth}"
    );
    assert!(speed < 1e-6 * (spec.gravity_m_s2 * r).sqrt(), "{speed}");
}
