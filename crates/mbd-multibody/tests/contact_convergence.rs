//! A contact element evaluated at every RK4 stage: with its bristle state
//! integrated alongside the body, the coupled system converges at fourth
//! order wherever the contact force is smooth. The motion here stays in
//! contact, below the static friction bound, deeper than the damping ramp
//! and with a positive normal load throughout, so no switch is crossed.

#![cfg(test)]

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    ContactPointSpec, ContactPointState, FloatingBaseState, MultibodyModel, Rk4, SupportedLoad,
    SurfaceMedium, SurfaceSample, UniformGravity, Workspace,
};

const MASS_KG: f64 = 1.0e-6;
const SIZE_M: f64 = 1.0e-3;
const G: f64 = 9.81;
/// Static penetration, 1 % of the body size.
const DEPTH_M: f64 = 0.01 * SIZE_M;

fn foot() -> ContactPointSpec {
    let load = SupportedLoad {
        mass_kg: MASS_KG,
        size_m: SIZE_M,
        gravity_m_s2: G,
        penetration_fraction: 0.01,
        damping_ratio: 0.3,
    };
    ContactPointSpec::sized_for(ComponentId(0), DVec3::ZERO, &load, 0.9, 0.7)
}

/// Nondimensional `[position, velocity, bristle deflection]` after `t_end`
/// in `steps` RK4 steps, with whether the contact stuck throughout.
fn settle(steps: u32, t_end: f64) -> (Vec<f64>, bool) {
    let spec = ComponentSpec {
        name: "mote".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: MASS_KG,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(0.1 * MASS_KG * SIZE_M * SIZE_M)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let model = MultibodyModel::new(&tree, &[]).unwrap();
    let ground = SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    };
    let foot = foot();
    let omega = (foot.stiffness_n_m / MASS_KG).sqrt();
    let elements = (UniformGravity(DVec3::new(0.0, 0.0, -G)), foot);
    // 10 % deeper than static, sliding sideways slowly enough to stick:
    // the bristle force peaks near a fifth of the static friction bound.
    let start = FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, -1.1 * DEPTH_M),
        linear_vel_root_m_s: DVec3::new(0.2 * 0.9 * G / omega, 0.1 * 0.9 * G / omega, 0.0),
        ..FloatingBaseState::AT_REST
    };
    let mut state = model.state(&tree, &start, &elements, &ground).unwrap();
    let mut work = Workspace::new();
    let mut rk4 = Rk4::new();
    let h = t_end / f64::from(steps);
    let mut stuck = true;
    for i in 0..steps {
        model
            .advance(
                &mut rk4,
                &elements,
                &ground,
                &mut state,
                f64::from(i) * h,
                h,
                &mut work,
            )
            .unwrap();
        let contact = ContactPointState::from_blocks(state.auxiliary(), state.modes());
        stuck &= contact.sticking;
    }
    let base = state.base();
    let mut out = Vec::new();
    out.extend((base.position_world_m / DEPTH_M).to_array());
    out.extend((base.linear_vel_root_m_s / (DEPTH_M * omega)).to_array());
    out.extend(state.auxiliary().iter().map(|z| z / DEPTH_M));
    (out, stuck)
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

#[test]
fn contact_inside_rk4_stages_converges_at_fourth_order() {
    // Three contact periods; the coarsest step has omega * h of about 0.3.
    let omega = (foot().stiffness_n_m / MASS_KG).sqrt();
    let t_end = 3.0 * core::f64::consts::TAU / omega;
    let runs: Vec<(Vec<f64>, bool)> = [64, 128, 256, 512]
        .iter()
        .map(|&steps| settle(steps, t_end))
        .collect();
    assert!(runs.iter().all(|(_, stuck)| *stuck), "contact slipped");
    // Richardson: |y(h) - y(h/2)| falls 2^p per halving for order p.
    let diffs: Vec<f64> = runs
        .windows(2)
        .map(|w| distance(&w[0].0, &w[1].0))
        .collect();
    let orders: Vec<f64> = diffs.windows(2).map(|d| (d[0] / d[1]).log2()).collect();
    // The bristle must actually have deflected, or the test proves nothing.
    assert!(runs[3].0[6].abs() > 1.0e-4, "bristle {:e}", runs[3].0[6]);
    assert!(
        diffs.iter().all(|d| *d > 1.0e-11),
        "at rounding floor: {diffs:?}"
    );
    for order in orders {
        assert!((3.6..=4.4).contains(&order), "observed order {order:.3}");
    }
}
