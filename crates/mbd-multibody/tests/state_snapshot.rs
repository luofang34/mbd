//! The state is an explicit value and the derivative a pure function of it:
//! evaluating twice gives identical bits and changes nothing, and a cloned
//! state restored after stepping continues bit for bit, with every
//! integrator and with elements that carry continuous states and modes.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
};
use mbd_multibody::{
    ContactPointSpec, DormandPrince54, FloatingBaseState, Integrator, ModelScale, ModelState,
    MultibodyError, MultibodyModel, Rk4, Rosenbrock23, SupportedLoad, SurfaceMedium, SurfaceSample,
    Tether, TetherAnchor, TetherCommand, TimeScale, TireModelSpec, Tolerances, UniformGravity,
    Wheel, WheelCommand, WheelSpec, WinchTetherSpec, Workspace,
};

const MASS_KG: f64 = 2.0;
const SIZE_M: f64 = 0.3;
const G: f64 = 9.81;

type Elements = (UniformGravity, ContactPointSpec, Tether, Wheel);

fn part(parent: Option<u32>, at: DVec3, mass_kg: f64, joint: JointSpec) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement: parent.map_or(Placement::ROOT, |p| Placement::child_of(ComponentId(p), at)),
        dry_mass_kg: mass_kg,
        dry_inertia_body: DMat3::from_diagonal(
            DVec3::new(0.08, 0.1, 0.12) * (mass_kg * SIZE_M * SIZE_M),
        ),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint,
    }
}

/// A body with a ball-jointed boom and a hinged leg.
fn tree() -> ComponentTree {
    let posed = |joint| ComponentState {
        joint,
        ..ComponentState::at_temperature(293.15)
    };
    ComponentTree::new([
        (
            part(None, DVec3::ZERO, MASS_KG, JointSpec::FIXED),
            posed(JointState::FIXED),
        ),
        (
            part(
                Some(0),
                DVec3::new(0.0, 0.0, 0.5 * SIZE_M),
                0.2 * MASS_KG,
                JointSpec::spherical(core::f64::consts::PI),
            ),
            posed(JointState::spherical(
                DQuat::from_rotation_x(0.2),
                DVec3::new(0.3, -0.2, 0.1),
            )),
        ),
        (
            part(
                Some(0),
                DVec3::new(0.4 * SIZE_M, 0.0, -0.3 * SIZE_M),
                0.1 * MASS_KG,
                JointSpec::revolute(DVec3::Y, -1.0, 1.0),
            ),
            posed(JointState::revolute(0.2, -0.5)),
        ),
    ])
    .unwrap()
}

fn ground() -> SurfaceSample {
    SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    }
}

/// Gravity, a contact under the leg, a line from the boom to a world anchor
/// and a driven wheel under the body: seven continuous element states and
/// three modes.
fn elements() -> Elements {
    let load = SupportedLoad {
        mass_kg: MASS_KG,
        size_m: SIZE_M,
        gravity_m_s2: G,
        penetration_fraction: 0.02,
        damping_ratio: 0.4,
    };
    let foot = ContactPointSpec::sized_for(
        ComponentId(2),
        DVec3::new(0.0, 0.0, -0.4 * SIZE_M),
        &load,
        0.9,
        0.7,
    );
    let line = Tether {
        spec: WinchTetherSpec {
            stiffness_n_m: 20.0 * MASS_KG * G / SIZE_M,
            damping_n_s_m: 2.0,
            max_length_m: 10.0 * SIZE_M,
            breaking_tension_n: 1.0e3 * MASS_KG * G,
            burn_through_energy_j: Some(50.0),
            auto_cut_low_tension_n: Some(0.01 * MASS_KG * G),
            auto_cut_dwell_s: 10.0,
        },
        anchor_a: TetherAnchor::Link {
            component: ComponentId(1),
            offset_local_m: DVec3::new(0.0, 0.0, 0.5 * SIZE_M),
        },
        anchor_b: TetherAnchor::World {
            position_world_m: DVec3::new(0.2, 0.0, 3.0) * SIZE_M,
            velocity_world_m_s: DVec3::ZERO,
        },
        initial_length_m: 1.3 * SIZE_M,
        command: TetherCommand {
            payout_rate_m_s: -0.05 * SIZE_M,
            heat_rate_w: 2.0,
            cut: false,
        },
    };
    let wheel = Wheel {
        spec: WheelSpec {
            component: ComponentId(0),
            hub_offset_local_m: DVec3::new(-0.3 * SIZE_M, 0.0, -0.5 * SIZE_M),
            wheel_radius_m: 0.3 * SIZE_M,
            spring_n_m: 10.0 * MASS_KG * G / SIZE_M,
            damper_n_s_m: 20.0,
            forward_axis_local: DVec3::X,
            axle_axis_local: DVec3::Y,
            peak_mu: 0.8,
            cornering_stiffness_n_rad: 30.0,
            rolling_resistance_coeff: 0.02,
            longitudinal_stiffness_n: 40.0,
            tire: TireModelSpec::for_wheel(0.3 * SIZE_M, 0.5 * MASS_KG * G, G),
        },
        spin_inertia_kg_m2: 1.0e-3,
        command: WheelCommand {
            steer_angle_rad: 0.1,
            drive_torque_nm: 0.05,
            brake_torque_nm: 0.0,
        },
    };
    (UniformGravity(DVec3::new(0.0, 0.0, -G)), foot, line, wheel)
}

fn setup() -> (MultibodyModel, ModelState, Elements) {
    let tree = tree();
    let scale = ModelScale::with_time(SIZE_M, MASS_KG, TimeScale::Acceleration(G)).unwrap();
    let model = MultibodyModel::new(&tree, &[], scale).unwrap();
    let elements = elements();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, 0.85 * SIZE_M),
        rotation_world_from_root: DQuat::from_rotation_y(0.1),
        linear_vel_root_m_s: DVec3::new(0.4, 0.1, -0.2),
        angular_vel_root_rad_s: DVec3::new(0.1, 0.3, -0.2),
    };
    let state = model.state(&tree, &base, &elements, &ground()).unwrap();
    (model, state, elements)
}

fn bits(values: &[f64]) -> Vec<u64> {
    values.iter().map(|v| v.to_bits()).collect()
}

fn derivative(
    model: &MultibodyModel,
    state: &ModelState,
    elements: &Elements,
    work: &mut Workspace,
) -> Vec<u64> {
    let mut rates = vec![0.0; state.as_slice().len()];
    model
        .derivative(
            0.25,
            state.as_slice(),
            state.modes(),
            elements,
            &ground(),
            work,
            &mut rates,
        )
        .unwrap();
    bits(&rates)
}

#[test]
fn derivative_is_a_pure_function_of_the_state() {
    let (model, mut state, elements) = setup();
    assert_eq!(state.auxiliary().len(), 7);
    assert_eq!(state.modes().len(), 3);
    // Move into contact with every element active.
    let mut work = Workspace::new();
    for i in 0..80 {
        model
            .advance(
                &mut Rk4::new(),
                &elements,
                &ground(),
                &mut state,
                f64::from(i) * 1e-3,
                1e-3,
                &mut work,
            )
            .unwrap();
    }
    let before = state.clone();
    let first = derivative(&model, &state, &elements, &mut work);
    let second = derivative(&model, &state, &elements, &mut work);
    assert_eq!(first, second, "second evaluation differs");
    assert_eq!(
        bits(state.as_slice()),
        bits(before.as_slice()),
        "state changed"
    );
    assert_eq!(bits(state.modes()), bits(before.modes()), "modes changed");
    // The workspace carries nothing: evaluating elsewhere in between, or in
    // a fresh workspace, gives the same bits.
    let (_, elsewhere, _) = setup();
    derivative(&model, &elsewhere, &elements, &mut work);
    assert_eq!(first, derivative(&model, &state, &elements, &mut work));
    assert_eq!(
        first,
        derivative(&model, &state, &elements, &mut Workspace::new())
    );
    // Every element contributes a rate, so purity was tested on all of them.
    let aux_rates = &first[first.len() - 7..];
    assert!(
        aux_rates
            .iter()
            .filter(|b| f64::from_bits(**b) != 0.0)
            .count()
            >= 5
    );
}

/// Steps `state` over `steps` host steps of 2 ms from step index `from`.
fn step<G: Integrator>(
    integrator: &mut G,
    model: &MultibodyModel,
    elements: &Elements,
    state: &mut ModelState,
    from: u32,
    steps: u32,
) {
    let mut work = Workspace::new();
    for i in from..from + steps {
        model
            .advance(
                integrator,
                elements,
                &ground(),
                state,
                f64::from(i) * 2e-3,
                2e-3,
                &mut work,
            )
            .unwrap();
    }
}

/// Runs 60 steps, snapshots, runs 60 more, restores the snapshot and runs
/// the same 60 again with the same integrator object.
fn continuation_matches<G: Integrator>(mut integrator: G) {
    let (model, mut state, elements) = setup();
    step(&mut integrator, &model, &elements, &mut state, 0, 60);
    let snapshot = state.clone();
    step(&mut integrator, &model, &elements, &mut state, 60, 60);
    let first = state.clone();
    assert_ne!(bits(first.as_slice()), bits(snapshot.as_slice()));
    state = snapshot;
    step(&mut integrator, &model, &elements, &mut state, 60, 60);
    assert_eq!(bits(state.as_slice()), bits(first.as_slice()));
    assert_eq!(bits(state.modes()), bits(first.modes()));
}

#[test]
fn restored_snapshot_continues_bit_identically_with_rk4() {
    continuation_matches(Rk4::new());
}

#[test]
fn restored_snapshot_continues_bit_identically_with_dormand_prince() {
    continuation_matches(DormandPrince54::new(Tolerances::uniform(1e-7, 1e-7)));
}

#[test]
fn restored_snapshot_continues_bit_identically_with_rosenbrock() {
    continuation_matches(Rosenbrock23::new(Tolerances::uniform(1e-4, 1e-4)));
}

#[test]
fn failed_step_leaves_the_state_as_it_was() {
    let (model, mut state, elements) = setup();
    step(&mut Rk4::new(), &model, &elements, &mut state, 0, 60);
    let before = state.clone();
    // One internal step cannot cover a tenth of a second at this tolerance,
    // so the interval fails after its first accepted sub-step.
    let mut starved = DormandPrince54::new(Tolerances::uniform(1e-10, 1e-10));
    starved.max_steps = 1;
    let outcome = model.advance(
        &mut starved,
        &elements,
        &ground(),
        &mut state,
        0.12,
        0.1,
        &mut Workspace::new(),
    );
    assert!(
        matches!(
            outcome,
            Err(MultibodyError::StepLimitExceeded { limit: 1, .. })
        ),
        "{outcome:?}"
    );
    assert_eq!(bits(state.as_slice()), bits(before.as_slice()));
    assert_eq!(bits(state.modes()), bits(before.modes()));
}
