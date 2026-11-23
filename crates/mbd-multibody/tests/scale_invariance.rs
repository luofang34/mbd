//! One nondimensional problem with a contact and a tether, built at insect
//! scale (1 mm, 1 mg) and at aircraft scale (10 m, 1 t) with consistently
//! scaled time: a body lands sliding on the ground, sticks, and is then
//! reeled off it by a winch line. Mechanics has no intrinsic scale and the
//! elements carry no dimensional constant, so the nondimensional
//! trajectories, element states and modes coincide.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    ContactPointSpec, ContactPointState, FloatingBaseState, MultibodyModel, Rk4, SupportedLoad,
    SurfaceMedium, SurfaceSample, Tether, TetherAnchor, TetherCommand, TimeScale, UniformGravity,
    WinchTetherSpec, WinchTetherState, Workspace,
};

/// Characteristic length (m), mass (kg) and time (s).
#[derive(Clone, Copy, Debug)]
struct Scale {
    l: f64,
    m: f64,
    t: f64,
}

impl Scale {
    /// Gravity of nondimensional magnitude one.
    fn gravity(self) -> f64 {
        self.l / (self.t * self.t)
    }
}

fn foot(s: Scale) -> ContactPointSpec {
    let load = SupportedLoad {
        mass_kg: s.m,
        size_m: s.l,
        gravity_m_s2: s.gravity(),
        penetration_fraction: 0.01,
        damping_ratio: 0.3,
    };
    ContactPointSpec::sized_for(
        ComponentId(0),
        DVec3::new(0.0, 0.0, -0.5) * s.l,
        &load,
        0.9,
        0.7,
    )
}

/// Winch line from the top of the body to an anchor three lengths up,
/// slack at first and reeled in at `0.3 L/T`.
fn winch(s: Scale) -> Tether {
    let stiffness = s.m * s.gravity() / (0.01 * s.l);
    Tether {
        spec: WinchTetherSpec {
            stiffness_n_m: stiffness,
            damping_n_s_m: 2.0 * 0.3 * (stiffness * s.m).sqrt(),
            max_length_m: 10.0 * s.l,
            breaking_tension_n: 1.0e3 * s.m * s.gravity(),
            burn_through_energy_j: None,
            auto_cut_low_tension_n: None,
            auto_cut_dwell_s: 0.0,
        },
        anchor_a: TetherAnchor::Link {
            component: ComponentId(0),
            offset_local_m: DVec3::new(0.0, 0.0, 0.5) * s.l,
        },
        anchor_b: TetherAnchor::World {
            position_world_m: DVec3::new(0.0, 0.0, 3.0) * s.l,
            velocity_world_m_s: DVec3::ZERO,
        },
        initial_length_m: 2.4 * s.l,
        command: TetherCommand {
            payout_rate_m_s: -0.3 * s.l / s.t,
            ..TetherCommand::default()
        },
    }
}

/// What a run reports besides its final state.
struct Outcome {
    /// Nondimensional base state, element states and modes.
    state: Vec<f64>,
    /// Steps on which the contact was sliding.
    slipping_steps: usize,
    /// Peak nondimensional line tension, in body weights.
    peak_tension: f64,
}

fn nondimensional_run(s: Scale, steps: usize) -> Outcome {
    let spec = ComponentSpec {
        name: "body".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: s.m,
        dry_inertia_body: DMat3::from_diagonal(DVec3::new(0.08, 0.1, 0.12) * (s.m * s.l * s.l)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(s.gravity()))
            .unwrap();
    let ground = SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    };
    let (foot, line) = (foot(s), winch(s));
    let elements = (
        UniformGravity(DVec3::new(0.0, 0.0, -s.gravity())),
        foot,
        line,
    );
    let start = FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, 0.7) * s.l,
        rotation_world_from_root: DQuat::from_rotation_y(0.05),
        linear_vel_root_m_s: DVec3::new(0.3, 0.1, 0.0) * (s.l / s.t),
        angular_vel_root_rad_s: DVec3::new(0.02, -0.01, 0.03) / s.t,
    };
    let mut state = model.state(&tree, &start, &elements, &ground).unwrap();
    let mut work = Workspace::new();
    let mut rk4 = Rk4::new();
    let h = 0.01 * s.t;
    let mut out = Outcome {
        state: Vec::new(),
        slipping_steps: 0,
        peak_tension: 0.0,
    };
    for i in 0..steps {
        model
            .advance(
                &mut rk4,
                &elements,
                &ground,
                &mut state,
                i as f64 * h,
                h,
                &mut work,
            )
            .unwrap();
        let (aux, modes) = (state.auxiliary(), state.modes());
        let contact = ContactPointState::from_blocks(&aux[..3], &modes[..1]);
        out.slipping_steps += usize::from(!contact.sticking);
        let line_state = WinchTetherState::from_blocks(&aux[3..], &modes[1..]);
        let top = state.base().position_world_m
            + state.base().rotation_world_from_root * (DVec3::new(0.0, 0.0, 0.5) * s.l);
        let stretch =
            (top - DVec3::new(0.0, 0.0, 3.0) * s.l).length() - line_state.deployed_length_m;
        let tension = stretch.max(0.0) * line.spec.stiffness_n_m / (s.m * s.gravity());
        out.peak_tension = out.peak_tension.max(tension);
    }
    let base = state.base();
    out.state.extend((base.position_world_m / s.l).to_array());
    out.state.extend(base.rotation_world_from_root.to_array());
    out.state
        .extend((base.linear_vel_root_m_s * (s.t / s.l)).to_array());
    out.state
        .extend((base.angular_vel_root_rad_s * s.t).to_array());
    let aux = state.auxiliary();
    // Bristle deflection and line length are lengths; heat and the
    // low-tension timer stay zero for this line.
    out.state.extend(aux[..4].iter().map(|v| v / s.l));
    out.state.extend(&aux[4..]);
    out.state.extend(state.modes());
    out
}

#[test]
fn landing_and_winching_trajectory_is_scale_invariant() {
    let insect = Scale {
        l: 1.0e-3,
        m: 1.0e-6,
        t: 1.0e-3,
    };
    let aircraft = Scale {
        l: 10.0,
        m: 1.0e3,
        t: 1.0,
    };
    let big = nondimensional_run(aircraft, 600);
    let small = nondimensional_run(insect, 600);
    // Both runs perform the same nondimensional arithmetic and differ only
    // by rounding (~1e-16 per operation). The landing, the stick-slip latch
    // and the line going taut are switches, which amplify rounding by the
    // stiffness ratio but must not move by a step: 1e-9 of the O(1)
    // nondimensional state is a rounding-only bound.
    for (k, (a, b)) in small.state.iter().zip(&big.state).enumerate() {
        assert!(
            (a - b).abs() < 1e-9 * a.abs().max(b.abs()).max(1.0),
            "component {k}: insect {a} vs aircraft {b}"
        );
    }
    assert_eq!(small.slipping_steps, big.slipping_steps);
    // The scenario must have exercised both elements: the contact slid and
    // then stuck, and the line went taut and lifted the body off the ground.
    assert!(
        big.slipping_steps > 0 && big.slipping_steps < 600,
        "{}",
        big.slipping_steps
    );
    assert!(
        big.peak_tension > 0.9,
        "peak tension {} weights",
        big.peak_tension
    );
    assert!(big.state[2] > 0.6, "body height {} L", big.state[2]);
}
