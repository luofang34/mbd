//! Elements plugged into a model load it exactly as their stand-alone
//! evaluations do, and the model's refresh hooks match a fresh preparation.

#![cfg(test)]

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointSpec,
    JointState, Placement, WrenchAtSite,
};
use mbd_multibody::{
    DockingCapture, DockingPhase, DockingSpec, DockingState, FloatingBaseState, ForceElement,
    HullStation, HydrodynamicsModel, JointLoadInput, LoopConstraint, MultiBodyDynamics,
    MultibodyModel, PortAnchor, PortKinematics, Rk4, TimeScale, WaterState, Workspace,
};

fn part(parent: Option<u32>, at: DVec3, mass_kg: f64, joint: JointSpec) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement: parent.map_or(Placement::ROOT, |p| Placement::child_of(ComponentId(p), at)),
        dry_mass_kg: mass_kg,
        dry_inertia_body: DMat3::from_diagonal(DVec3::new(0.8, 1.0, 1.2) * mass_kg),
        thermal_capacity_j_k: 10.0,
        nominal_power_w: 0.0,
        joint,
    }
}

fn posed(joint: JointState) -> ComponentState {
    ComponentState {
        joint,
        ..ComponentState::at_temperature(293.15)
    }
}

/// Chassis with two hinged rockers, moving.
fn rover() -> (ComponentTree, FloatingBaseState) {
    let rocker = |y: f64, q: f64, rate: f64| {
        (
            part(
                Some(0),
                DVec3::new(0.0, y, 0.0),
                10.0,
                JointSpec::revolute(DVec3::Y, -1.0, 1.0),
            ),
            posed(JointState::revolute(q, rate)),
        )
    };
    let tree = ComponentTree::new([
        (
            part(None, DVec3::ZERO, 100.0, JointSpec::FIXED),
            posed(JointState::FIXED),
        ),
        rocker(-0.5, 0.2, 0.3),
        rocker(0.5, -0.1, -0.4),
    ])
    .unwrap();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(5.0, -2.0, 1.0),
        rotation_world_from_root: DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.1, 0.4),
        linear_vel_world_m_s: DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.1, 0.4)
            * DVec3::new(1.0, 0.2, -0.3),
        angular_vel_root_rad_s: DVec3::new(0.3, -0.2, 0.5),
    };
    (tree, base)
}

/// Base and joint accelerations of `tree` loaded by `elements`.
fn accelerations<I, E: ForceElement<I>>(
    tree: &ComponentTree,
    base: &FloatingBaseState,
    elements: &E,
    inputs: &I,
) -> Vec<f64> {
    let model =
        MultibodyModel::with_derived_scale(tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let state = model.state(tree, base, elements, inputs).unwrap();
    let mut work = Workspace::new();
    let mut rates = vec![0.0; state.as_slice().len()];
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            elements,
            inputs,
            &mut work,
            &mut rates,
        )
        .unwrap();
    let (w, v) = (
        work.base_angular_accel_root_rad_s2(),
        work.base_linear_accel_world_m_s2(),
    );
    let mut out = vec![w.x, w.y, w.z, v.x, v.y, v.z];
    out.extend(work.joint_accels().iter().map(|j| j.q_accel_scalar));
    out
}

fn assert_close(a: &[f64], b: &[f64], tol: f64) {
    assert_eq!(a.len(), b.len());
    let scale = b.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    assert!(scale > 0.0, "nothing was loaded");
    for (k, (x, y)) in a.iter().zip(b).enumerate() {
        assert!((x - y).abs() < tol * scale, "[{k}]: {x} vs {y}");
    }
}

#[test]
fn loop_constraints_as_elements_match_their_tree_evaluation() {
    let (tree, base) = rover();
    let constraints = vec![
        LoopConstraint::JointRatio {
            joint_a: ComponentId(1),
            joint_b: ComponentId(2),
            ratio: 1.0,
            offset: 0.0,
            stiffness: 500.0,
            damping: 10.0,
        },
        LoopConstraint::UnilateralDistance {
            site_a: ComponentId(1),
            offset_a_m: DVec3::X,
            site_b: ComponentId(2),
            offset_b_m: DVec3::X,
            max_length_m: 1.0,
            stiffness_n_m: 1_000.0,
            damping_n_s_m: 10.0,
        },
        LoopConstraint::PointWeld {
            site_a: ComponentId(1),
            offset_a_m: DVec3::new(0.3, 0.0, 0.1),
            site_b: ComponentId(0),
            offset_b_m: DVec3::new(0.3, -0.5, 0.0),
            linear_stiffness_n_m: 800.0,
            linear_damping_n_s_m: 20.0,
        },
    ];
    // The loads the tree-side evaluation produced for this rover, recorded
    // before that evaluation was folded into the element.
    let wrench = |site, at: [f64; 3], f: [f64; 3]| WrenchAtSite {
        site: ComponentId(site),
        point_local_m: DVec3::from_array(at),
        force_local_n: DVec3::from_array(f),
        torque_local_nm: DVec3::ZERO,
    };
    let wrenches = vec![
        wrench(
            1,
            [1.0, 0.0, 0.0],
            [-1.9552300532214935, 43.77690009010112, 12.936958561619504],
        ),
        wrench(
            2,
            [1.0, 0.0, 0.0],
            [-1.9552300532214884, -43.77690009010113, -12.936958561619502],
        ),
        wrench(
            1,
            [0.3, 0.0, 0.1],
            [-5.384021318102016, 0.0, -30.519360609185327],
        ),
        wrench(
            0,
            [0.3, -0.5, 0.0],
            [11.339960296776532, 0.0, 28.84136539789338],
        ),
    ];
    let joint_loads = vec![
        JointLoadInput::scalar(ComponentId(1), -49.0),
        JointLoadInput::scalar(ComponentId(2), -49.0),
    ];
    let reference = accelerations(&tree, &base, &(wrenches, joint_loads), &());
    let as_elements = accelerations(&tree, &base, &constraints, &());
    // The element reads root-frame poses back from world kinematics, a
    // different but equivalent chain of rotations: rounding only.
    assert_close(&as_elements, &reference, 1e-11);
}

#[test]
fn hull_as_an_element_matches_its_base_state_evaluation() {
    let (tree, mut base) = rover();
    base.position_world_m = DVec3::new(0.0, 0.0, -0.4);
    let stations = vec![
        HullStation {
            offset_local_m: DVec3::new(0.0, 0.0, 0.3),
            cross_area_m2: 12.0,
            thickness_m: 1.5,
            drag_coeff: 1.3,
            slamming_coeff: 2.5,
        },
        HullStation {
            offset_local_m: DVec3::new(0.0, 1.0, 0.1),
            cross_area_m2: 2.0,
            thickness_m: 1.0,
            drag_coeff: 1.0,
            slamming_coeff: 1.0,
        },
    ];
    let hull = HydrodynamicsModel::new(ComponentId(0), stations).unwrap();
    let water = WaterState {
        surface_origin_world_m: DVec3::ZERO,
        outward_up_world: DVec3::Z,
        gravity_accel_m_s2: 9.80665,
        fluid_density_kg_m3: 1_025.0,
        fluid_vel_world_m_s: DVec3::new(0.2, 0.0, 0.0),
    };
    // Station loads the base-state evaluation produced, recorded before it
    // was folded into the element.
    let station = |at: [f64; 3], f: [f64; 3]| WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::from_array(at),
        force_local_n: DVec3::from_array(f),
        torque_local_nm: DVec3::ZERO,
    };
    let recorded = vec![
        station(
            [0.0, 0.0, 0.3],
            [1821.466032344521, 1797.2703974874003, 12828.365770848452],
        ),
        station(
            [0.0, 1.0, 0.1],
            [512.0997035106157, 443.56979677140134, 3088.384739690477],
        ),
    ];
    let reference = accelerations(&tree, &base, &recorded, &());
    let as_element = accelerations(&tree, &base, &hull, &water);
    assert_close(&as_element, &reference, 1e-12);
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let state = model.state(&tree, &base, &hull, &water).unwrap();
    let mut work = Workspace::new();
    let totals = hull
        .totals(&model.kinematics(&state, &mut work).unwrap(), &water, 0.0)
        .unwrap();
    assert!((totals.total_buoyancy_n - 16128.979901128469).abs() < 1e-9);
    assert!((totals.total_drag_n - 429.6108667673827).abs() < 1e-9);
}

#[test]
fn refreshed_mass_matches_a_fresh_preparation() {
    let (mut tree, base) = rover();
    // The scale is fixed at construction and survives the refresh, so both
    // sides are given the same one.
    let scale = MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81))
        .unwrap()
        .scale();
    let mut model = MultibodyModel::new(&tree, &[], scale).unwrap();
    // Propellant loaded into a rocker and a distributed mass on the chassis.
    let mut rocker = *tree.state(ComponentId(1)).unwrap();
    rocker.fluid_mass_kg = 7.5;
    tree.set_state(ComponentId(1), rocker).unwrap();
    let riding = [DynamicMassContribution {
        component: ComponentId(0),
        mass_kg: 30.0,
        cg_local_m: DVec3::new(0.3, 0.2, -0.4),
        inertia_cg_local: DMat3::from_diagonal(DVec3::new(2.0, 3.0, 4.0)),
    }];
    model.refresh_mass(&tree, &riding).unwrap();
    assert_eq!(model, MultibodyModel::new(&tree, &riding, scale).unwrap());
    let fresh = MultiBodyDynamics::system_momentum(&tree, &riding, &base).unwrap();
    let state = model.state(&tree, &base, &(), &()).unwrap();
    let refreshed = model
        .system_momentum(&state, &mut Workspace::new())
        .unwrap();
    assert_eq!(refreshed, fresh);
    assert!((fresh.total_mass_kg - 157.5).abs() < 1e-12);
}

fn ring_spec() -> DockingSpec {
    DockingSpec {
        capture_radius_m: 0.4,
        capture_angle_rad: 0.3,
        max_closing_speed_m_s: 0.5,
        damping_vel_threshold_m_s: 0.2,
        damping_rate_threshold_rad_s: 0.2,
        retraction_speed_m_s: 0.2,
        linear_stiffness_n_m: 1_000.0,
        linear_damping_n_s_m: 200.0,
        angular_stiffness_nm_rad: 500.0,
        angular_damping_nm_s_rad: 100.0,
        undock_impulse_ns: 300.0,
        min_capture_standoff_m: DockingSpec::DEFAULT_MIN_CAPTURE_STANDOFF_M,
        damped_dwell_s: DockingSpec::DEFAULT_DAMPED_DWELL_S,
        latch_standoff_tolerance_m: DockingSpec::DEFAULT_LATCH_STANDOFF_TOLERANCE_M,
        latch_distance_m: DockingSpec::DEFAULT_LATCH_DISTANCE_M,
    }
}

#[test]
fn docking_ring_as_an_element_captures_and_pulls_the_ports_together() {
    let tree = ComponentTree::new([(
        part(None, DVec3::ZERO, 300.0, JointSpec::FIXED),
        posed(JointState::FIXED),
    )])
    .unwrap();
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let spec = ring_spec();
    // Target port held 0.3 m ahead of the chaser's port along the axis.
    let ring = DockingCapture {
        spec,
        port_a: PortAnchor::Link {
            component: ComponentId(0),
            offset_local_m: DVec3::X,
        },
        port_b: PortAnchor::World(PortKinematics {
            position_world_m: DVec3::new(1.3, 0.0, 0.0),
            rotation_world: DQuat::IDENTITY,
            velocity_world_m_s: DVec3::ZERO,
            angular_vel_world_rad_s: DVec3::ZERO,
        }),
        axis_a_local: DVec3::X,
    };
    let mut state = model
        .state(&tree, &FloatingBaseState::AT_REST, &ring, &())
        .unwrap();
    let mut work = Workspace::new();
    let mut rk4 = Rk4::new();
    let phase = |state: &mbd_multibody::ModelState| {
        DockingState::from_blocks(state.auxiliary(), state.modes()).phase
    };
    assert_eq!(phase(&state), DockingPhase::Open);
    // The capture is an event of the first accepted boundary; the ring then
    // settles, retracts and draws the chaser's port onto the target.
    let mut seen = vec![phase(&state)];
    for i in 0..4_000 {
        let report = model
            .advance(
                &mut rk4,
                &ring,
                &(),
                &mut state,
                f64::from(i) * 0.005,
                0.005,
                &mut work,
            )
            .unwrap();
        if report.event {
            seen.push(phase(&state));
        }
        if phase(&state) == DockingPhase::HardLatched {
            break;
        }
    }
    // Clamping the stand-off at zero is an event that keeps the phase.
    seen.dedup();
    assert_eq!(
        seen,
        [
            DockingPhase::Open,
            DockingPhase::SoftContact,
            DockingPhase::Retracting,
            DockingPhase::HardLatched
        ]
    );
    let gap = (DVec3::new(1.3, 0.0, 0.0) - (state.base().position_world_m + DVec3::X)).length();
    assert!(gap <= spec.latch_distance_m, "gap {gap}");
}
