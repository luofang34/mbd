//! The 10-tonne end of the scale: a swing-wing vehicle with a rotor, a
//! dummy and a parachute canopy, each element on its own link, against the
//! per-link evaluation recorded from the previous implementation.

use std::f64::consts::PI;

use glam::{DMat3, DQuat};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState, Placement,
};
use mbd_multibody::{FloatingBaseState, ModelState, MultibodyModel, Rk4, TimeScale, Workspace};

use super::*;
use crate::{AeroEnvironment, AirSample};

const HEAVY_RIGID: Pin = (
    [0.0, 0.0, 0.0],
    [-3452859.389324588, -191307.57401210154, -10288955.55573172],
    [-19071728.59623174, 5189636.668998008, 1074132.9945557413],
    4333.473412322608,
);
const HEAVY_SWEPT: Pin = (
    [0.0, 0.0, 0.0],
    [-3350236.7208321984, -63258.57660052651, -8914707.392822186],
    [-17143467.283216175, -3978349.2425329005, 1402308.8836647903],
    4333.473412322608,
);
const HEAVY_SWEEPING: Pin = (
    [0.0, 0.0, 0.0],
    [-3350346.3105434207, -63333.55095578913, -8845963.91998841],
    [-16550499.472288417, -3857298.7843767414, 1368591.7191718516],
    4333.473412322608,
);

/// Root sums and accelerations agree with the recorded root wrench to
/// rounding: the chain of rotations differs (world kinematics, not a root
/// pose table), the arithmetic per element does not.
const REL: f64 = 1e-12;

fn spec(parent: Option<u32>, at: DVec3, rot: DQuat, mass: f64, joint: JointSpec) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement: Placement {
            parent: parent.map(ComponentId),
            translation_m: at,
            rotation: rot,
        },
        dry_mass_kg: mass,
        dry_inertia_body: DMat3::from_diagonal(DVec3::new(0.8, 1.0, 1.2) * mass),
        thermal_capacity_j_k: 10.0,
        nominal_power_w: 0.0,
        joint,
    }
}

/// Fuselage, two swing wings on vertical hinges, a canopy riser and a
/// pod carrying a rotor and a dummy: 10 000 kg.
fn heavy_tree(swing_q: f64, swing_rate: f64) -> ComponentTree {
    hinged_tree(
        swing_q,
        swing_rate,
        JointSpec::revolute(DVec3::Z, -1.2, 1.2),
    )
}

fn hinged_tree(swing_q: f64, swing_rate: f64, swing: JointSpec) -> ComponentTree {
    let st = |j| ComponentState {
        joint: j,
        ..ComponentState::at_temperature(293.15)
    };
    let fixed = JointSpec::FIXED;
    let one = DQuat::IDENTITY;
    ComponentTree::new([
        (
            spec(None, DVec3::ZERO, one, 8_000.0, fixed),
            st(JointState::FIXED),
        ),
        (
            spec(Some(0), DVec3::new(1.0, 2.0, 0.0), one, 600.0, swing),
            st(JointState::revolute(swing_q, swing_rate)),
        ),
        (
            spec(
                Some(0),
                DVec3::new(1.0, -2.0, 0.0),
                DQuat::from_rotation_x(PI),
                600.0,
                swing,
            ),
            st(JointState::revolute(-swing_q, -swing_rate)),
        ),
        (
            spec(
                Some(0),
                DVec3::new(-9.0, 0.0, 0.5),
                DQuat::from_rotation_y(0.3),
                300.0,
                fixed,
            ),
            st(JointState::FIXED),
        ),
        (
            spec(
                Some(0),
                DVec3::new(-4.0, 0.0, -1.5),
                DQuat::from_rotation_z(0.4),
                500.0,
                fixed,
            ),
            st(JointState::FIXED),
        ),
    ])
    .unwrap()
}

fn elements() -> Vec<AeroElement> {
    [
        (0, body_fin()),
        (1, wing()),
        (2, wing()),
        (3, chute()),
        (4, rotor()),
        (4, dummy()),
    ]
    .into_iter()
    .map(|(c, kind)| AeroElement::new(ComponentId(c), kind))
    .collect()
}

fn base() -> FloatingBaseState {
    FloatingBaseState {
        linear_vel_root_m_s: DVec3::new(250.0, 10.0, 25.0),
        angular_vel_root_rad_s: rate(),
        ..FloatingBaseState::AT_REST
    }
}

fn env() -> AeroEnvironment<'static> {
    AeroEnvironment {
        air: AirSample {
            ambient: sea(),
            wind_world_m_s: DVec3::ZERO,
        },
        controls: &CONTROLS,
    }
}

fn prepared(tree: &ComponentTree) -> (MultibodyModel, ModelState) {
    let model =
        MultibodyModel::with_derived_scale(tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let state = model.state(tree, &base(), &elements(), &env()).unwrap();
    (model, state)
}

/// Sum of every element's link wrench, moved to the root origin in root
/// axes.
fn root_total(tree: &ComponentTree) -> AeroWrench {
    let (model, state) = prepared(tree);
    let mut work = Workspace::new();
    let kin = model.kinematics(&state, &mut work).unwrap();
    let (mut force, mut moment) = (DVec3::ZERO, DVec3::ZERO);
    let mut aux = state.auxiliary();
    let mut modes = state.modes();
    for e in elements() {
        let n = ForceElement::<AeroEnvironment<'_>>::state_len(&e);
        let (s, m) = (&aux[..n], &modes[..n]);
        (aux, modes) = (&aux[n..], &modes[n..]);
        if !kin.is_attached(e.component) {
            continue;
        }
        let w = e
            .loads(&e.flow(&kin, &env()).unwrap(), s, m)
            .unwrap()
            .wrench;
        let pose = kin.link_pose_in_root(e.component).unwrap();
        let f_root = pose.rotation_root_from_link * w.force_link_n;
        let at = pose.position_root_m + pose.rotation_root_from_link * w.reference_point_link_m;
        force += f_root;
        moment += pose.rotation_root_from_link * w.moment_ref_link_nm + at.cross(f_root);
    }
    AeroWrench {
        reference_point_link_m: DVec3::ZERO,
        force_link_n: force,
        moment_ref_link_nm: moment,
        stagnation_heat_flux_w_m2: HEAVY_RIGID.3,
    }
}

#[test]
fn every_link_loads_as_the_recorded_per_link_evaluation() {
    assert_pin(
        "rigid",
        &root_total(&heavy_tree(0.0, 0.0)),
        &HEAVY_RIGID,
        REL,
    );
    assert_pin(
        "swept",
        &root_total(&heavy_tree(0.6, 0.0)),
        &HEAVY_SWEPT,
        REL,
    );
    let sweeping = root_total(&heavy_tree(0.6, 0.8));
    assert_pin("sweeping", &sweeping, &HEAVY_SWEEPING, REL);
}

/// Generalized accelerations of `tree` at the pinned base state.
fn accelerations<E: ForceElement<AeroEnvironment<'static>>>(
    tree: &ComponentTree,
    elements: &E,
) -> Vec<f64> {
    let model =
        MultibodyModel::with_derived_scale(tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let state = model.state(tree, &base(), elements, &env()).unwrap();
    let mut work = Workspace::new();
    let mut rates = vec![0.0; state.as_slice().len()];
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            elements,
            &env(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    let a = work.base_accel();
    let mut out = vec![a.ang.x, a.ang.y, a.ang.z, a.lin.x, a.lin.y, a.lin.z];
    out.extend(work.joint_accels().iter().map(|j| j.q_accel_scalar));
    out
}

fn lumped_root_wrench() -> mbd_core::WrenchAtSite {
    mbd_core::WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: DVec3::from_array(HEAVY_RIGID.1),
        torque_local_nm: DVec3::from_array(HEAVY_RIGID.2),
    }
}

fn assert_close(a: &[f64], b: &[f64], rel: f64) {
    let scale = b.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    for (k, (x, y)) in a.iter().zip(b).enumerate() {
        assert!((x - y).abs() <= rel * scale, "[{k}] {x} vs {y}");
    }
}

#[test]
fn a_rigid_vehicle_moves_as_under_the_recorded_root_wrench() {
    // Same geometry with the hinges welded: the vehicle is one rigid body.
    let tree = hinged_tree(0.0, 0.0, JointSpec::FIXED);
    let by_link = accelerations(&tree, &elements());
    let lumped = accelerations(&tree, &[lumped_root_wrench()]);
    assert_eq!(by_link.len(), 6);
    assert_close(&by_link, &lumped, REL);
}

#[test]
fn a_swing_wing_hinge_feels_its_own_air_load() {
    // With free hinges the same total load gives a different motion: each
    // wing's load acts about its hinge instead of on the root.
    let tree = heavy_tree(0.0, 0.0);
    let by_link = accelerations(&tree, &elements());
    let lumped = accelerations(&tree, &[lumped_root_wrench()]);
    let (hinge, hinge_lumped) = (by_link[6], lumped[6]);
    assert!(
        (hinge - hinge_lumped).abs() > 0.5 * hinge.abs(),
        "hinge {hinge} vs lumped {hinge_lumped}"
    );
}

#[test]
fn a_commanded_canopy_latches_and_inflates_with_the_vehicle() {
    let AeroElementKind::ParachuteCanopy(m) = chute() else {
        panic!("the chute fixture is a canopy")
    };
    let stowed = ParachuteParams {
        deploy_channel: Some(11),
        deployed_at_start: false,
        inflation_at_start: 0.0,
        ..*m.params()
    };
    let canopy = [AeroElement::new(
        ComponentId(3),
        AeroElementKind::ParachuteCanopy(ParachuteCanopyAero::new(stowed).unwrap()),
    )];
    let tree = hinged_tree(0.0, 0.0, JointSpec::FIXED);
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let mut controls = CONTROLS.to_vec();
    controls.push(1.0);
    let fire = AeroEnvironment {
        controls: &controls,
        ..env()
    };
    let mut state = model.state(&tree, &base(), &canopy, &fire).unwrap();
    let mut work = Workspace::new();
    let dt = 1e-3;
    let mut rk4 = Rk4::new();
    model
        .advance(&mut rk4, &canopy, &fire, &mut state, 0.0, dt, &mut work)
        .unwrap();
    assert_eq!(state.modes(), &[1.0], "deployment latches at the boundary");
    assert_eq!(state.auxiliary(), &[0.0], "and inflation starts after it");
    for k in 1..500 {
        let t = f64::from(k) * dt;
        model
            .advance(&mut rk4, &canopy, &fire, &mut state, t, dt, &mut work)
            .unwrap();
    }
    let f = state.auxiliary()[0];
    assert!((0.2..1.0).contains(&f), "inflation after 0.5 s: {f}");
}
