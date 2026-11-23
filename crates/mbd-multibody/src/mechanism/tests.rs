use core::f64::consts::TAU;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution, JointSpec,
    Placement,
};

use super::*;
use crate::aba::{FloatingBaseState, MultiBodyDynamics, StageForces, step_rk4_with_loads};

fn m_spec(
    name: &str,
    parent: Option<ComponentId>,
    pos: DVec3,
    mass_kg: f64,
    diag: DVec3,
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement: Placement {
            parent,
            translation_m: pos,
            rotation: DQuat::IDENTITY,
        },
        dry_mass_kg: mass_kg,
        dry_inertia_body: DMat3::from_diagonal(diag),
        thermal_capacity_j_k: 100.0,
        nominal_power_w: 0.0,
        joint,
    }
}

fn combine_momentum(
    t1: &ComponentTree,
    m1: &[DynamicMassContribution],
    s1: &FloatingBaseState,
    t2: &ComponentTree,
    m2: &[DynamicMassContribution],
    s2: &FloatingBaseState,
) -> (DVec3, DVec3) {
    let a = MultiBodyDynamics::system_momentum(t1, m1, s1).unwrap();
    let b = MultiBodyDynamics::system_momentum(t2, m2, s2).unwrap();
    let m_tot = a.total_mass_kg + b.total_mass_kg;
    let com = (a.com_world_m * a.total_mass_kg + b.com_world_m * b.total_mass_kg) / m_tot;
    let p = a.linear_world_kg_m_s + b.linear_world_kg_m_s;
    let l = a.angular_about_com_world_nms
        + (a.com_world_m - com).cross(a.linear_world_kg_m_s)
        + b.angular_about_com_world_nms
        + (b.com_world_m - com).cross(b.linear_world_kg_m_s);
    (p, l)
}

#[test]
fn off_cg_impulse_release_conserves_momentum() {
    let tree = ComponentTree::new([
        (
            m_spec(
                "stage",
                None,
                DVec3::ZERO,
                1_200.0,
                DVec3::new(300.0, 900.0, 900.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            m_spec(
                "payload",
                Some(ComponentId(0)),
                DVec3::new(3.0, 0.4, -0.2),
                400.0,
                DVec3::new(50.0, 120.0, 130.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    let base = FloatingBaseState {
        position_world_m: DVec3::new(1000.0, 0.0, 0.0),
        rotation_world_from_root: DQuat::from_rotation_z(0.5),
        linear_vel_root_m_s: DVec3::new(100.0, -10.0, 5.0),
        angular_vel_root_rad_s: DVec3::new(0.1, -0.2, 0.3),
    };
    let pre = MultiBodyDynamics::system_momentum(&tree, &[], &base).unwrap();
    let split =
        execute_impulse_release(&tree, &[], &base, ComponentId(1), 800.0, DVec3::X).unwrap();
    let (p_post, l_post) = combine_momentum(
        &split.parent_tree,
        &split.parent_masses,
        &split.parent_state,
        &split.child_tree,
        &split.child_masses,
        &split.child_state,
    );
    let p_tol = 1e-12 * pre.linear_world_kg_m_s.length().max(1.0);
    let l_tol = 1e-12 * pre.angular_about_com_world_nms.length().max(1.0);
    assert!(
        (p_post - pre.linear_world_kg_m_s).length() < p_tol,
        "p_post={p_post:?} vs pre={:?}",
        pre.linear_world_kg_m_s
    );
    assert!(
        (l_post - pre.angular_about_com_world_nms).length() < l_tol,
        "l_post={l_post:?} vs pre={:?}",
        pre.angular_about_com_world_nms
    );
}

#[test]
fn rail_slide_release_ejects_seat_and_conserves_momentum() {
    let mut rtree = ComponentTree::new([
        (
            m_spec(
                "capsule",
                None,
                DVec3::ZERO,
                2_000.0,
                DVec3::splat(400.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            m_spec(
                "ejection_seat",
                Some(ComponentId(0)),
                DVec3::new(0.0, 0.0, -0.5),
                120.0,
                DVec3::splat(15.0),
                JointSpec::prismatic(-DVec3::Z, -0.1, 2.0),
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    let rail = RailSlideSpec {
        component: ComponentId(1),
        rail_length_m: 1.5,
        catapult_force_n: 12_000.0,
        guide_friction_n: 200.0,
    };
    let mut rst = RailSlideState {
        fired: true,
        released: false,
    };
    let mut rbase = FloatingBaseState::AT_REST;
    let init_r = MultiBodyDynamics::system_momentum(&rtree, &[], &rbase).unwrap();
    let mut ready = false;
    for _ in 0..40 {
        let out = step_rail_drive(&rail, &mut rst, &rtree).unwrap();
        if out.ready_to_release {
            ready = true;
            break;
        }
        let forces = StageForces {
            joint_drives: out.drive.into_iter().collect(),
            joint_loads: out.load.into_iter().collect(),
            ..StageForces::default()
        };
        rbase = step_rk4_with_loads::<crate::MultibodyError, _>(
            &mut rtree,
            &[],
            &rbase,
            0.01,
            |_, _, _| Ok(forces.clone()),
        )
        .unwrap();
    }
    assert!(ready);
    let ejected = release_from_rail(&rail, &mut rst, &rtree, &[], &rbase).unwrap();
    let (pr, lr) = combine_momentum(
        &ejected.parent_tree,
        &ejected.parent_masses,
        &ejected.parent_state,
        &ejected.child_tree,
        &ejected.child_masses,
        &ejected.child_state,
    );
    assert!((pr - init_r.linear_world_kg_m_s).length() < 1e-11);
    assert!((lr - init_r.angular_about_com_world_nms).length() < 1e-11);
    assert!(ejected.child_state.linear_vel_world_m_s().z < -10.0);
}

fn gemini_and_agena_fixtures() -> (ComponentTree, ComponentTree) {
    let gemini = ComponentTree::new([
        (
            m_spec(
                "gemini",
                None,
                DVec3::ZERO,
                3_200.0,
                DVec3::new(900.0, 2_200.0, 2_200.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            m_spec(
                "nose_port",
                Some(ComponentId(0)),
                DVec3::X * 2.5,
                50.0,
                DVec3::ZERO,
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    let agena = ComponentTree::new([
        (
            m_spec(
                "agena",
                None,
                DVec3::ZERO,
                3_100.0,
                DVec3::new(800.0, 3_000.0, 3_000.0),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            m_spec(
                "target_adapter",
                Some(ComponentId(0)),
                DVec3::X * 2.0,
                40.0,
                DVec3::ZERO,
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    (gemini, agena)
}

#[test]
fn gemini8_docking_and_60rpm_undocking_conserve_momentum() {
    let (gemini, agena) = gemini_and_agena_fixtures();

    let sa = FloatingBaseState {
        position_world_m: DVec3::ZERO,
        rotation_world_from_root: DQuat::IDENTITY,
        linear_vel_root_m_s: DVec3::new(10.0, 2.0, -1.0),
        angular_vel_root_rad_s: DVec3::new(TAU, 0.5, -0.3),
    };
    let sb = FloatingBaseState {
        position_world_m: DVec3::new(4.5, 0.0, 0.0),
        rotation_world_from_root: DQuat::from_rotation_y(std::f64::consts::PI),
        linear_vel_root_m_s: DVec3::new(-9.8, 0.0, 0.0),
        angular_vel_root_rad_s: DVec3::ZERO,
    };
    let (p_before, l_before) = combine_momentum(&gemini, &[], &sa, &agena, &[], &sb);
    let mated = hard_mate_vessels(
        &gemini,
        &[],
        &sa,
        ComponentId(1),
        &agena,
        &[],
        &sb,
        ComponentId(1),
        Placement {
            parent: None,
            translation_m: DVec3::ZERO,
            rotation: DQuat::from_rotation_y(std::f64::consts::PI),
        },
    )
    .unwrap();
    let mom_mated =
        MultiBodyDynamics::system_momentum(&mated.tree, &mated.dynamic_masses, &mated.base_state)
            .unwrap();
    assert!(
        (mom_mated.linear_world_kg_m_s - p_before).length() < 1e-11,
        "mated p={:?} vs {p_before:?}",
        mom_mated.linear_world_kg_m_s
    );
    assert!(
        (mom_mated.angular_about_com_world_nms - l_before).length() < 1e-10,
        "mated l={:?} vs {l_before:?}",
        mom_mated.angular_about_com_world_nms
    );

    let sep = undock_vessel(
        &mated.tree,
        &mated.dynamic_masses,
        &mated.base_state,
        mated.b_id_map[1].1,
        300.0,
        DVec3::X,
    )
    .unwrap();
    let (p_sep, l_sep) = combine_momentum(
        &sep.parent_tree,
        &sep.parent_masses,
        &sep.parent_state,
        &sep.child_tree,
        &sep.child_masses,
        &sep.child_state,
    );
    assert!((p_sep - p_before).length() < 1e-11);
    assert!((l_sep - l_before).length() < 1e-10);
}

#[test]
fn soft_capture_ring_exerts_balanced_action_reaction_wrench() {
    let dock_spec = DockingSpec {
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
    };
    let mut dst = DockingState::default();
    let soft = step_docking_capture(
        &dock_spec,
        &mut dst,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::X * 0.2,
        DQuat::IDENTITY,
        DVec3::X * -0.05,
        DVec3::ZERO,
        DVec3::X,
        0.1,
    );
    assert_eq!(soft.phase, DockingPhase::SoftContact);
    assert!((soft.world_wrench_on_a.force + soft.world_wrench_on_b.force).length() < 1e-12);
}

#[test]
fn tether_models_vostok_burn_through_and_sky_crane_autocut() {
    let vostok_spec = WinchTetherSpec {
        stiffness_n_m: 50_000.0,
        damping_n_s_m: 1_000.0,
        max_length_m: 2.0,
        breaking_tension_n: 200_000.0,
        burn_through_energy_j: Some(1_000.0),
        auto_cut_low_tension_n: None,
        auto_cut_dwell_s: 0.0,
    };
    let mut vst = WinchTetherState::intact_at(1.0);
    let _ = step_tether(
        &vostok_spec,
        &mut vst,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::X * 1.1,
        DVec3::ZERO,
        600.0,
        1.0,
    )
    .unwrap();
    let out2 = step_tether(
        &vostok_spec,
        &mut vst,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::X * 1.1,
        DVec3::ZERO,
        600.0,
        1.0,
    )
    .unwrap();
    assert!(out2.severed_this_step);
    assert_eq!(vst.cut_reason, Some(TetherCutReason::ThermalBurnThrough));

    let sky_crane = WinchTetherSpec {
        stiffness_n_m: 20_000.0,
        damping_n_s_m: 500.0,
        max_length_m: 10.0,
        breaking_tension_n: 50_000.0,
        burn_through_energy_j: None,
        auto_cut_low_tension_n: Some(150.0),
        auto_cut_dwell_s: 0.2,
    };
    let mut sst = WinchTetherState {
        payout_rate_m_s: 1.5,
        ..WinchTetherState::intact_at(2.0)
    };
    let _ = step_tether(
        &sky_crane,
        &mut sst,
        DVec3::ZERO,
        DVec3::ZERO,
        DVec3::new(0.0, 0.0, 3.6),
        DVec3::ZERO,
        0.0,
        1.0,
    )
    .unwrap();
    assert!((sst.deployed_length_m - 3.5).abs() < 1e-12);
    sst.payout_rate_m_s = 0.0;
    for _ in 0..3 {
        let _ = step_tether(
            &sky_crane,
            &mut sst,
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 3.3),
            DVec3::ZERO,
            0.0,
            0.1,
        )
        .unwrap();
    }
    assert!(!sst.intact);
    assert_eq!(sst.cut_reason, Some(TetherCutReason::TouchdownAutoCut));
}
