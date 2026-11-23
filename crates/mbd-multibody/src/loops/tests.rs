use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};

use super::*;

fn link_spec(
    name: &str,
    parent: Option<ComponentId>,
    pos: DVec3,
    joint: JointSpec,
) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement: Placement {
            parent,
            translation_m: pos,
            rotation: glam::DQuat::IDENTITY,
        },
        dry_mass_kg: 10.0,
        dry_inertia_body: DMat3::from_diagonal(DVec3::ONE),
        thermal_capacity_j_k: 10.0,
        nominal_power_w: 0.0,
        joint,
    }
}

#[test]
fn rocker_bogie_differential_and_weld_loops_conserve_internal_equilibrium() {
    let mut tree = ComponentTree::new([
        (
            link_spec("chassis", None, DVec3::ZERO, JointSpec::FIXED),
            ComponentState::at_temperature(293.15),
        ),
        (
            link_spec(
                "rocker_l",
                Some(ComponentId(0)),
                DVec3::new(0.0, -0.5, 0.0),
                JointSpec::revolute(DVec3::Y, -1.0, 1.0),
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            link_spec(
                "rocker_r",
                Some(ComponentId(0)),
                DVec3::new(0.0, 0.5, 0.0),
                JointSpec::revolute(DVec3::Y, -1.0, 1.0),
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();

    // Climb on left rocker (q_l = +0.2 rad, q_r = 0) -> differential pushes q_l negative and q_r negative.
    let mut sl = *tree.state(ComponentId(1)).unwrap();
    sl.joint.q_scalar = 0.2;
    tree.set_state(ComponentId(1), sl).unwrap();

    let poses = tree.evaluate_pose_table().unwrap();
    let out = evaluate_loop_constraints(
        &tree,
        &poses,
        &[
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
        ],
    )
    .unwrap();

    assert_eq!(out.joint_loads.len(), 2);
    assert!(out.joint_loads[0].effort_scalar < 0.0);
    assert!(out.joint_loads[1].effort_scalar < 0.0);

    let net_root: (DVec3, DVec3) =
        out.wrenches
            .iter()
            .fold((DVec3::ZERO, DVec3::ZERO), |(f, m), w| {
                let p = poses.get(w.site).unwrap();
                let (fr, mr) = w.to_root_wrench(p, DVec3::ZERO);
                (f + fr, m + mr)
            });
    assert!(net_root.0.length() < 1e-12);
    assert!(net_root.1.length() < 1e-12);
}
