//! The zero-DoF `RigidFastPath` against the general ABA on the same rigid
//! tree. The general path is reached through the public API by adding a leaf
//! that transmits nothing to the rigid tree: a massless revolute link, or a
//! massive body on a zero-stiffness, zero-damping compliant joint.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentTree, JointSpec, Placement, WrenchAtSite};

use super::rigs::{diag, idle, part};
use crate::{FloatingBaseState, MultiBodyAccel, MultiBodyDynamics};

#[derive(Clone, Copy, PartialEq)]
enum Decoupled {
    None,
    MasslessHinge,
    FreeBody,
}

fn rigid_tree(leaf: Decoupled) -> ComponentTree {
    let mut items = vec![
        (
            part(
                "core",
                Placement::ROOT,
                200.0,
                diag(40.0, 60.0, 70.0),
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "pod",
                Placement {
                    parent: Some(ComponentId(0)),
                    translation_m: DVec3::new(2.0, -1.0, 0.5),
                    rotation: DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.4),
                },
                50.0,
                diag(8.0, 12.0, 15.0),
                JointSpec::FIXED,
            ),
            idle(),
        ),
        (
            part(
                "boom",
                Placement::child_of(ComponentId(1), DVec3::new(0.0, 1.5, -0.7)),
                12.0,
                diag(1.0, 0.5, 1.2),
                JointSpec::FIXED,
            ),
            idle(),
        ),
    ];
    let at = Placement::child_of(ComponentId(2), DVec3::new(0.3, 0.0, 0.0));
    let free = JointSpec::compliant_6dof(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
    let hinge = JointSpec::revolute(DVec3::Y, -1.0e6, 1.0e6);
    match leaf {
        Decoupled::None => {}
        Decoupled::MasslessHinge => {
            items.push((part("dummy", at, 0.0, DMat3::ZERO, hinge), idle()));
        }
        Decoupled::FreeBody => {
            items.push((part("free", at, 5.0, diag(1.0, 2.0, 3.0), free), idle()));
        }
    }
    ComponentTree::new(items).unwrap()
}

fn solve(leaf: Decoupled) -> MultiBodyAccel {
    let base = FloatingBaseState {
        position_world_m: DVec3::new(100.0, 0.0, 0.0),
        rotation_world_from_root: DQuat::from_rotation_y(0.4),
        linear_vel_world_m_s: DQuat::from_rotation_y(0.4) * DVec3::new(20.0, -5.0, 3.0),
        angular_vel_root_rad_s: DVec3::new(0.2, -0.4, 0.3),
    };
    let wrenches = [WrenchAtSite {
        site: ComponentId(2),
        point_local_m: DVec3::new(0.5, 0.0, 0.0),
        force_local_n: DVec3::new(120.0, -80.0, 50.0),
        torque_local_nm: DVec3::new(10.0, 20.0, -15.0),
    }];
    let tree = rigid_tree(leaf);
    let gravity = DVec3::new(0.0, 0.0, -9.81);
    MultiBodyDynamics::forward_dynamics(&tree, &[], &base, gravity, &[], &wrenches).unwrap()
}

fn assert_paths_agree(leaf: Decoupled) {
    let fast = solve(Decoupled::None);
    let general = solve(leaf);
    assert!(fast.used_rigid_fast_path && !general.used_rigid_fast_path);
    // Same composite inertia and wrench assembled in a different order;
    // 1e-12 relative to O(10) accelerations is a rounding-only bound.
    let pair = |a: &crate::MultiBodyAccel| {
        (
            a.base_angular_accel_root_rad_s2,
            a.base_linear_accel_world_m_s2,
        )
    };
    let (fa, ga) = (pair(&fast), pair(&general));
    let scale = fa.0.length().max(fa.1.length());
    assert!(
        (fa.0 - ga.0).length() < 1e-12 * scale && (fa.1 - ga.1).length() < 1e-12 * scale,
        "fast {fa:?} vs general {ga:?}"
    );
}

#[test]
fn rigid_fast_path_matches_general_aba_with_free_body_leaf() {
    assert_paths_agree(Decoupled::FreeBody);
}

/// Relies on the solver's `D <= 1e-12` branch treating a massless hinge as
/// transmitting nothing; if that branch becomes an error this test changes
/// meaning, while the free-body variant above does not.
#[test]
fn rigid_fast_path_matches_general_aba_with_massless_hinge_leaf() {
    assert_paths_agree(Decoupled::MasslessHinge);
}
