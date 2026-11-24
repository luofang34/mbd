use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};

use super::*;
use crate::{FloatingBaseState, MultibodyModel, TimeScale, Workspace};

#[test]
fn capsule_splashdown_floats_at_equilibrium_draft_and_experiences_entry_slam() {
    // 5-ton Apollo/Gemini capsule with low CG (-X offset or heat-shield down along -Z):
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
        HullStation {
            offset_local_m: DVec3::new(0.0, -1.0, 0.1),
            cross_area_m2: 2.0,
            thickness_m: 1.0,
            drag_coeff: 1.0,
            slamming_coeff: 1.0,
        },
    ];
    let hydro = HydrodynamicsModel::new(ComponentId(0), stations).unwrap();
    let water = WaterState {
        surface_origin_world_m: DVec3::ZERO,
        outward_up_world: DVec3::Z,
        gravity_accel_m_s2: 9.80665,
        fluid_density_kg_m3: 1_025.0,
        fluid_vel_world_m_s: DVec3::ZERO,
    };

    let tree = ComponentTree::new([(
        ComponentSpec {
            name: "capsule".to_owned(),
            placement: Placement::ROOT,
            dry_mass_kg: 5_000.0,
            dry_inertia_body: DMat3::from_diagonal(DVec3::splat(5_000.0)),
            thermal_capacity_j_k: 1.0,
            nominal_power_w: 0.0,
            joint: JointSpec::FIXED,
        },
        ComponentState::at_temperature(293.15),
    )])
    .unwrap();
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let totals = |base: FloatingBaseState| {
        let state = model.state(&tree, &base, &hydro, &water).unwrap();
        let mut work = Workspace::new();
        let kin = model.kinematics(&state, &mut work).unwrap();
        hydro.totals(&kin, &water, 0.0).unwrap()
    };

    // High-speed entry splashdown crossing interface gives strong slamming deceleration!
    let entry = totals(FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, -0.7),
        rotation_world_from_root: DQuat::IDENTITY,
        linear_vel_world_m_s: DVec3::new(0.0, 0.0, -10.0),
        angular_vel_root_rad_s: DVec3::ZERO,
    });
    assert!(entry.total_drag_n > 400_000.0);
    assert!(entry.total_buoyancy_n > 30_000.0);

    // Floating tilted generates upright restoring roll torque from positive metacentric height!
    let tilted = totals(FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, -0.3),
        rotation_world_from_root: DQuat::from_rotation_x(0.25),
        linear_vel_world_m_s: DVec3::ZERO,
        angular_vel_root_rad_s: DVec3::ZERO,
    });
    assert!(
        tilted.moment_local_nm.x < 0.0,
        "righting moment opposes +x roll: {tilted:?}"
    );
}
