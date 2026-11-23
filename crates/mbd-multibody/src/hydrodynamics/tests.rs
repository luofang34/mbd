use glam::{DQuat, DVec3};
use mbd_core::ComponentId;

use super::*;

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

    // High-speed entry splashdown crossing interface gives strong slamming deceleration!
    let entry = hydro
        .evaluate(
            &FloatingBaseState {
                position_world_m: DVec3::new(0.0, 0.0, -0.7),
                rotation_world_from_root: DQuat::IDENTITY,
                linear_vel_root_m_s: DVec3::new(0.0, 0.0, -10.0),
                angular_vel_root_rad_s: DVec3::ZERO,
            },
            &water,
        )
        .unwrap();
    assert!(entry.total_drag_n > 400_000.0);
    assert!(entry.total_buoyancy_n > 30_000.0);

    // Floating tilted generates upright restoring roll torque from positive metacentric height!
    let tilted = hydro
        .evaluate(
            &FloatingBaseState {
                position_world_m: DVec3::new(0.0, 0.0, -0.3),
                rotation_world_from_root: DQuat::from_rotation_x(0.25),
                linear_vel_root_m_s: DVec3::ZERO,
                angular_vel_root_rad_s: DVec3::ZERO,
            },
            &water,
        )
        .unwrap();
    let net_moment_local: DVec3 = tilted
        .wrenches
        .iter()
        .map(|w| w.point_local_m.cross(w.force_local_n))
        .sum();
    assert!(
        net_moment_local.x < 0.0,
        "righting moment opposes +x roll: {:?}, wrenches={:?}",
        net_moment_local,
        tilted.wrenches
    );
}
