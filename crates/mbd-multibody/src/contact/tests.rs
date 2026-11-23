use glam::{DQuat, DVec3};
use mbd_core::ComponentId;

use super::*;

#[test]
fn contact_bristle_sticks_on_solid_slope_and_ignores_liquid_surface() {
    let spec = ContactPointSpec {
        component: ComponentId(0),
        offset_local_m: DVec3::ZERO,
        stiffness_n_m: 100_000.0,
        damping_n_s_m: 2_000.0,
        mu_static: 0.9,
        mu_dynamic: 0.7,
        bristle_stiffness_n_m: 50_000.0,
        bristle_damping_n_s_m: 1_000.0,
    };
    let mut state = ContactPointState::default();
    let surface = SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    };

    let out = evaluate_contact_point(
        &spec,
        &mut state,
        DVec3::new(0.0, 0.0, -0.02),
        DVec3::new(0.005, 0.0, 0.0),
        &surface,
        DQuat::IDENTITY,
        0.01,
    )
    .unwrap();
    assert!(out.normal_force_n > 1_900.0);
    assert!(out.is_sticking);
    assert!(out.wrench.force_local_n.x < 0.0);

    let water = SurfaceSample {
        medium: SurfaceMedium::Liquid,
        ..surface
    };
    let splash = evaluate_contact_point(
        &spec,
        &mut state,
        DVec3::new(0.0, 0.0, -0.02),
        DVec3::ZERO,
        &water,
        DQuat::IDENTITY,
        0.01,
    )
    .unwrap();
    assert_eq!(splash.normal_force_n, 0.0);
}

#[test]
fn pinned_surface_rest_latches_and_breaks_free_on_ascent_ignition() {
    let mut rest = PinnedSurfaceRest::Free;
    for _ in 0..20 {
        step_pinned_surface_rest(
            &mut rest,
            true,
            0.001,
            0.001,
            DVec3::ZERO,
            DQuat::IDENTITY,
            100.0,
            15_000.0,
            0.5,
            0.05,
        );
    }
    assert!(matches!(rest, PinnedSurfaceRest::Pinned { .. }));
    step_pinned_surface_rest(
        &mut rest,
        true,
        0.0,
        0.0,
        DVec3::ZERO,
        DQuat::IDENTITY,
        16_000.0,
        15_000.0,
        0.5,
        0.05,
    );
    assert_eq!(rest, PinnedSurfaceRest::Free);
}
