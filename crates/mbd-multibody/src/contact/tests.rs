use glam::{DMat3, DQuat, DVec3};
use mbd_core::ComponentId;

use super::*;

/// One explicit-Euler step of a contact point at a prescribed point motion:
/// evaluate, advance the bristle, apply the boundary events.
fn step_contact_point(
    spec: &ContactPointSpec,
    state: &mut ContactPointState,
    point_world_m: DVec3,
    vel_world_m_s: DVec3,
    surface: &SurfaceSample,
    dt_s: f64,
) -> ContactEvaluation {
    let point = PointKinematics {
        position_world_m: point_world_m,
        velocity_world_m_s: vel_world_m_s,
        angular_vel_world_rad_s: DVec3::ZERO,
        rotation_world_from_link: DMat3::IDENTITY,
    };
    let out = spec.evaluate(state, &point, surface);
    state.tangent_deflection_world_m += out.deflection_rate_world_m_s * dt_s;
    spec.transition(state, &point, surface);
    out
}

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
        damping_ramp_depth_m: ContactPointSpec::DEFAULT_DAMPING_RAMP_DEPTH_M,
    };
    let mut state = ContactPointState::default();
    let surface = SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        velocity_world_m_s: DVec3::ZERO,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    };

    let out = step_contact_point(
        &spec,
        &mut state,
        DVec3::new(0.0, 0.0, -0.02),
        DVec3::new(0.005, 0.0, 0.0),
        &surface,
        0.01,
    );
    assert!(out.normal_force_n > 1_900.0);
    assert!(out.is_sticking);
    assert!(out.force_world_n.x < 0.0);

    let water = SurfaceSample {
        medium: SurfaceMedium::Liquid,
        ..surface
    };
    let splash = step_contact_point(
        &spec,
        &mut state,
        DVec3::new(0.0, 0.0, -0.02),
        DVec3::ZERO,
        &water,
        0.01,
    );
    assert_eq!(splash.normal_force_n, 0.0);
}

#[test]
fn pinned_surface_rest_latches_and_breaks_free_on_ascent_ignition() {
    let spec = PinnedRestSpec::lander(15_000.0, 0.5);
    let at_rest = |speed: f64, upward_n: f64| PinnedRestInputs {
        any_contact: true,
        speed_m_s: speed,
        ang_rate_rad_s: speed,
        world_pos_m: DVec3::ZERO,
        world_rot: DQuat::IDENTITY,
        net_non_grav_upward_n: upward_n,
    };
    let mut rest = PinnedSurfaceRest::Free;
    for _ in 0..20 {
        step_pinned_surface_rest(&mut rest, &spec, &at_rest(0.001, 100.0), 0.05);
    }
    assert!(matches!(rest, PinnedSurfaceRest::Pinned { .. }));
    step_pinned_surface_rest(&mut rest, &spec, &at_rest(0.0, 16_000.0), 0.05);
    assert_eq!(rest, PinnedSurfaceRest::Free);
}

#[test]
fn point_riding_a_moving_surface_neither_slips_nor_damps() {
    let spec = ContactPointSpec {
        component: ComponentId(0),
        offset_local_m: DVec3::ZERO,
        stiffness_n_m: 100_000.0,
        damping_n_s_m: 2_000.0,
        mu_static: 0.9,
        mu_dynamic: 0.7,
        bristle_stiffness_n_m: 50_000.0,
        bristle_damping_n_s_m: 1_000.0,
        damping_ramp_depth_m: ContactPointSpec::DEFAULT_DAMPING_RAMP_DEPTH_M,
    };
    // A deck moving sideways and heaving at once.
    let deck_velocity = DVec3::new(30.0, -12.0, 0.1);
    let deck = |velocity_world_m_s| SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        velocity_world_m_s,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    };
    let point = PointKinematics {
        position_world_m: DVec3::new(0.0, 0.0, -0.02),
        velocity_world_m_s: deck_velocity,
        angular_vel_world_rad_s: DVec3::ZERO,
        rotation_world_from_link: DMat3::IDENTITY,
    };
    let state = ContactPointState::default();
    let riding = spec.evaluate(&state, &point, &deck(deck_velocity));
    // Relative to the deck the point is at rest: the spring alone acts.
    assert!((riding.force_world_n - DVec3::new(0.0, 0.0, 2_000.0)).length() < 1e-9);
    assert_eq!(riding.deflection_rate_world_m_s, DVec3::ZERO);
    assert!(riding.is_sticking);
    // The same point over ground at rest scrubs and is damped.
    let scrubbing = spec.evaluate(&state, &point, &deck(DVec3::ZERO));
    assert!(scrubbing.force_world_n.truncate().length() > 100.0);
    assert!(scrubbing.normal_force_n < riding.normal_force_n);
}
