use std::f64::consts::{FRAC_PI_2, PI};

use glam::{DMat3, DQuat, DVec3};

use super::*;
use crate::{ComponentId, ComponentSpec, ComponentState, ComponentTree, Placement};

const ENTRY_SPEED_M_S: f64 = 7_500.0;
const ENTRY_DENSITY_KG_M3: f64 = 1e-3;
const NOSE_RADIUS_M: f64 = 4.694;
const NORMAL_FORCE_SLOPE: f64 = 0.113;
const AXIAL_DRAG_COEFF: f64 = 1.3;
/// Trim mass offset, on the side the lift should point, from the shield apex.
const CG_OFFSET_M: DVec3 = DVec3::new(-1.15, 0.0, -0.112);

/// Apollo-class shield with the body origin at its apex.
fn apollo_params() -> BluntBodyCapsuleParams {
    BluntBodyCapsuleParams {
        base_radius_m: 1.956,
        nose_radius_m: NOSE_RADIUS_M,
        reference_point_body_m: DVec3::new(-NOSE_RADIUS_M, 0.0, 0.0),
        normal_force_slope: NORMAL_FORCE_SLOPE,
        axial_drag_coeff: AXIAL_DRAG_COEFF,
        pitch_damping_coeff: -1.0,
        sutton_graves_k: SUTTON_GRAVES_K_EARTH,
    }
}

fn apollo() -> BluntBodyCapsuleAero {
    BluntBodyCapsuleAero::new(apollo_params()).unwrap()
}

fn flow(velocity_air_body_m_s: DVec3, density_kg_m3: f64) -> AeroFlowState {
    AeroFlowState {
        velocity_air_body_m_s,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient: AmbientFlow {
            pressure_pa: 30.0,
            temperature_k: 230.0,
            density_kg_m3,
            sound_speed_m_s: 300.0,
        },
        control_surfaces_rad: [0.0; 8],
    }
}

/// Velocity at pitch-plane angle of attack `alpha_rad`.
fn at_alpha(alpha_rad: f64) -> DVec3 {
    DVec3::new(alpha_rad.cos(), 0.0, alpha_rad.sin()) * ENTRY_SPEED_M_S
}

fn pitching_moment(alpha_rad: f64, cg_body_m: DVec3) -> f64 {
    apollo()
        .evaluate(&flow(at_alpha(alpha_rad), ENTRY_DENSITY_KG_M3))
        .unwrap()
        .moment_about_cg(cg_body_m)
        .y
}

/// Bisects for the angle where the pitching moment about `cg_body_m` changes
/// sign, within a bracket chosen to exclude the apex-forward trim.
fn trim_alpha(cg_body_m: DVec3) -> f64 {
    let (mut low, mut high) = (-1.2, 1.2);
    assert!(pitching_moment(low, cg_body_m) > 0.0);
    assert!(pitching_moment(high, cg_body_m) < 0.0);
    for _ in 0..80 {
        let middle = (low + high) / 2.0;
        if pitching_moment(middle, cg_body_m) > 0.0 {
            low = middle;
        } else {
            high = middle;
        }
    }
    low
}

#[test]
fn vacuum_exerts_nothing() {
    let wrench = VacuumAero
        .evaluate(&flow(at_alpha(0.3), ENTRY_DENSITY_KG_M3))
        .unwrap();
    assert_eq!(wrench, AeroWrench::ZERO);
    assert_eq!(wrench.moment_about_cg(CG_OFFSET_M), DVec3::ZERO);
}

#[test]
fn moment_transfers_to_a_live_cg() {
    let wrench = AeroWrench {
        reference_point_body_m: DVec3::new(2.0, 0.0, 0.0),
        force_body_n: DVec3::new(0.0, 0.0, -10.0),
        moment_ref_body_nm: DVec3::new(1.0, 2.0, 3.0),
        stagnation_heat_flux_w_m2: 0.0,
    };
    assert_eq!(
        wrench.moment_about_cg(DVec3::ZERO),
        DVec3::new(1.0, 22.0, 3.0)
    );
    assert_eq!(
        wrench.moment_about_cg(DVec3::new(3.0, 0.0, 0.0)),
        DVec3::new(1.0, -8.0, 3.0)
    );
}

#[test]
fn an_on_axis_cg_trims_at_zero_incidence_and_flies_ballistic() {
    let on_axis = DVec3::new(CG_OFFSET_M.x, 0.0, 0.0);
    assert!(trim_alpha(on_axis).abs() < 1e-9);
    let wrench = apollo()
        .evaluate(&flow(at_alpha(0.0), ENTRY_DENSITY_KG_M3))
        .unwrap();
    assert_eq!(wrench.force_body_n.z, 0.0);
    assert!(wrench.force_body_n.x < 0.0);
}

#[test]
fn an_offset_cg_trims_at_the_analytic_incidence_with_a_restoring_moment() {
    let trim = trim_alpha(CG_OFFSET_M);
    let arm = apollo_params().reference_point_body_m - CG_OFFSET_M;
    let expected = (AXIAL_DRAG_COEFF / NORMAL_FORCE_SLOPE * arm.z / arm.x).atan();
    assert!((trim - expected).abs() < 1e-6, "{trim} vs {expected}");
    assert!(
        (-22f64.to_radians()..-18f64.to_radians()).contains(&trim),
        "{}°",
        trim.to_degrees()
    );

    let step = 0.05;
    assert!(
        pitching_moment(trim - step, CG_OFFSET_M) > 0.0,
        "nose rises"
    );
    assert!(
        pitching_moment(trim + step, CG_OFFSET_M) < 0.0,
        "nose falls"
    );
}

#[test]
fn trim_produces_lift_towards_the_cg_offset_at_apollo_lift_to_drag() {
    let trim = trim_alpha(CG_OFFSET_M);
    let velocity = at_alpha(trim);
    let wrench = apollo()
        .evaluate(&flow(velocity, ENTRY_DENSITY_KG_M3))
        .unwrap();
    let along_velocity = velocity.normalize();
    let drag = -wrench.force_body_n.dot(along_velocity);
    let lift = wrench.force_body_n + along_velocity * drag;
    assert!(drag > 0.0);
    assert!(lift.z < 0.0, "lift {lift} should point to the -z CG offset");
    let lift_to_drag = lift.length() / drag;
    assert!((0.25..0.4).contains(&lift_to_drag), "L/D is {lift_to_drag}");
}

/// Integrates pitch alone, with the flow held fixed in the air.
fn settle_from(mut alpha_rad: f64) -> f64 {
    let aero = apollo();
    let pitch_inertia = 6_000.0;
    let dt = 0.01;
    let mut rate = 0.0;
    for _ in 0..40_000 {
        let mut state = flow(at_alpha(alpha_rad), ENTRY_DENSITY_KG_M3);
        state.angular_rate_body_rad_s = DVec3::Y * rate;
        let moment = aero
            .evaluate(&state)
            .unwrap()
            .moment_about_cg(CG_OFFSET_M)
            .y;
        rate += moment / pitch_inertia * dt;
        alpha_rad += rate * dt;
    }
    alpha_rad
}

#[test]
fn a_released_capsule_pitches_to_trim_and_stays_there() {
    let trim = trim_alpha(CG_OFFSET_M);
    for start in [0.0, -0.6, 0.3] {
        let settled = settle_from(start);
        assert!((settled - trim).abs() < 1e-2, "{settled} vs {trim}");
    }
}

#[test]
fn pitch_damping_opposes_the_rate_and_leaves_roll_free() {
    let mut state = flow(at_alpha(0.0), ENTRY_DENSITY_KG_M3);
    state.angular_rate_body_rad_s = DVec3::new(0.7, 0.2, -0.1);
    let moment = apollo().evaluate(&state).unwrap().moment_ref_body_nm;
    assert_eq!(moment.x, 0.0);
    assert!(moment.y < 0.0);
    assert!(moment.z > 0.0);
    let ratio = moment.y / moment.z;
    assert!((ratio + 2.0).abs() < 1e-12);
}

/// Force and CG moment in air-fixed axes for a vehicle at `trim` incidence
/// banked by `bank_rad` about its velocity. Air-fixed `+x` is the velocity and
/// `-z` the unbanked lift direction.
fn banked_wrench(trim: f64, bank_rad: f64) -> (DVec3, DVec3) {
    let air_from_body = DQuat::from_rotation_x(bank_rad) * DQuat::from_rotation_y(trim);
    let velocity_body = air_from_body.conjugate() * DVec3::X * ENTRY_SPEED_M_S;
    let wrench = apollo()
        .evaluate(&flow(velocity_body, ENTRY_DENSITY_KG_M3))
        .unwrap();
    let moment_body = wrench.moment_about_cg(CG_OFFSET_M);
    (air_from_body * wrench.force_body_n, moment_body)
}

#[test]
fn banking_about_the_velocity_rotates_lift_and_keeps_drag_and_trim() {
    let trim = trim_alpha(CG_OFFSET_M);
    let (level_force, _) = banked_wrench(trim, 0.0);
    assert!(level_force.z < 0.0, "unbanked lift is up");
    assert!(level_force.y.abs() < 1e-6 * level_force.length());

    for bank_deg in [30.0_f64, 90.0, 135.0, 180.0, -60.0] {
        let bank = bank_deg.to_radians();
        let (force, moment) = banked_wrench(trim, bank);
        let scale = level_force.length();
        assert!((force.x - level_force.x).abs() < 1e-9 * scale, "drag");
        let expected = DQuat::from_rotation_x(bank) * level_force;
        assert!(
            (force - expected).length() < 1e-9 * scale,
            "bank {bank_deg}°: {force} vs {expected}"
        );
        assert!(moment.length() < 1e-4 * scale, "still trimmed: {moment}");
    }
}

#[test]
fn full_lift_down_and_lift_right_steer_the_trajectory() {
    let trim = trim_alpha(CG_OFFSET_M);
    let (up, _) = banked_wrench(trim, 0.0);
    let (right, _) = banked_wrench(trim, FRAC_PI_2);
    let (down, _) = banked_wrench(trim, PI);
    assert!(right.y > 0.0 && right.z.abs() < 1e-6 * right.length());
    assert!((down.z + up.z).abs() < 1e-9 * up.length());
}

#[test]
fn the_body_has_no_preferred_roll_angle() {
    let reference = apollo()
        .evaluate(&flow(at_alpha(0.35), ENTRY_DENSITY_KG_M3))
        .unwrap();
    for roll in [0.4, 1.5, 3.0, -2.0] {
        let spin = DQuat::from_rotation_x(roll);
        let rolled = apollo()
            .evaluate(&flow(spin * at_alpha(0.35), ENTRY_DENSITY_KG_M3))
            .unwrap();
        let expected = spin * reference.force_body_n;
        assert!((rolled.force_body_n - expected).length() < 1e-9 * expected.length());
    }
}

#[test]
fn stagnation_heating_follows_sutton_graves() {
    let aero = apollo();
    let heat = |speed: f64, density| {
        aero.evaluate(&flow(DVec3::X * speed, density))
            .unwrap()
            .stagnation_heat_flux_w_m2
    };
    let density = 1e-4;
    let speed = 11_000.0_f64;
    let expected = SUTTON_GRAVES_K_EARTH * (density / NOSE_RADIUS_M).sqrt() * speed.powi(3);
    assert!((heat(speed, density) - expected).abs() < 1e-6 * expected);
    assert!(
        (1e6..1.2e6).contains(&expected),
        "{expected} W/m² is not the expected order for lunar return"
    );
    assert!((heat(2.0 * speed, density) / heat(speed, density) - 8.0).abs() < 1e-9);
    assert!((heat(speed, 4.0 * density) / heat(speed, density) - 2.0).abs() < 1e-9);

    let blunter = BluntBodyCapsuleParams {
        nose_radius_m: 4.0 * NOSE_RADIUS_M,
        ..apollo_params()
    };
    let blunter = BluntBodyCapsuleAero::new(blunter)
        .unwrap()
        .evaluate(&flow(DVec3::X * speed, density))
        .unwrap();
    assert!((blunter.stagnation_heat_flux_w_m2 / expected - 0.5).abs() < 1e-9);
}

#[test]
fn forces_scale_with_dynamic_pressure_and_vanish_at_rest() {
    let aero = apollo();
    let at = |speed: f64, density| {
        aero.evaluate(&flow(at_alpha(0.3).normalize() * speed, density))
            .unwrap()
    };
    let base = at(1_000.0, 1e-3).force_body_n;
    assert!((at(2_000.0, 1e-3).force_body_n - base * 4.0).length() < 1e-6);
    assert!((at(1_000.0, 2e-3).force_body_n - base * 2.0).length() < 1e-6);

    let mut still = flow(DVec3::ZERO, 1.2);
    still.angular_rate_body_rad_s = DVec3::Y;
    let wrench = aero.evaluate(&still).unwrap();
    assert_eq!(wrench.force_body_n, DVec3::ZERO);
    assert_eq!(wrench.moment_ref_body_nm, DVec3::ZERO);
    assert_eq!(wrench.stagnation_heat_flux_w_m2, 0.0);
}

#[test]
fn an_apex_first_capsule_still_feels_drag() {
    let backwards = DVec3::new(-ENTRY_SPEED_M_S, 0.0, 300.0);
    let force = apollo()
        .evaluate(&flow(backwards, ENTRY_DENSITY_KG_M3))
        .unwrap()
        .force_body_n;
    assert!(force.dot(backwards) < 0.0);
}

#[test]
fn the_cg_from_the_component_tree_sets_the_trim() {
    let spec = |name: &str, placement, dry_mass_kg| ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg,
        dry_inertia_body: DMat3::ZERO,
        thermal_capacity_j_k: 0.0,
        nominal_power_w: 0.0,
    };
    let state = ComponentState::at_temperature(293.15);
    let shell = Placement {
        translation_m: DVec3::new(CG_OFFSET_M.x, 0.0, 0.0),
        ..Placement::ROOT
    };
    let ballast = Placement::child_of(ComponentId(0), DVec3::new(0.0, 0.0, -1.0));
    let mut tree = ComponentTree::new([
        (spec("shell", shell, 5_000.0), state),
        (
            spec("ballast", ballast, 0.0),
            ComponentState {
                payload_mass_kg: 600.0,
                ..state
            },
        ),
    ])
    .unwrap();

    let offset_cg = tree.mass_properties().unwrap().cg_body_m;
    assert!(offset_cg.z < -0.1);
    assert!(trim_alpha(offset_cg) < -0.3);

    tree.set_state(ComponentId(1), state).unwrap();
    let centred_cg = tree.mass_properties().unwrap().cg_body_m;
    assert!(trim_alpha(centred_cg).abs() < 1e-9);
}

#[test]
fn invalid_flow_is_rejected() {
    let aero = apollo();
    let negative_density = flow(at_alpha(0.0), -1.0);
    assert_eq!(
        aero.evaluate(&negative_density),
        Err(MbdError::InvalidFlow {
            field: "ambient.density_kg_m3",
            value: -1.0
        })
    );
    assert!(aero.evaluate(&flow(DVec3::NAN, 1.0)).is_err());
    let mut spinning = flow(DVec3::X, 1.0);
    spinning.angular_rate_body_rad_s = DVec3::INFINITY;
    assert!(aero.evaluate(&spinning).is_err());
}

#[test]
fn invalid_parameters_are_rejected() {
    let rejected = |params| {
        matches!(
            BluntBodyCapsuleAero::new(params),
            Err(MbdError::InvalidAeroParameter { .. })
        )
    };
    let base = apollo_params();
    assert!(rejected(BluntBodyCapsuleParams {
        base_radius_m: 0.0,
        ..base
    }));
    assert!(rejected(BluntBodyCapsuleParams {
        nose_radius_m: base.base_radius_m / 2.0,
        ..base
    }));
    assert!(rejected(BluntBodyCapsuleParams {
        axial_drag_coeff: -1.0,
        ..base
    }));
    assert!(rejected(BluntBodyCapsuleParams {
        normal_force_slope: f64::NAN,
        ..base
    }));
    assert!(rejected(BluntBodyCapsuleParams {
        pitch_damping_coeff: f64::INFINITY,
        ..base
    }));
    assert!(rejected(BluntBodyCapsuleParams {
        sutton_graves_k: -1.0,
        ..base
    }));
    assert!(rejected(BluntBodyCapsuleParams {
        reference_point_body_m: DVec3::NAN,
        ..base
    }));
    assert_eq!(*apollo().params(), base);
}
