use std::f64::consts::FRAC_PI_2;

use glam::{DQuat, DVec3};

use super::*;
use crate::{ComponentId, ComponentSpec, ComponentState, ComponentTree, Placement};

fn assert_vec_close(actual: DVec3, expected: DVec3) {
    assert!(
        (actual - expected).length() < 1e-9,
        "{actual} is not {expected}"
    );
}

const STILL: DVec3 = DVec3::ZERO;
const CG: DVec3 = DVec3::new(1.0, 0.0, 0.0);

#[test]
fn a_site_at_the_cg_senses_only_the_cg_specific_force() {
    let site = compute_site_kinematics(
        DVec3::new(0.0, 0.0, -9.8),
        DVec3::new(0.3, 0.1, 0.2),
        DVec3::new(1.0, 2.0, 3.0),
        (CG, DQuat::IDENTITY),
        CG,
    );
    assert_vec_close(site.specific_force_site_m_s2, DVec3::new(0.0, 0.0, -9.8));
    assert_eq!(site.position_m, DVec3::ZERO);
    assert_eq!(site.velocity_m_s, DVec3::ZERO);
}

#[test]
fn spin_gives_a_centripetal_force_towards_the_axis() {
    let omega = 2.0;
    let radius = 3.0;
    let site = (CG + DVec3::X * radius, DQuat::IDENTITY);
    let sensed = compute_site_kinematics(STILL, DVec3::Z * omega, STILL, site, CG);
    assert_vec_close(
        sensed.specific_force_site_m_s2,
        DVec3::X * (-omega * omega * radius),
    );
    assert_vec_close(sensed.velocity_m_s, DVec3::Y * (omega * radius));
}

#[test]
fn angular_acceleration_gives_a_tangential_force() {
    let alpha = 0.5;
    let radius = 3.0;
    let site = (CG + DVec3::X * radius, DQuat::IDENTITY);
    let sensed = compute_site_kinematics(STILL, STILL, DVec3::Z * alpha, site, CG);
    assert_vec_close(sensed.specific_force_site_m_s2, DVec3::Y * (alpha * radius));
}

#[test]
fn readings_come_out_in_site_axes() {
    let body_from_site = DQuat::from_rotation_z(FRAC_PI_2);
    let omega = 2.0;
    let radius = 3.0;
    let site = (CG + DVec3::X * radius, body_from_site);
    let sensed = compute_site_kinematics(STILL, DVec3::Z * omega, STILL, site, CG);
    // Site +x lies along body +y, so body -x reads as site +y.
    assert_vec_close(
        sensed.specific_force_site_m_s2,
        DVec3::Y * (omega * omega * radius),
    );
    assert_vec_close(sensed.angular_rate_site_rad_s, DVec3::Z * omega);

    let about_body_y = compute_site_kinematics(STILL, DVec3::Y, STILL, site, CG);
    assert_vec_close(about_body_y.angular_rate_site_rad_s, DVec3::X);
}

#[test]
fn draining_a_tank_changes_what_an_off_cg_imu_senses() {
    let spec = |name: &str, placement, dry_mass_kg| ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg,
        dry_inertia_body: glam::DMat3::ZERO,
        thermal_capacity_j_k: 0.0,
        nominal_power_w: 0.0,
    };
    let state = ComponentState::at_temperature(293.15);
    let root = ComponentId(0);
    let tank = ComponentId(1);
    let imu = ComponentId(2);
    let mut tree = ComponentTree::new([
        (spec("core", Placement::ROOT, 100.0), state),
        (
            spec("tank", Placement::child_of(root, DVec3::X * 4.0), 0.0),
            ComponentState {
                fluid_mass_kg: 100.0,
                ..state
            },
        ),
        (
            spec("imu", Placement::child_of(root, DVec3::ZERO), 0.0),
            state,
        ),
    ])
    .unwrap();

    let omega = DVec3::Z;
    let sense = |tree: &ComponentTree| {
        let cg = tree.mass_properties().unwrap().cg_body_m;
        let site = tree.world_transform_in_root(imu).unwrap();
        compute_site_kinematics(STILL, omega, STILL, site, cg).specific_force_site_m_s2
    };
    assert_vec_close(sense(&tree), DVec3::X * 2.0);
    tree.set_state(tank, state).unwrap();
    assert_vec_close(sense(&tree), DVec3::ZERO);
}

#[test]
fn a_body_site_lifts_into_ecef_around_the_cg() {
    let body_from_site = DQuat::from_rotation_z(FRAC_PI_2);
    let site = (CG + DVec3::X, body_from_site);
    let body = compute_site_kinematics(DVec3::X, DVec3::Z, STILL, site, CG);
    let ecef_from_body = DQuat::from_rotation_x(FRAC_PI_2);
    let cg_ecef = DVec3::new(6.4e6, 0.0, 0.0);
    let v_cg_ecef = DVec3::new(0.0, 100.0, 0.0);

    let ecef = body.in_ecef(cg_ecef, v_cg_ecef, ecef_from_body);

    assert_eq!(ecef.frame, KinematicsFrame::Ecef);
    assert_vec_close(ecef.position_m, cg_ecef + DVec3::X);
    assert_vec_close(ecef.velocity_m_s, v_cg_ecef + DVec3::Z);
    assert_vec_close(ecef.attitude * DVec3::X, DVec3::Z);
    assert_eq!(ecef.specific_force_site_m_s2, body.specific_force_site_m_s2);
    assert_eq!(ecef.in_ecef(cg_ecef, v_cg_ecef, ecef_from_body), ecef);
}

#[test]
fn ideal_environment_is_benign() {
    let site = compute_site_kinematics(STILL, STILL, STILL, (CG, DQuat::IDENTITY), CG);
    let environment = SiteEnvironment::ideal(site);
    assert_eq!(environment.kinematics, site);
    assert_eq!(environment.temperature_k, 293.15);
    assert_eq!(environment.power_rail, PowerRailState::nominal_28v());
    assert_eq!(environment.vibration_rms_m_s2, DVec3::ZERO);
    assert_eq!(environment.emi_field_t, DVec3::ZERO);
}

#[test]
fn nominal_rails_are_clean_and_enabled() {
    for (rail, volts) in [
        (PowerRailState::nominal_28v(), 28.0),
        (PowerRailState::nominal_5v(), 5.0),
        (PowerRailState::nominal_3v3(), 3.3),
    ] {
        assert_eq!(rail.voltage_v, volts);
        assert_eq!(rail.ripple_v_rms, 0.0);
        assert!(rail.enabled);
    }
}
