//! Every ported element against values recorded from the root-lumped
//! implementation it replaces, element by element and on a whole vehicle.

use glam::DVec3;
use mbd_core::AmbientGas;

use super::*;
use crate::{
    BluntCapsuleParams, BodyAndFinParams, HumanDummyParams, ParachuteParams, RarefiedPanelParams,
    RotorBemParams, WingSurfaceParams,
};

mod heavy;
mod pins;

/// Control channels: body-fin pitch, yaw, roll; wing sweep, flap, aileron;
/// rotor speed, collective, lateral and longitudinal cyclic; reefing.
const CONTROLS: [f64; 11] = [
    0.05, -0.04, 0.03, 0.2, 0.15, -0.1, 265.7, 0.01, 0.02, -0.015, 0.8,
];

/// Parachute continuous state and mode recorded with the old fixture:
/// deployed, 60 % inflated.
const CHUTE_STATE: [f64; 1] = [0.6];
const CHUTE_MODES: [f64; 1] = [1.0];

fn sea() -> AmbientGas {
    AmbientGas::earth_air(101_325.0, 288.15).unwrap()
}

fn rate() -> DVec3 {
    DVec3::new(0.1, -0.2, 0.15)
}

fn capsule() -> AeroElementKind {
    AeroElementKind::BluntCapsule(
        BluntCapsuleAero::new(BluntCapsuleParams {
            base_radius_m: 1.956,
            nose_radius_m: 4.694,
            reference_point_body_m: DVec3::new(-4.694, 0.0, 0.0),
            normal_force_slope: 0.113,
            axial_drag_coeff: 1.3,
            pitch_damping_coeff: -1.0,
            sutton_graves_k: None,
            subsonic_drag_coeff: Some(1.05),
            transonic_drag_rise: 0.2,
            free_molecular_drag_coeff: Some(2.1),
        })
        .unwrap(),
    )
}

fn body_fin() -> AeroElementKind {
    AeroElementKind::BodyAndFin(
        BodyAndFinAero::new(BodyAndFinParams {
            ref_area_m2: 10.5,
            ref_length_m: 3.66,
            reference_point_body_m: DVec3::new(-20.0, 0.0, 0.0),
            nose_radius_m: 0.5,
            cd0_subsonic: 0.3,
            cd0_supersonic: 0.25,
            transonic_drag_peak: 0.4,
            cn_alpha_body: 2.5,
            pitch_damping_coeff: -8.0,
            roll_damping_coeff: -0.5,
            fin_area_m2: 4.0,
            fin_arm_m: 15.0,
            fin_cl_alpha: 3.0,
            fin_stall_rad: 0.35,
            pitch_control_channel: Some(0),
            yaw_control_channel: Some(1),
            roll_control_channel: Some(2),
        })
        .unwrap(),
    )
}

fn wing() -> AeroElementKind {
    AeroElementKind::WingSurface(
        WingSurfaceAero::new(WingSurfaceParams {
            area_m2: 232.0,
            mean_chord_m: 6.0,
            span_m: 35.6,
            aspect_ratio: 5.5,
            sweep_quarter_chord_rad: 0.35,
            sweep_control_channel: Some(3),
            cl_alpha_incompressible: 4.6,
            cl0: 0.1,
            cl_max: 1.4,
            alpha_stall_rad: 0.28,
            cd0: 0.018,
            oswald_efficiency: 0.8,
            wave_drag_factor: 0.9,
            critical_mach: 0.78,
            cm0: -0.03,
            ac_position_body_m: DVec3::new(1.2, 3.0, -0.4),
            chord_direction_body: DVec3::X,
            lift_normal_body: -DVec3::Z,
            flap_channel: Some(4),
            flap_dcl: 0.9,
            flap_dcd: 0.05,
            aileron_channel: Some(5),
            aileron_dcl: 0.3,
        })
        .unwrap(),
    )
}

fn rotor() -> AeroElementKind {
    AeroElementKind::RotorBem(
        RotorBemAero::new(RotorBemParams {
            radius_m: 0.605,
            hub_radius_m: 0.08,
            blade_count: 2,
            chord_m: 0.075,
            root_pitch_rad: 19f64.to_radians(),
            tip_pitch_rad: 9f64.to_radians(),
            cl_alpha: 5.5,
            cl_max: 1.25,
            cd0_high_re: 0.018,
            low_re_drag_factor: 4.5,
            critical_tip_mach: 0.72,
            coaxial_interference_factor: 1.15,
            rotation_direction: -1.0,
            hub_position_body_m: DVec3::new(0.1, -0.05, -0.3),
            shaft_axis_body: -DVec3::Z,
            radial_stations: 10,
            omega_channel: 6,
            collective_channel: Some(7),
            cyclic_lat_channel: Some(8),
            cyclic_lon_channel: Some(9),
        })
        .unwrap(),
    )
}

fn panel() -> AeroElementKind {
    AeroElementKind::RarefiedPanel(
        RarefiedPanelAero::new(RarefiedPanelParams {
            area_m2: 6.0,
            normal_body: DVec3::new(0.6, 0.0, 0.8),
            center_body_m: DVec3::new(0.0, 2.5, 0.1),
            double_sided: true,
            surface_temperature_k: 300.0,
            energy_accommodation: 0.9,
            tangential_accommodation: 0.95,
        })
        .unwrap(),
    )
}

fn chute() -> AeroElementKind {
    AeroElementKind::ParachuteCanopy(
        ParachuteCanopyAero::new(ParachuteParams {
            nominal_area_m2: 363.0,
            cd_subsonic: 0.65,
            cd_supersonic: 0.52,
            cn_alpha: 0.4,
            riser_attach_body_m: DVec3::new(-1.5, 0.0, 0.2),
            canopy_axis_body: -DVec3::X,
            inflation_time_constant_s: 0.8,
            wake_velocity_ratio: 0.85,
            rotational_damping_coeff: 0.5,
            reference_dynamic_pressure_pa: 200.0,
            deploy_channel: None,
            reefing_channel: Some(10),
            deployed_at_start: true,
            inflation_at_start: CHUTE_STATE[0],
        })
        .unwrap(),
    )
}

fn dummy() -> AeroElementKind {
    AeroElementKind::HumanDummy(
        HumanDummyAero::new(HumanDummyParams {
            frontal_area_m2: 0.7,
            reference_length_m: 1.8,
            drag_coeff: 1.1,
            damping_coeff: -0.4,
            reference_point_body_m: DVec3::new(0.0, 0.1, -0.2),
        })
        .unwrap(),
    )
}

/// A recorded wrench: reference point, force, moment, heat flux.
type Pin = ([f64; 3], [f64; 3], [f64; 3], f64);

/// Asserts `got` equals the recorded `pin` to `rel` of each vector's
/// magnitude.
fn assert_pin(label: &str, got: &AeroWrench, pin: &Pin, rel: f64) {
    let close = |a: DVec3, b: [f64; 3]| {
        let b = DVec3::from_array(b);
        (a - b).length() <= rel * b.length()
    };
    assert!(
        close(got.reference_point_link_m, pin.0),
        "{label} ref {got:?}"
    );
    assert!(close(got.force_link_n, pin.1), "{label} force {got:?}");
    assert!(
        close(got.moment_ref_link_nm, pin.2),
        "{label} moment {got:?}"
    );
    assert!(
        (got.stagnation_heat_flux_w_m2 - pin.3).abs() <= rel * pin.3.abs(),
        "{label} heat {got:?}"
    );
}
