use std::f64::consts::{FRAC_PI_2, PI};

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{
    AmbientGas, ComponentId, ComponentSpec, ComponentState, ComponentTree, GasComposition,
    JointSpec, Placement, SUTTON_GRAVES_K_EARTH, SUTTON_GRAVES_K_MARS,
};

use super::*;

const ENTRY_SPEED_M_S: f64 = 7_500.0;
const ENTRY_DENSITY_KG_M3: f64 = 1e-3;
const NOSE_RADIUS_M: f64 = 4.694;
const NORMAL_FORCE_SLOPE: f64 = 0.113;
const AXIAL_DRAG_COEFF: f64 = 1.3;
const CG_OFFSET_M: DVec3 = DVec3::new(-1.15, 0.0, -0.112);

fn apollo() -> BluntCapsuleAero {
    BluntCapsuleAero::hypersonic_newtonian(
        1.956,
        NOSE_RADIUS_M,
        DVec3::new(-NOSE_RADIUS_M, 0.0, 0.0),
        NORMAL_FORCE_SLOPE,
        AXIAL_DRAG_COEFF,
        -1.0,
        SUTTON_GRAVES_K_EARTH,
    )
    .unwrap()
}

fn flow(velocity_air_body_m_s: DVec3, density_kg_m3: f64) -> AeroFlowState {
    let mut ambient = AmbientGas::earth_air(30.0, 230.0).unwrap();
    ambient.density_kg_m3 = density_kg_m3;
    AeroFlowState {
        velocity_air_body_m_s,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient,
        control_surfaces_rad: [0.0; 8],
    }
}

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
fn on_axis_and_offset_cg_trim_match_analytic_incidence() {
    let on_axis = DVec3::new(CG_OFFSET_M.x, 0.0, 0.0);
    assert!(trim_alpha(on_axis).abs() < 1e-9);
    let trim = trim_alpha(CG_OFFSET_M);
    let arm = apollo().params().reference_point_body_m - CG_OFFSET_M;
    let expected = (AXIAL_DRAG_COEFF / NORMAL_FORCE_SLOPE * arm.z / arm.x).atan();
    assert!((trim - expected).abs() < 1e-6);
    assert!(pitching_moment(trim - 0.05, CG_OFFSET_M) > 0.0);
    assert!(pitching_moment(trim + 0.05, CG_OFFSET_M) < 0.0);
}

#[test]
fn trim_produces_apollo_lift_to_drag_and_dynamic_stability() {
    let trim = trim_alpha(CG_OFFSET_M);
    let velocity = at_alpha(trim);
    let wrench = apollo()
        .evaluate(&flow(velocity, ENTRY_DENSITY_KG_M3))
        .unwrap();
    let along = velocity.normalize();
    let drag = -wrench.force_body_n.dot(along);
    let lift = wrench.force_body_n + along * drag;
    let ld = lift.length() / drag;
    assert!((0.25..0.4).contains(&ld));
    assert!(lift.z < 0.0);

    let aero = apollo();
    let mut rate = 0.0;
    let mut alpha = -0.5;
    for _ in 0..30_000 {
        let mut st = flow(at_alpha(alpha), ENTRY_DENSITY_KG_M3);
        st.angular_rate_body_rad_s = DVec3::Y * rate;
        let m = aero.evaluate(&st).unwrap().moment_about_cg(CG_OFFSET_M).y;
        rate += m / 6_000.0 * 0.01;
        alpha += rate * 0.01;
    }
    assert!((alpha - trim).abs() < 1e-2);
}

#[test]
fn banking_and_axisymmetry_hold() {
    let trim = trim_alpha(CG_OFFSET_M);
    let banked = |bank: f64| {
        let q = DQuat::from_rotation_x(bank) * DQuat::from_rotation_y(trim);
        let vb = q.conjugate() * DVec3::X * ENTRY_SPEED_M_S;
        let w = apollo().evaluate(&flow(vb, ENTRY_DENSITY_KG_M3)).unwrap();
        (q * w.force_body_n, w.moment_about_cg(CG_OFFSET_M))
    };
    let (up, _) = banked(0.0);
    let (right, _) = banked(FRAC_PI_2);
    let (down, _) = banked(PI);
    assert!(up.z < 0.0 && right.y > 0.0);
    assert!((down.z + up.z).abs() < 1e-9 * up.length());

    let reference = apollo()
        .evaluate(&flow(at_alpha(0.35), ENTRY_DENSITY_KG_M3))
        .unwrap();
    let spin = DQuat::from_rotation_x(1.5);
    let rolled = apollo()
        .evaluate(&flow(spin * at_alpha(0.35), ENTRY_DENSITY_KG_M3))
        .unwrap();
    assert!((rolled.force_body_n - spin * reference.force_body_n).length() < 1e-9 * 1e6);
}

#[test]
fn sutton_graves_adapts_from_gas_composition_when_unspecified() {
    let mars_shell = BluntCapsuleAero::new(BluntCapsuleParams {
        base_radius_m: 2.25,
        nose_radius_m: 1.125,
        reference_point_body_m: DVec3::new(-1.125, 0.0, 0.0),
        normal_force_slope: 0.12,
        axial_drag_coeff: 1.4,
        pitch_damping_coeff: -0.8,
        sutton_graves_k: None,
        subsonic_drag_coeff: Some(0.95),
        transonic_drag_rise: 0.2,
        free_molecular_drag_coeff: Some(2.1),
    })
    .unwrap();

    let mars_gas =
        AmbientGas::from_state_and_composition(50.0, 180.0, GasComposition::mars_air()).unwrap();
    let earth_gas =
        AmbientGas::from_state_and_composition(50.0, 220.0, GasComposition::earth_dry_air())
            .unwrap();

    let flux_mars = mars_shell
        .evaluate(&AeroFlowState {
            velocity_air_body_m_s: DVec3::X * 5_500.0,
            angular_rate_body_rad_s: DVec3::ZERO,
            ambient: mars_gas,
            control_surfaces_rad: [0.0; 8],
        })
        .unwrap()
        .stagnation_heat_flux_w_m2;
    let expected_mars =
        SUTTON_GRAVES_K_MARS * (mars_gas.density_kg_m3 / 1.125).sqrt() * 5_500.0_f64.powi(3);
    assert!((flux_mars - expected_mars).abs() < 0.01 * expected_mars);

    // Rarefied high-Knudsen entry produces higher free-molecular drag coefficient than subsonic.
    let rarefied_gas = AmbientGas::earth_air(1e-4, 400.0).unwrap();
    assert!(rarefied_gas.knudsen_number(4.5) > 1.0);
    let w_rare = mars_shell
        .evaluate(&AeroFlowState {
            velocity_air_body_m_s: DVec3::X * 7_000.0,
            angular_rate_body_rad_s: DVec3::ZERO,
            ambient: rarefied_gas,
            control_surfaces_rad: [0.0; 8],
        })
        .unwrap();
    let q_rare = 0.5 * rarefied_gas.density_kg_m3 * 7_000.0_f64.powi(2) * PI * 2.25_f64.powi(2);
    let cd_eff_rare = -w_rare.force_body_n.x / q_rare;
    assert!(cd_eff_rare > 1.7 && cd_eff_rare <= 2.15);

    // Subsonic regime uses subsonic_drag_coeff.
    let w_sub = mars_shell
        .evaluate(&AeroFlowState {
            velocity_air_body_m_s: DVec3::X * 60.0,
            angular_rate_body_rad_s: DVec3::ZERO,
            ambient: AmbientGas::earth_air(101_325.0, 288.15).unwrap(),
            control_surfaces_rad: [0.0; 8],
        })
        .unwrap();
    let rho_sl = AmbientGas::earth_air(101_325.0, 288.15)
        .unwrap()
        .density_kg_m3;
    let q_sub = 0.5 * rho_sl * 60.0_f64.powi(2) * PI * 2.25_f64.powi(2);
    let cd_eff_sub = -w_sub.force_body_n.x / q_sub;
    assert!((cd_eff_sub - 0.95).abs() < 1e-6);
    let _ = earth_gas;
}

#[test]
fn component_tree_cg_and_invalid_params_behave_cleanly() {
    let spec = |name: &str, placement, dry_mass_kg| ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg,
        dry_inertia_body: DMat3::ZERO,
        thermal_capacity_j_k: 0.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let state = ComponentState::at_temperature(293.15);
    let mut tree = ComponentTree::new([
        (
            spec(
                "shell",
                Placement {
                    translation_m: DVec3::new(CG_OFFSET_M.x, 0.0, 0.0),
                    ..Placement::ROOT
                },
                5_000.0,
            ),
            state,
        ),
        (
            spec(
                "ballast",
                Placement::child_of(ComponentId(0), DVec3::new(0.0, 0.0, -1.0)),
                0.0,
            ),
            ComponentState {
                payload_mass_kg: 600.0,
                ..state
            },
        ),
    ])
    .unwrap();
    let offset_cg = tree.mass_properties().unwrap().cg_body_m;
    assert!(trim_alpha(offset_cg) < -0.3);
    tree.set_state(ComponentId(1), state).unwrap();
    assert!(trim_alpha(tree.mass_properties().unwrap().cg_body_m).abs() < 1e-9);

    assert!(
        BluntCapsuleAero::new(BluntCapsuleParams {
            base_radius_m: 0.0,
            ..*apollo().params()
        })
        .is_err()
    );
    assert!(
        BluntCapsuleAero::new(BluntCapsuleParams {
            nose_radius_m: 0.0,
            ..*apollo().params()
        })
        .is_err()
    );
    assert!(
        BluntCapsuleAero::hypersonic_newtonian(
            1.0,
            0.5,
            DVec3::ZERO,
            0.1,
            1.3,
            -1.0,
            SUTTON_GRAVES_K_EARTH,
        )
        .is_err()
    );
}
