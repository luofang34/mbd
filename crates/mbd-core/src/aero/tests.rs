use glam::DVec3;

use super::*;
use crate::GasComposition;

fn flow(velocity_air_body_m_s: DVec3, pressure_pa: f64) -> AeroFlowState {
    AeroFlowState {
        velocity_air_body_m_s,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient: AmbientGas::from_state_and_composition(
            pressure_pa,
            230.0,
            GasComposition::earth_dry_air(),
        )
        .unwrap(),
        control_surfaces_rad: [0.0; 8],
    }
}

#[test]
fn vacuum_exerts_nothing() {
    let wrench = VacuumAero
        .evaluate(&flow(DVec3::X * 7_500.0, 30.0))
        .unwrap();
    assert_eq!(wrench, AeroWrench::ZERO);
    assert_eq!(
        wrench.moment_about_cg(DVec3::new(-1.15, 0.0, -0.112)),
        DVec3::ZERO
    );
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
fn invalid_flow_is_rejected() {
    let mut bad = flow(DVec3::X, 100.0);
    bad.ambient.density_kg_m3 = -1.0;
    assert_eq!(
        bad.validate(),
        Err(MbdError::InvalidFlow {
            field: "ambient.density_kg_m3",
            value: -1.0
        })
    );
    assert!(flow(DVec3::NAN, 100.0).validate().is_err());

    let mut bad_rate = flow(DVec3::X, 100.0);
    bad_rate.angular_rate_body_rad_s = DVec3::INFINITY;
    assert!(bad_rate.validate().is_err());

    let mut bad_cs = flow(DVec3::X, 100.0);
    bad_cs.control_surfaces_rad[0] = f64::NAN;
    assert!(bad_cs.validate().is_err());
}
