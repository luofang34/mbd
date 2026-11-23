use mbd_provenance::{ProvenanceKind, SourceTable};

use super::{CalibrationObservation, CalibrationParamSpec, fit_parameters_lm};

#[test]
fn recovers_drag_polar_coefficients_with_fitted_provenance() {
    let true_cd0 = 0.0245;
    let true_k = 0.0480;
    let cls = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2];
    let observations: Vec<_> = cls
        .iter()
        .map(|&cl| CalibrationObservation {
            inputs: vec![cl],
            target: true_cd0 + true_k * cl * cl,
            weight: 1.0,
        })
        .collect();

    let params = [
        CalibrationParamSpec {
            name: "cd0".into(),
            unit: None,
            initial: 0.05,
            min: 0.0,
            max: 0.2,
            fd_step: 1.0e-6,
        },
        CalibrationParamSpec {
            name: "k_induced".into(),
            unit: None,
            initial: 0.10,
            min: 0.0,
            max: 0.3,
            fd_step: 1.0e-6,
        },
    ];

    let report = fit_parameters_lm(
        &params,
        &observations,
        |inputs, theta| {
            let cl = inputs.first().copied().unwrap_or(0.0);
            let cd0 = theta.first().copied().unwrap_or(0.0);
            let k = theta.get(1).copied().unwrap_or(0.0);
            Ok(cd0 + k * cl * cl)
        },
        40,
        1.0e-9,
        "wind_tunnel_polar_table_4",
        &SourceTable::default(),
    )
    .unwrap();

    assert!(report.converged);
    assert!(report.rmse < 1.0e-6);
    assert!((report.fitted_params[0].1.value - true_cd0).abs() < 1.0e-6);
    assert!((report.fitted_params[1].1.value - true_k).abs() < 1.0e-6);
    assert_eq!(report.fitted_params[0].1.kind(), ProvenanceKind::Fitted);
    assert!(!report.fitted_params[0].1.kind().is_evidence());
}
