use super::{StageDesignSpec, analyze_multistage_stack};

#[test]
fn three_stage_saturn_class_stack_matches_tsiolkovsky_leo_tli_budget() {
    let stages = [
        StageDesignSpec {
            name: "S-IC".into(),
            dry_mass_kg: 130_000.0,
            propellant_mass_kg: 2_150_000.0,
            isp_vac_s: 304.0,
            isp_sl_s: 263.0,
            thrust_vac_n: 38_000_000.0,
            gravity_loss_margin_m_s: 1_150.0,
            drag_loss_margin_m_s: 120.0,
        },
        StageDesignSpec {
            name: "S-II".into(),
            dry_mass_kg: 36_000.0,
            propellant_mass_kg: 444_000.0,
            isp_vac_s: 421.0,
            isp_sl_s: 421.0,
            thrust_vac_n: 5_100_000.0,
            gravity_loss_margin_m_s: 250.0,
            drag_loss_margin_m_s: 0.0,
        },
        StageDesignSpec {
            name: "S-IVB".into(),
            dry_mass_kg: 13_500.0,
            propellant_mass_kg: 109_000.0,
            isp_vac_s: 421.0,
            isp_sl_s: 421.0,
            thrust_vac_n: 1_033_000.0,
            gravity_loss_margin_m_s: 60.0,
            drag_loss_margin_m_s: 0.0,
        },
    ];
    let payload_kg = 48_600.0;
    let required_net_dv_m_s = 10_800.0;
    let report =
        analyze_multistage_stack(&stages, payload_kg, 9.806_65, required_net_dv_m_s).unwrap();

    let expected_ideal = 283.5 * 9.806_65 * (2_931_100.0_f64 / 781_100.0).ln()
        + 421.0 * 9.806_65 * (651_100.0_f64 / 207_100.0).ln()
        + 421.0 * 9.806_65 * (171_100.0_f64 / 62_100.0).ln();

    assert_eq!(report.stages.len(), 3);
    assert!((report.total_liftoff_mass_kg - 2_931_100.0).abs() < 1e-6);
    assert!((report.total_ideal_delta_v_m_s - expected_ideal).abs() < 1e-6);
    assert!(report.stages[0].initial_twr > 1.25);
    assert!(report.total_net_delta_v_m_s >= required_net_dv_m_s);
    assert!(report.meets_requirement);
    assert!((report.payload_mass_fraction - payload_kg / 2_931_100.0).abs() < 1e-12);
}
