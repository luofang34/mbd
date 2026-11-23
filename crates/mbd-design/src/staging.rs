use serde::{Deserialize, Serialize};

use crate::error::{DesignError, check_non_negative, check_positive};

#[cfg(test)]
mod tests;

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Sizing specification of one stage inside a serial multi-stage launch vehicle stack.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StageDesignSpec {
    /// Human-readable stage label (e.g. `"S-IC"`, `"S-II"`, `"S-IVB"`).
    pub name: String,
    /// Stage structural + engine + avionics inert dry mass, kg.
    pub dry_mass_kg: f64,
    /// Usable propellant mass carried by this stage, kg.
    pub propellant_mass_kg: f64,
    /// Effective vacuum specific impulse across this stage's burn, s.
    pub isp_vac_s: f64,
    /// Sea-level specific impulse (reported on stages operating from sea level), s.
    pub isp_sl_s: f64,
    /// Stage vacuum thrust at ignition, N.
    pub thrust_vac_n: f64,
    /// Estimated gravity + steering + cosine loss allowance subtracted from ideal $\Delta v$, m/s.
    pub gravity_loss_margin_m_s: f64,
    /// Estimated aerodynamic drag loss subtracted from ideal $\Delta v$, m/s.
    pub drag_loss_margin_m_s: f64,
}

impl StageDesignSpec {
    fn validate(&self) -> Result<(), DesignError> {
        if self.name.trim().is_empty() {
            return Err(DesignError::InvalidConfig {
                detail: "stage name must not be empty".into(),
            });
        }
        check_positive("stage.dry_mass_kg", self.dry_mass_kg)?;
        check_positive("stage.propellant_mass_kg", self.propellant_mass_kg)?;
        check_positive("stage.isp_vac_s", self.isp_vac_s)?;
        check_positive("stage.isp_sl_s", self.isp_sl_s)?;
        check_positive("stage.thrust_vac_n", self.thrust_vac_n)?;
        check_non_negative(
            "stage.gravity_loss_margin_m_s",
            self.gravity_loss_margin_m_s,
        )?;
        check_non_negative("stage.drag_loss_margin_m_s", self.drag_loss_margin_m_s)?;
        Ok(())
    }
}

/// Analytical mass and performance summary for a single stage in the stack.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StagePerformance {
    /// Stage label.
    pub name: String,
    /// Total wet stack mass above + including this stage at ignition, kg.
    pub initial_mass_kg: f64,
    /// Total stack mass at main engine cutoff before staging, kg.
    pub burnout_mass_kg: f64,
    /// Stage structural mass fraction $m_{\text{dry}} / (m_{\text{dry}} + m_{\text{prop}})$.
    pub structural_ratio: f64,
    /// Ideal Tsiolkovsky $\Delta v = I_{\text{sp}} g_0 \ln(m_0 / m_f)$, m/s.
    pub ideal_delta_v_m_s: f64,
    /// Net $\Delta v$ after gravity and drag loss margins, m/s.
    pub net_delta_v_m_s: f64,
    /// Stage thrust-to-weight ratio at ignition relative to reference gravity.
    pub initial_twr: f64,
    /// Stage thrust-to-weight ratio at burnout relative to reference gravity.
    pub burnout_twr: f64,
}

/// Complete bottom-to-top Tsiolkovsky mission $\Delta v$ and mass fraction audit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MultistageBudgetReport {
    /// Stage breakdown ordered from first-to-ignite (lowest) to upper stage (last).
    pub stages: Vec<StagePerformance>,
    /// Total stack liftoff mass including payload, kg.
    pub total_liftoff_mass_kg: f64,
    /// Payload mass divided by total liftoff mass.
    pub payload_mass_fraction: f64,
    /// Sum of ideal stage $\Delta v$ values, m/s.
    pub total_ideal_delta_v_m_s: f64,
    /// Sum of net stage $\Delta v$ values after loss allowances, m/s.
    pub total_net_delta_v_m_s: f64,
    /// Excess (`>= 0`) or shortfall (`< 0`) relative to `required_delta_v_m_s`, m/s.
    pub margin_m_s: f64,
    /// Whether `total_net_delta_v_m_s >= required_delta_v_m_s`.
    pub meets_requirement: bool,
}

/// Evaluates serial staging performance bottom-up (`stages[0]` ignites first) carrying `payload_mass_kg`.
pub fn analyze_multistage_stack(
    stages: &[StageDesignSpec],
    payload_mass_kg: f64,
    surface_gravity_m_s2: f64,
    required_delta_v_m_s: f64,
) -> Result<MultistageBudgetReport, DesignError> {
    if stages.is_empty() {
        return Err(DesignError::InvalidConfig {
            detail: "multistage stack must contain at least one stage".into(),
        });
    }
    check_non_negative("staging.payload_mass_kg", payload_mass_kg)?;
    check_positive("staging.surface_gravity_m_s2", surface_gravity_m_s2)?;
    check_non_negative("staging.required_delta_v_m_s", required_delta_v_m_s)?;
    for stage in stages {
        stage.validate()?;
    }

    let mut upper_mass_kg = payload_mass_kg;
    let mut reversed = Vec::with_capacity(stages.len());
    for stage in stages.iter().rev() {
        let m_wet_stage = stage.dry_mass_kg + stage.propellant_mass_kg;
        let burnout_mass_kg = upper_mass_kg + stage.dry_mass_kg;
        let initial_mass_kg = upper_mass_kg + m_wet_stage;
        let isp_eff = 0.5 * (stage.isp_vac_s + stage.isp_sl_s);
        let ideal_delta_v_m_s =
            isp_eff * STANDARD_GRAVITY_M_S2 * (initial_mass_kg / burnout_mass_kg).ln();
        let net_delta_v_m_s =
            ideal_delta_v_m_s - stage.gravity_loss_margin_m_s - stage.drag_loss_margin_m_s;
        reversed.push(StagePerformance {
            name: stage.name.clone(),
            initial_mass_kg,
            burnout_mass_kg,
            structural_ratio: stage.dry_mass_kg / m_wet_stage,
            ideal_delta_v_m_s,
            net_delta_v_m_s,
            initial_twr: stage.thrust_vac_n / (initial_mass_kg * surface_gravity_m_s2),
            burnout_twr: stage.thrust_vac_n / (burnout_mass_kg * surface_gravity_m_s2),
        });
        upper_mass_kg = initial_mass_kg;
    }

    reversed.reverse();
    let total_liftoff_mass_kg = upper_mass_kg;
    let total_ideal_delta_v_m_s: f64 = reversed.iter().map(|s| s.ideal_delta_v_m_s).sum();
    let total_net_delta_v_m_s: f64 = reversed.iter().map(|s| s.net_delta_v_m_s).sum();
    let margin_m_s = total_net_delta_v_m_s - required_delta_v_m_s;

    Ok(MultistageBudgetReport {
        stages: reversed,
        total_liftoff_mass_kg,
        payload_mass_fraction: payload_mass_kg / total_liftoff_mass_kg,
        total_ideal_delta_v_m_s,
        total_net_delta_v_m_s,
        margin_m_s,
        meets_requirement: margin_m_s >= 0.0,
    })
}
