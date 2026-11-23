use crate::error::{SystemsError, check_finite, check_non_negative};

/// Closed-loop center-of-gravity fuel trim controller (e.g. Concorde supersonic Mach CG trim
/// or spacecraft fluid balance).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CgTrimController {
    /// Target longitudinal or axis coordinate of vehicle CG in root frame, m.
    pub target_cg_coord_m: f64,
    /// Neutral deadband half-width before pumping starts, m.
    pub deadband_m: f64,
    /// Proportional flow gain mapping CG error (m) to signed mass flow (kg/s).
    pub gain_kg_s_per_m: f64,
    /// Maximum pump command magnitude, kg/s.
    pub max_rate_kg_s: f64,
}

impl CgTrimController {
    /// Computes signed trim transfer command (`> 0` moves mass toward `+coord` tank).
    pub fn compute_command(&self, current_cg_coord_m: f64) -> Result<f64, SystemsError> {
        check_finite("cg_trim.current_cg_coord_m", current_cg_coord_m)?;
        check_finite("cg_trim.target_cg_coord_m", self.target_cg_coord_m)?;
        check_non_negative("cg_trim.deadband_m", self.deadband_m)?;
        check_non_negative("cg_trim.gain_kg_s_per_m", self.gain_kg_s_per_m)?;
        check_non_negative("cg_trim.max_rate_kg_s", self.max_rate_kg_s)?;

        let error = self.target_cg_coord_m - current_cg_coord_m;
        if error.abs() <= self.deadband_m {
            return Ok(0.0);
        }
        let excess = if error > 0.0 {
            error - self.deadband_m
        } else {
            error + self.deadband_m
        };
        Ok((excess * self.gain_kg_s_per_m).clamp(-self.max_rate_kg_s, self.max_rate_kg_s))
    }
}
