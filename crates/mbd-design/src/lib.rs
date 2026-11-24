//! Forward and reverse Model-Based Design (MBD) verification and calibration suite.
//!
//! Operates directly over the single-source-of-truth crates (`mbd-core`, `mbd-systems`,
//! `mbd-provenance`, and `syspack`):
//! - [`staging`]: Multistage launch vehicle bottom-to-top Tsiolkovsky $\Delta v$ & TWR budgets.
//! - [`trim`]: Multi-tank CG excursion & longitudinal static-margin sweep.
//! - [`power_thermal`]: Orbit sunlit/eclipse electrical balance and multi-node thermal limits.
//! - [`calibrate`]: Reverse-MBD bounded Levenberg-Marquardt parameter identification with [`mbd_provenance::Basis::Fitted`].
//! - [`contract`]: End-to-end `VerifiedSyspack` physical consistency & provenance contract auditing.

/// Bounded Levenberg-Marquardt reverse-MBD parameter identification.
pub mod calibrate;
/// End-to-end physical contract and provenance auditing on `VerifiedSyspack`.
pub mod contract;
/// Typed errors returned by forward and reverse MBD analyses.
pub mod error;
/// Orbit sunlit/eclipse power balance and thermal operating envelope analysis.
pub mod power_thermal;
/// Bottom-to-top multistage launch vehicle Tsiolkovsky $\Delta v$ and TWR budgeting.
pub mod staging;
/// Longitudinal CG fuel-trim excursion and static-margin sweeps.
pub mod trim;

pub use calibrate::{
    CalibrationObservation, CalibrationParamSpec, CalibrationReport, fit_parameters_lm,
};
pub use contract::{DesignAuditReport, DesignFinding, FindingSeverity, audit_verified_syspack};
pub use error::DesignError;
pub use power_thermal::{
    OrbitalPowerScenario, PowerBalanceReport, ThermalEnvelopeReport, ThermalNodeBounds,
    analyze_orbital_power_balance, analyze_thermal_envelope,
};
pub use staging::{
    MultistageBudgetReport, StageDesignSpec, StagePerformance, analyze_multistage_stack,
};
pub use trim::{CgEnvelopeReport, CgScenarioResult, TankFillScenario, analyze_cg_envelope};
