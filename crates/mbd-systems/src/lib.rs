//! Unified aerospace, spacecraft, and aircraft physical subsystems shared as a single
//! source of truth across `mbd` design/verification tools and `Simulate`.
//!
//! Includes:
//! - [`fluid`]: Multi-tank storage, specific-force settling/unporting (`GravitySettled`,
//!   `UllageSettled`, `HeaderTank`, `BladderOrDiaphragm`, `CapillaryPmd`), lateral slosh modes,
//!   boil-off, dynamic CG/inertia contributions, and conserved crossfeed/trim transfers.
//! - [`propulsion`]: Liquid/monopropellant/bipropellant rockets and RCS, solid motors, power-limited
//!   electric ion/Hall thrusters, VLEO `AtomicO`-collecting ABEP thrusters, multi-species
//!   airbreathing engines, and structural/ground plume impingement.
//! - [`electrical`]: Solar arrays, primary/secondary batteries, RTGs/fuel cells/alternators, and
//!   priority load-shedding distribution buses emitting per-site `PowerRailState`.
//! - [`thermal`]: Lumped-capacitance thermal conduction/radiation/coolant networks, multi-species
//!   entry TPS ablation with RF plasma blackout, and VLEO atomic oxygen surface erosion.
//! - [`payload`]: Conserved inter-component crew/sample/cargo payload mass transfer.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

/// Solar arrays, batteries, generators, and prioritized load-shedding buses.
pub mod electrical;
/// Shared subsystem validation and runtime error types.
pub mod error;
/// Multi-species tanks, specific-force settling, slosh, boil-off, and CG trim networks.
pub mod fluid;
/// Conserved inter-component crew, sample, and cargo payload mass transfers.
pub mod payload;
/// Chemical, solid, electric, ABEP, and airbreathing propulsion plus plume impingement.
pub mod propulsion;
/// Lumped thermal networks, radiative/pumped loops, TPS ablation, and atomic-oxygen erosion.
pub mod thermal;

pub use electrical::{
    BatterySpec, BatteryState, ComponentRailOutput, ElectricalBus, ElectricalEnvInput,
    ElectricalStepOutput, PowerGeneratorKind, PowerGeneratorSpec, PowerGeneratorState,
    SolarArraySpec, SolarArrayState, evaluate_solar_array,
};
pub use error::SystemsError;
pub use fluid::{
    CgTrimController, FeedAvailability, FeedRegime, FluidNetwork, FluidSpecies, FluidStepOutput,
    SloshModeSpec, TankGeometry, TankId, TankSpec, TankState, TankStepInput, TankStepOutput,
    TransferLineSpec, TransferLineState, step_tank,
};
pub use payload::{drop_payload_mass, transfer_payload_mass};
pub use propulsion::{
    AbepStepInput, AbepStepOutput, AbepThrusterSpec, AbepThrusterState, AirbreathingEngineKind,
    AirbreathingEngineSpec, AirbreathingEngineState, AirbreathingStepInput, AirbreathingStepOutput,
    ElectricThrusterSpec, ElectricThrusterState, ElectricThrusterStepInput,
    ElectricThrusterStepOutput, LiquidRocketSpec, LiquidRocketState, LiquidRocketStepInput,
    LiquidRocketStepOutput, PlumeImpingementResult, PlumeImpingementSpec, PlumeSurfaceTarget,
    SolidMotorSpec, SolidMotorState, SolidMotorStepOutput, evaluate_plume_impingement,
    step_abep_thruster, step_airbreathing_engine, step_electric_thruster, step_liquid_rocket,
    step_solid_motor,
};
pub use thermal::{
    AtomicOxygenErosionOutput, AtomicOxygenErosionSpec, AtomicOxygenErosionState, ThermalLinkSpec,
    ThermalNetwork, ThermalNodeSpec, ThermalNodeState, ThermalStepEnv, ThermalStepOutput,
    TpsAblationOutput, TpsAblationSpec, TpsAblationState, apply_tps_ablation_to_tree,
    step_atomic_oxygen_erosion, step_tps_ablation,
};
