mod airbreathing;
mod chemical;
mod electric;
mod plume;
mod solid;

#[cfg(test)]
mod tests;

pub use airbreathing::{
    AirbreathingEngineKind, AirbreathingEngineSpec, AirbreathingEngineState, AirbreathingStepInput,
    AirbreathingStepOutput, step_airbreathing_engine,
};
pub use chemical::{
    LiquidRocketSpec, LiquidRocketState, LiquidRocketStepInput, LiquidRocketStepOutput,
    step_liquid_rocket,
};
pub use electric::{
    AbepStepInput, AbepStepOutput, AbepThrusterSpec, AbepThrusterState, ElectricThrusterSpec,
    ElectricThrusterState, ElectricThrusterStepInput, ElectricThrusterStepOutput,
    step_abep_thruster, step_electric_thruster,
};
pub use plume::{
    PlumeImpingementResult, PlumeImpingementSpec, PlumeSurfaceTarget, evaluate_plume_impingement,
};
pub use solid::{SolidMotorSpec, SolidMotorState, SolidMotorStepOutput, step_solid_motor};
