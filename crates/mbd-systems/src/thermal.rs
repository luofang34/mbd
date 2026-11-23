mod network;
mod tps;

#[cfg(test)]
mod tests;

pub use network::{
    ThermalLinkSpec, ThermalNetwork, ThermalNodeSpec, ThermalNodeState, ThermalStepEnv,
    ThermalStepOutput,
};
pub use tps::{
    AtomicOxygenErosionOutput, AtomicOxygenErosionSpec, AtomicOxygenErosionState,
    TpsAblationOutput, TpsAblationSpec, TpsAblationState, apply_tps_ablation_to_tree,
    step_atomic_oxygen_erosion, step_tps_ablation,
};
