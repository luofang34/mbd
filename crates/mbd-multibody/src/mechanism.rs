mod docking;
mod release;
mod tether;

#[cfg(test)]
mod tests;

pub use docking::{
    DockingPhase, DockingSpec, DockingState, DockingStepOutput, GrappleLockSpec, HardMateResult,
    grapple_lock, hard_mate_vessels, step_docking_capture, undock_vessel,
};
pub use release::{
    HingeSwingSpec, HingeSwingState, ImpulseReleaseResult, RailDriveOutput, RailSlideSpec,
    RailSlideState, execute_impulse_release, release_from_rail, step_hinge_release,
    step_rail_drive,
};
pub use tether::{
    TetherCutReason, TetherStepOutput, WinchTetherSpec, WinchTetherState, step_tether,
};
