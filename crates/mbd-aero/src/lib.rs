//! Unified aerodynamic element library built on [`mbd_core::AmbientGas`]
//! and [`mbd_core::PoseTable`].
//!
//! Every continuum, transonic, supersonic, hypersonic, rarefied free-molecular,
//! parachute, rotorcraft/propeller BEM, and composite multi-body load shares
//! these pure `no_std` element models.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod body_fin;
mod capsule;
mod composite;
mod dummy;
mod parachute;
mod rarefied;
mod rotor;
mod validation;
mod wing;

pub use body_fin::{BodyAndFinAero, BodyAndFinParams};
pub use capsule::{BluntCapsuleAero, BluntCapsuleParams};
pub use composite::{AeroElement, BoundAeroElement, CompositeAeroModel};
pub use dummy::{HumanDummyAero, HumanDummyParams};
pub use parachute::{ParachuteCanopyAero, ParachuteParams, ParachuteState};
pub use rarefied::{RarefiedPanelAero, RarefiedPanelParams};
pub use rotor::{RotorBemAero, RotorBemParams, RotorState, RotorWrench};
pub use wing::{WingSurfaceAero, WingSurfaceParams};
