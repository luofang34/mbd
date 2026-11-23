//! Aerodynamic elements of articulated vehicles, built on
//! [`mbd_core::AmbientGas`] and the force-element interface of
//! [`mbd_multibody`].
//!
//! Each [`AeroElement`] is bound to one component: it sees the motion of
//! that link relative to the air the host samples for it, joint motion and
//! placement included, and applies its load to that link, so articulated
//! wings, rotors and canopies load their own joints. Continuum, transonic,
//! supersonic, hypersonic, free-molecular, parachute, rotor BEM and
//! insect-scale flapping models share the interface.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod body_fin;
mod capsule;
mod dummy;
mod element;
mod flapping;
mod flow;
mod parachute;
mod rarefied;
mod rotor;
mod validation;
mod wing;

pub use body_fin::{BodyAndFinAero, BodyAndFinParams};
pub use capsule::{BluntCapsuleAero, BluntCapsuleParams};
pub use dummy::{HumanDummyAero, HumanDummyParams};
pub use element::{AeroElement, AeroElementKind, AeroElementLoads};
pub use flapping::{
    FlappingWingAero, FlappingWingLoads, FlappingWingParams, translational_coefficients,
};
pub use flow::{AeroEnvironment, AeroInputs, AeroWrench, AirSample, LinkFlow};
pub use parachute::{ParachuteCanopyAero, ParachuteParams, ParachuteState};
pub use rarefied::{RarefiedPanelAero, RarefiedPanelParams};
pub use rotor::{RotorBemAero, RotorBemParams, RotorState, RotorWrench};
pub use wing::{WingSurfaceAero, WingSurfaceParams};
