//! A component tree prepared for repeated dynamics evaluation: topology and
//! link constants resolved once, the state held outside as an explicit
//! value.

use alloc::vec::Vec;

use mbd_core::{ComponentId, ComponentTree, DynamicMassContribution, JointState, MbdError};

use crate::MultibodyError;
use crate::aba::FloatingBaseState;
use crate::aba::chain::{PreparedLink, attached_joint_kinds, prepare_links, refresh_link_mass};
use crate::element::ForceElement;

mod dynamics;
mod kinematics;
pub(crate) mod layout;
mod loads;
mod ode;
mod operators;
mod state;
mod workspace;

pub use kinematics::{Kinematics, PointKinematics};
pub use layout::{StateKind, StateLayout};
pub use loads::Loads;
pub use ode::ModelOde;
pub use operators::Resolution;
pub use state::ModelState;
pub use workspace::Workspace;

/// Characteristic length and time of a model. Absolute tolerances and
/// finite-difference steps are multiples of these, which keeps integrator
/// behaviour the same for a 1 mg insect and a 1 t aircraft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelScale {
    /// Characteristic length, m.
    pub length_m: f64,
    /// Characteristic time of the motion of interest, s.
    pub time_s: f64,
}

/// The attached links of a [`ComponentTree`] in parent-before-child order
/// with their constants, ready for evaluation at any state.
///
/// The model holds no state. [`MultibodyModel::state`] reads one from the
/// tree, [`MultibodyModel::derivative`] evaluates the equations of motion at
/// a state, and [`MultibodyModel::write_joints`] writes joint coordinates
/// back when the tree is needed for pose queries.
///
/// A model is valid while the tree's attachment topology and joint
/// definitions stay as they were at preparation. Mass changes (propellant,
/// payload, distributed masses) are picked up by
/// [`MultibodyModel::refresh_mass`] without rebuilding; a separation or a
/// docking needs a new model.
#[derive(Debug, Clone, PartialEq)]
pub struct MultibodyModel {
    pub(crate) links: Vec<PreparedLink>,
    pub(crate) order_of: Vec<Option<usize>>,
    pub(crate) layout: StateLayout,
    pub(crate) all_fixed: bool,
    pub(crate) scale: ModelScale,
}

impl MultibodyModel {
    /// Prepares the attached chain of `tree` with `dynamic_masses` riding on
    /// its components.
    pub fn new(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
    ) -> Result<Self, MultibodyError> {
        let kinds = attached_joint_kinds(tree)?;
        let (layout, slots) = StateLayout::new(kinds.iter().copied());
        let (links, order_of) = prepare_links(tree, dynamic_masses, &slots)?;
        let all_fixed = kinds
            .iter()
            .all(|k| matches!(k, mbd_core::JointKind::Fixed));
        let length_m = links.iter().fold(0.0_f64, |acc, l| {
            let gyration = if l.mass_kg > 0.0 {
                let i = l.inertia_cg_local;
                libm::sqrt((i.x_axis.x + i.y_axis.y + i.z_axis.z).max(0.0) / l.mass_kg)
            } else {
                0.0
            };
            acc.max(l.placement.translation_m.length())
                .max(l.cg_local_m.length())
                .max(gyration)
        });
        Ok(Self {
            links,
            order_of,
            layout,
            all_fixed,
            scale: ModelScale {
                length_m: if length_m > 0.0 { length_m } else { 1.0 },
                time_s: 1.0,
            },
        })
    }

    /// Recomputes every link's mass properties from the tree's current
    /// component masses and `dynamic_masses`. Does not allocate.
    pub fn refresh_mass(
        &mut self,
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
    ) -> Result<(), MultibodyError> {
        for m in dynamic_masses {
            m.validate()?;
        }
        for link in &mut self.links {
            refresh_link_mass(link, tree, dynamic_masses)?;
        }
        Ok(())
    }

    /// Layout of the position and rate blocks of the flat state.
    pub const fn layout(&self) -> &StateLayout {
        &self.layout
    }

    /// Characteristic length and time. The length defaults to the largest
    /// link offset or radius of gyration in the tree; the time defaults to
    /// one second and should be set to the period of the motion of interest.
    pub const fn scale(&self) -> ModelScale {
        self.scale
    }

    /// Replaces the characteristic length and time.
    pub fn set_scale(&mut self, scale: ModelScale) -> Result<(), MultibodyError> {
        for (field, value) in [("length_m", scale.length_m), ("time_s", scale.time_s)] {
            if !value.is_finite() || value <= 0.0 {
                return Err(MultibodyError::InvalidParameter { field, value });
            }
        }
        self.scale = scale;
        Ok(())
    }

    /// Components of the attached chain, parent before child.
    pub fn components(&self) -> impl Iterator<Item = ComponentId> + '_ {
        self.links.iter().map(|l| l.id)
    }

    pub(crate) fn link(&self, id: ComponentId) -> Result<&PreparedLink, MultibodyError> {
        let order = self
            .order_of
            .get(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })?;
        order
            .and_then(|idx| self.links.get(idx))
            .ok_or(MultibodyError::DetachedComponent { id })
    }

    /// State of `tree` at `base`, with every element of `elements` at its
    /// initial state. `inputs` only names the input type the elements will
    /// be evaluated with; its value is not read.
    pub fn state<I: ?Sized, E: ForceElement<I> + ?Sized>(
        &self,
        tree: &ComponentTree,
        base: &FloatingBaseState,
        elements: &E,
        _inputs: &I,
    ) -> Result<ModelState, MultibodyError> {
        let mut state = ModelState::zeroed(
            self.layout.position_len(),
            self.layout.velocity_len(),
            elements.state_len(),
            elements.mode_len(),
        );
        state.set_base(base);
        for link in &self.links {
            let joint = tree.state(link.id)?.joint;
            layout::write_joint(&link.joint.kind, link.slots, &joint, state.as_mut_slice());
        }
        let at = self.layout.len();
        let (x, modes) = state.parts_mut();
        elements.initial_state(x.get_mut(at..).unwrap_or(&mut []), modes);
        Ok(state)
    }

    /// Joint coordinate and rate of `component` in `state`.
    pub fn joint_state(
        &self,
        state: &ModelState,
        component: ComponentId,
    ) -> Result<JointState, MultibodyError> {
        let link = self.link(component)?;
        Ok(layout::read_joint(
            &link.joint.kind,
            link.slots,
            state.as_slice(),
        ))
    }

    /// Replaces the joint coordinate and rate of `component` in `state`.
    pub fn set_joint_state(
        &self,
        state: &mut ModelState,
        component: ComponentId,
        joint: &JointState,
    ) -> Result<(), MultibodyError> {
        joint.validate("joint")?;
        let link = self.link(component)?;
        layout::write_joint(&link.joint.kind, link.slots, joint, state.as_mut_slice());
        Ok(())
    }

    /// Writes the joint coordinates and rates of `state` into `tree`,
    /// leaving every other part of the tree's component states as it is.
    pub fn write_joints(
        &self,
        state: &ModelState,
        tree: &mut ComponentTree,
    ) -> Result<(), MultibodyError> {
        for link in &self.links {
            let mut component = *tree.state(link.id)?;
            let fresh = layout::read_joint(&link.joint.kind, link.slots, state.as_slice());
            component.joint = merge_joint(&link.joint.kind, component.joint, fresh);
            tree.set_state(link.id, component)?;
        }
        Ok(())
    }
}

/// `fresh` for the fields `kind` integrates, `kept` for the others.
fn merge_joint(kind: &mbd_core::JointKind, kept: JointState, fresh: JointState) -> JointState {
    use mbd_core::JointKind;
    match kind {
        JointKind::Fixed => kept,
        JointKind::Revolute { .. } | JointKind::Prismatic { .. } => JointState {
            q_scalar: fresh.q_scalar,
            q_vel_scalar: fresh.q_vel_scalar,
            ..kept
        },
        JointKind::Spherical { .. } => JointState {
            rotation: fresh.rotation,
            angular_vel_rad_s: fresh.angular_vel_rad_s,
            ..kept
        },
        JointKind::Compliant6Dof { .. } => JointState {
            q_scalar: kept.q_scalar,
            q_vel_scalar: kept.q_vel_scalar,
            ..fresh
        },
    }
}
