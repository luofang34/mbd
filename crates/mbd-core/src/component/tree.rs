use alloc::vec::Vec;

use glam::{DQuat, DVec3};

use super::{ComponentId, ComponentSpec, ComponentState};
use crate::MbdError;

/// Frame of a component in the root, and whether its whole chain is attached.
pub(super) struct ChainPose {
    pub(super) translation: DVec3,
    pub(super) rotation: DQuat,
    pub(super) attached: bool,
}

/// The vehicle as a tree of placed components: fixed design data plus the
/// live state that changes as propellant burns and stages separate.
///
/// Every parent component exists and no chain cycles: construction rejects
/// anything else, and the topology never changes afterwards. A component's
/// [`ComponentId`] is its position in the list given to [`ComponentTree::new`],
/// so a parent may be listed after its children.
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentTree {
    specs: Vec<ComponentSpec>,
    states: Vec<ComponentState>,
}

impl ComponentTree {
    /// Builds a tree, numbering components in order, after validating every
    /// value and every parent chain.
    pub fn new(
        components: impl IntoIterator<Item = (ComponentSpec, ComponentState)>,
    ) -> Result<Self, MbdError> {
        let (specs, states): (Vec<_>, Vec<_>) = components.into_iter().unzip();
        let tree = Self { specs, states };
        for (spec, state) in tree.specs.iter().zip(&tree.states) {
            spec.validate()?;
            state.validate(&spec.name)?;
        }
        for id in tree.ids() {
            tree.chain_pose(id)?;
        }
        Ok(tree)
    }

    /// Number of components, attached or not.
    pub fn len(&self) -> usize {
        self.specs.len()
    }

    /// Whether the tree has no components.
    pub fn is_empty(&self) -> bool {
        self.specs.is_empty()
    }

    /// Every component id, in order.
    pub fn ids(&self) -> impl Iterator<Item = ComponentId> + use<> {
        (0u32..)
            .zip(0..self.specs.len())
            .map(|(id, _)| ComponentId(id))
    }

    /// Design-time data of `id`.
    pub fn spec(&self, id: ComponentId) -> Result<&ComponentSpec, MbdError> {
        self.specs
            .get(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })
    }

    /// Runtime state of `id`.
    pub fn state(&self, id: ComponentId) -> Result<&ComponentState, MbdError> {
        self.states
            .get(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })
    }

    /// Replaces the runtime state of `id`.
    pub fn set_state(&mut self, id: ComponentId, state: ComponentState) -> Result<(), MbdError> {
        state.validate(&self.spec(id)?.name)?;
        let slot = self
            .states
            .get_mut(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })?;
        *slot = state;
        Ok(())
    }

    /// All runtime states in id order, the tree's snapshot.
    pub fn states(&self) -> &[ComponentState] {
        &self.states
    }

    /// Restores a snapshot from [`ComponentTree::states`]. On error the tree
    /// is unchanged.
    pub fn restore_states(&mut self, states: &[ComponentState]) -> Result<(), MbdError> {
        if states.len() != self.states.len() {
            return Err(MbdError::StateCountMismatch {
                expected: self.states.len(),
                actual: states.len(),
            });
        }
        for (spec, state) in self.specs.iter().zip(states) {
            state.validate(&spec.name)?;
        }
        self.states.clear();
        self.states.extend_from_slice(states);
        Ok(())
    }

    /// Origin of `id` in root axes and its component-to-root rotation,
    /// composing every placement and articulation up the parent chain.
    pub fn world_transform_in_root(&self, id: ComponentId) -> Result<(DVec3, DQuat), MbdError> {
        let pose = self.chain_pose(id)?;
        Ok((pose.translation, pose.rotation))
    }

    /// Whether `id` and all its ancestors are still attached.
    pub fn is_attached(&self, id: ComponentId) -> Result<bool, MbdError> {
        Ok(self.chain_pose(id)?.attached)
    }

    /// Walks from `id` to its root. A walk longer than the tree has components
    /// has revisited one, which bounds the check without any allocation.
    pub(super) fn chain_pose(&self, id: ComponentId) -> Result<ChainPose, MbdError> {
        let mut pose = ChainPose {
            translation: DVec3::ZERO,
            rotation: DQuat::IDENTITY,
            attached: true,
        };
        let mut cursor = Some(id);
        let mut hops = 0usize;
        while let Some(current) = cursor {
            let placement = &self.spec(current)?.placement;
            if hops == self.specs.len() {
                return Err(MbdError::HierarchyCycle { id });
            }
            hops = hops.wrapping_add(1);
            let state = self.state(current)?;
            let rotation = placement.rotation * state.articulation;
            pose.translation = placement.translation_m + rotation * pose.translation;
            pose.rotation = rotation * pose.rotation;
            pose.attached &= state.attached;
            cursor = placement.parent;
        }
        pose.rotation = pose.rotation.normalize();
        Ok(pose)
    }
}
