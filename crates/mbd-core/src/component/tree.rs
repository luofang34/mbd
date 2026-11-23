use alloc::vec;
use alloc::vec::Vec;

use glam::{DQuat, DVec3};

use super::{ComponentId, ComponentSpec, ComponentState};
use crate::{LinkPose, MbdError, PoseTable};

/// Frame of a component in the root, and whether its whole chain is attached.
pub(super) struct ChainPose {
    pub(super) translation: DVec3,
    pub(super) rotation: DQuat,
    pub(super) attached: bool,
}

/// The vehicle as a tree of placed components: fixed design data plus the
/// live state that changes as propellant burns, joints articulate, and stages
/// separate or dock.
///
/// Every parent component exists and no chain cycles: construction rejects
/// anything else. A component's [`ComponentId`] is its position in the list
/// given to [`ComponentTree::new`], so a parent may be listed after its
/// children. Structural reconfiguration operations ([`ComponentTree::split_subtree`],
/// [`ComponentTree::graft_subtree`], and [`ComponentTree::reroot`]) produce new
/// validated trees along with explicit ID maps.
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentTree {
    pub(super) specs: Vec<ComponentSpec>,
    pub(super) states: Vec<ComponentState>,
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
            state.validate(spec)?;
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
        state.validate(self.spec(id)?)?;
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
            state.validate(spec)?;
        }
        self.states.clear();
        self.states.extend_from_slice(states);
        Ok(())
    }

    /// Origin of `id` in root axes and its component-to-root rotation,
    /// composing every placement and joint transform up the parent chain.
    pub fn world_transform_in_root(&self, id: ComponentId) -> Result<(DVec3, DQuat), MbdError> {
        let pose = self.chain_pose(id)?;
        Ok((pose.translation, pose.rotation))
    }

    /// Whether `id` and all its ancestors are still attached.
    pub fn is_attached(&self, id: ComponentId) -> Result<bool, MbdError> {
        Ok(self.chain_pose(id)?.attached)
    }

    /// Evaluates every component's link pose and relative velocities in the
    /// vehicle root frame in a single $O(N)$ pass.
    pub fn evaluate_pose_table(&self) -> Result<PoseTable, MbdError> {
        let mut resolved: Vec<Option<LinkPose>> = vec![None; self.specs.len()];
        let mut chain = Vec::new();
        for id in self.ids() {
            if resolved.get(id.0 as usize).is_some_and(Option::is_some) {
                continue;
            }
            self.push_unresolved_chain(id, &resolved, &mut chain)?;
            while let Some(node_id) = chain.pop() {
                let pose = self.resolve_node_pose(node_id, &resolved)?;
                if let Some(slot) = resolved.get_mut(node_id.0 as usize) {
                    *slot = Some(pose);
                }
            }
        }
        let poses = resolved.into_iter().flatten().collect();
        Ok(PoseTable::from_vec(poses))
    }

    fn push_unresolved_chain(
        &self,
        start: ComponentId,
        resolved: &[Option<LinkPose>],
        chain: &mut Vec<ComponentId>,
    ) -> Result<(), MbdError> {
        chain.clear();
        let mut cursor = Some(start);
        while let Some(current) = cursor {
            if resolved
                .get(current.0 as usize)
                .is_some_and(Option::is_some)
            {
                break;
            }
            if chain.len() == self.specs.len() {
                return Err(MbdError::HierarchyCycle { id: start });
            }
            chain.push(current);
            cursor = self.spec(current)?.placement.parent;
        }
        Ok(())
    }

    fn resolve_node_pose(
        &self,
        id: ComponentId,
        resolved: &[Option<LinkPose>],
    ) -> Result<LinkPose, MbdError> {
        let spec = self.spec(id)?;
        let state = self.state(id)?;
        let parent_pose = match spec.placement.parent {
            Some(pid) => *resolved
                .get(pid.0 as usize)
                .and_then(Option::as_ref)
                .ok_or(MbdError::UnknownComponent { id: pid })?,
            None => LinkPose {
                id,
                position_root_m: DVec3::ZERO,
                rotation_root_from_link: DQuat::IDENTITY,
                linear_vel_root_m_s: DVec3::ZERO,
                angular_vel_root_rad_s: DVec3::ZERO,
                attached: true,
            },
        };
        step_link_pose(id, spec, state, parent_pose)
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
            let spec = self.spec(current)?;
            let placement = &spec.placement;
            if hops == self.specs.len() {
                return Err(MbdError::HierarchyCycle { id });
            }
            hops = hops.wrapping_add(1);
            let state = self.state(current)?;
            let (j_trans, j_rot) = spec.joint.local_transform(&state.joint)?;
            let local_rot = placement.rotation * j_rot;
            let local_trans = placement.translation_m + placement.rotation * j_trans;
            pose.translation = local_trans + local_rot * pose.translation;
            pose.rotation = local_rot * pose.rotation;
            pose.attached &= state.attached;
            cursor = placement.parent;
        }
        pose.rotation = pose.rotation.normalize();
        Ok(pose)
    }
}

fn step_link_pose(
    id: ComponentId,
    spec: &ComponentSpec,
    state: &ComponentState,
    parent: LinkPose,
) -> Result<LinkPose, MbdError> {
    let (j_trans, j_rot) = spec.joint.local_transform(&state.joint)?;
    let (j_vel, j_omega) = spec.joint.local_velocity(&state.joint);
    let placed_rot = parent.rotation_root_from_link * spec.placement.rotation;
    let local_trans = spec.placement.translation_m + spec.placement.rotation * j_trans;
    let rel = parent.rotation_root_from_link * local_trans;
    Ok(LinkPose {
        id,
        position_root_m: parent.position_root_m + rel,
        rotation_root_from_link: (placed_rot * j_rot).normalize(),
        linear_vel_root_m_s: parent.linear_vel_root_m_s
            + parent.angular_vel_root_rad_s.cross(rel)
            + placed_rot * j_vel,
        angular_vel_root_rad_s: parent.angular_vel_root_rad_s + placed_rot * j_omega,
        attached: parent.attached && state.attached,
    })
}
