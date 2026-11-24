use alloc::vec::Vec;

use glam::{DQuat, DVec3};

use super::{ComponentId, ComponentSpec, ComponentState, ComponentTree, Placement};
use crate::{JointKind, JointSpec, JointState, MbdError};

/// Result of splitting a subtree away from a parent [`ComponentTree`].
#[derive(Debug, Clone, PartialEq)]
pub struct SplitTrees {
    /// Remaining vehicle tree excluding the detached subtree.
    pub parent: ComponentTree,
    /// Detached subtree re-rooted at `child_root`.
    pub child: ComponentTree,
    /// `(original_id, new_parent_id)` mapping.
    pub parent_id_map: Vec<(ComponentId, ComponentId)>,
    /// `(original_id, new_child_id)` mapping.
    pub child_id_map: Vec<(ComponentId, ComponentId)>,
    /// Pose `(position, rotation)` of `child_root` in `parent`'s root frame at split.
    pub child_root_pose_in_parent: (DVec3, DQuat),
}

/// Result of attaching one [`ComponentTree`] onto a parent component of another.
#[derive(Debug, Clone, PartialEq)]
pub struct GraftedTree {
    /// Combined vehicle tree.
    pub tree: ComponentTree,
    /// `(original_base_id, combined_id)` mapping.
    pub base_id_map: Vec<(ComponentId, ComponentId)>,
    /// `(original_other_id, combined_id)` mapping.
    pub grafted_id_map: Vec<(ComponentId, ComponentId)>,
}

/// Result of changing a [`ComponentTree`]'s kinematic root node (used for Apollo
/// TD&E extraction, robotic grapple re-rooting, and vessel control handoff).
#[derive(Debug, Clone, PartialEq)]
pub struct RerootedTree {
    /// Tree rooted at the chosen component with identical world placements.
    pub tree: ComponentTree,
    /// `(original_id, rerooted_id)` mapping.
    pub id_map: Vec<(ComponentId, ComponentId)>,
    /// Pose `(position, rotation)` of the new root inside the old root frame.
    pub new_root_pose_in_old_root: (DVec3, DQuat),
}

impl ComponentTree {
    /// Splits out the attached subtree rooted at `child_root` into its own
    /// independent [`ComponentTree`], while removing that subtree from `self`.
    pub fn split_subtree(&self, child_root: ComponentId) -> Result<SplitTrees, MbdError> {
        let root_spec = self.spec(child_root)?;
        if root_spec.placement.parent.is_none() {
            return Err(MbdError::CannotSplitRoot { id: child_root });
        }
        if !self.is_attached(child_root)? {
            return Err(MbdError::DetachedComponent { id: child_root });
        }
        let child_root_pose_in_parent = self.world_transform_in_root(child_root)?;
        let flags = self.mark_subtree(child_root)?;
        let (parent_ids, child_ids) = partition_ids(&flags);
        let parent_id_map = build_id_map(&parent_ids);
        let child_id_map = build_id_map(&child_ids);
        let parent = self.rebuild_filtered(&parent_ids, &parent_id_map, None)?;
        let child = self.rebuild_filtered(&child_ids, &child_id_map, Some(child_root))?;
        Ok(SplitTrees {
            parent,
            child,
            parent_id_map,
            child_id_map,
            child_root_pose_in_parent,
        })
    }

    /// Attaches a single-root tree `other` as a child of `attach_parent` with
    /// `relative_placement` (its `parent` field is overridden to `Some(attach_parent)`)
    /// and the specified docking/grapple `joint` + `joint_state`.
    pub fn graft_subtree(
        &self,
        other: &ComponentTree,
        attach_parent: ComponentId,
        relative_placement: Placement,
        joint: JointSpec,
        joint_state: JointState,
    ) -> Result<GraftedTree, MbdError> {
        if !self.is_attached(attach_parent)? {
            return Err(MbdError::DetachedComponent { id: attach_parent });
        }
        if other.is_empty() {
            return Err(MbdError::EmptyTree);
        }
        let other_root = single_root_id(other)?;
        let mut items = Vec::with_capacity(self.len() + other.len());
        let mut base_id_map = Vec::with_capacity(self.len());
        let mut grafted_id_map = Vec::with_capacity(other.len());
        for (idx, id) in self.ids().enumerate() {
            let new_id = ComponentId(idx as u32);
            base_id_map.push((id, new_id));
            items.push((self.spec(id)?.clone(), *self.state(id)?));
        }
        let offset = self.len() as u32;
        for (idx, id) in other.ids().enumerate() {
            let new_id = ComponentId(offset.wrapping_add(idx as u32));
            grafted_id_map.push((id, new_id));
        }
        append_grafted(
            other,
            other_root,
            attach_parent,
            relative_placement,
            joint,
            joint_state,
            &base_id_map,
            &grafted_id_map,
            &mut items,
        )?;
        Ok(GraftedTree {
            tree: ComponentTree::new(items)?,
            base_id_map,
            grafted_id_map,
        })
    }

    /// Re-roots the tree at an attached component `new_root` by reversing parent
    /// pointers along the path from `new_root` to the former root.
    ///
    /// Supports exact analytical reversal across [`JointKind::Fixed`] (any offset),
    /// [`JointKind::Prismatic`] (any offset), and zero-translation
    /// [`JointKind::Revolute`] / [`JointKind::Spherical`] joints.
    pub fn reroot(&self, new_root: ComponentId) -> Result<RerootedTree, MbdError> {
        let new_root_pose_in_old_root = self.world_transform_in_root(new_root)?;
        if !self.is_attached(new_root)? {
            return Err(MbdError::DetachedComponent { id: new_root });
        }
        let path = self.ancestry_path(new_root)?;
        let mut new_specs = self.specs.clone();
        let mut new_states = self.states.clone();
        reverse_path(self, &path, &mut new_specs, &mut new_states)?;
        if let (Some(root_spec), Some(root_state)) = (
            new_specs.get_mut(new_root.0 as usize),
            new_states.get_mut(new_root.0 as usize),
        ) {
            root_spec.placement = Placement::ROOT;
            root_spec.joint = JointSpec::FIXED;
            root_state.joint = JointState::FIXED;
        }
        let items: Vec<_> = new_specs.into_iter().zip(new_states).collect();
        let id_map = self.ids().map(|id| (id, id)).collect();
        Ok(RerootedTree {
            tree: ComponentTree::new(items)?,
            id_map,
            new_root_pose_in_old_root,
        })
    }

    fn mark_subtree(&self, root: ComponentId) -> Result<Vec<bool>, MbdError> {
        let mut marks = Vec::with_capacity(self.len());
        for id in self.ids() {
            marks.push(self.is_in_subtree(id, root)?);
        }
        Ok(marks)
    }

    fn is_in_subtree(&self, id: ComponentId, subtree_root: ComponentId) -> Result<bool, MbdError> {
        let mut cursor = Some(id);
        let mut hops = 0usize;
        while let Some(current) = cursor {
            if current == subtree_root {
                return Ok(true);
            }
            if hops == self.len() {
                return Err(MbdError::HierarchyCycle { id });
            }
            hops = hops.wrapping_add(1);
            cursor = self.spec(current)?.placement.parent;
        }
        Ok(false)
    }

    fn rebuild_filtered(
        &self,
        keep_ids: &[ComponentId],
        map: &[(ComponentId, ComponentId)],
        promote_to_root: Option<ComponentId>,
    ) -> Result<ComponentTree, MbdError> {
        let mut items = Vec::with_capacity(keep_ids.len());
        for &old_id in keep_ids {
            let mut spec = self.spec(old_id)?.clone();
            let mut state = *self.state(old_id)?;
            if Some(old_id) == promote_to_root {
                spec.placement = Placement::ROOT;
                spec.joint = JointSpec::FIXED;
                state.joint = JointState::FIXED;
                state.attached = true;
            } else if let Some(pid) = spec.placement.parent {
                spec.placement.parent = Some(lookup_mapped(map, pid)?);
            }
            items.push((spec, state));
        }
        ComponentTree::new(items)
    }

    fn ancestry_path(&self, start: ComponentId) -> Result<Vec<ComponentId>, MbdError> {
        let mut path = Vec::new();
        let mut cursor = Some(start);
        while let Some(current) = cursor {
            if path.len() == self.len() {
                return Err(MbdError::HierarchyCycle { id: start });
            }
            path.push(current);
            cursor = self.spec(current)?.placement.parent;
        }
        Ok(path)
    }
}

fn single_root_id(tree: &ComponentTree) -> Result<ComponentId, MbdError> {
    let mut found = None;
    let mut count = 0usize;
    for id in tree.ids() {
        if tree.spec(id)?.placement.parent.is_none() {
            found = Some(id);
            count = count.wrapping_add(1);
        }
    }
    if count == 1
        && let Some(root) = found
    {
        return Ok(root);
    }
    Err(MbdError::InvalidRootCount { roots: count })
}

#[allow(clippy::too_many_arguments)]
fn append_grafted(
    other: &ComponentTree,
    other_root: ComponentId,
    attach_parent: ComponentId,
    relative_placement: Placement,
    joint: JointSpec,
    joint_state: JointState,
    base_map: &[(ComponentId, ComponentId)],
    other_map: &[(ComponentId, ComponentId)],
    items: &mut Vec<(ComponentSpec, ComponentState)>,
) -> Result<(), MbdError> {
    let parent_mapped = lookup_mapped(base_map, attach_parent)?;
    for id in other.ids() {
        let mut spec = other.spec(id)?.clone();
        let mut state = *other.state(id)?;
        if id == other_root {
            if spec.joint.kind != JointKind::Fixed {
                return Err(MbdError::UnsupportedRerootJoint { id });
            }
            spec.placement = Placement {
                parent: Some(parent_mapped),
                translation_m: relative_placement.translation_m
                    + relative_placement.rotation * spec.placement.translation_m,
                rotation: (relative_placement.rotation * spec.placement.rotation).normalize(),
            };
            spec.joint = joint;
            state.joint = joint_state;
            state.attached = true;
        } else if let Some(pid) = spec.placement.parent {
            spec.placement.parent = Some(lookup_mapped(other_map, pid)?);
        }
        items.push((spec, state));
    }
    Ok(())
}

fn reverse_path(
    tree: &ComponentTree,
    path: &[ComponentId],
    specs: &mut [ComponentSpec],
    states: &mut [ComponentState],
) -> Result<(), MbdError> {
    for pair in path.windows(2) {
        let (Some(&child_id), Some(&old_parent_id)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        let child_spec = tree.spec(child_id)?;
        let child_state = tree.state(child_id)?;
        let (inv_place, inv_joint, inv_jstate) = invert_edge(child_id, child_spec, child_state)?;
        let (Some(p_spec), Some(p_state)) = (
            specs.get_mut(old_parent_id.0 as usize),
            states.get_mut(old_parent_id.0 as usize),
        ) else {
            return Err(MbdError::UnknownComponent { id: old_parent_id });
        };
        p_spec.placement = inv_place;
        p_spec.joint = inv_joint;
        p_state.joint = inv_jstate;
    }
    Ok(())
}

fn invert_edge(
    child_id: ComponentId,
    child_spec: &ComponentSpec,
    child_state: &ComponentState,
) -> Result<(Placement, JointSpec, JointState), MbdError> {
    let t = child_spec.placement.translation_m;
    let r = child_spec.placement.rotation;
    let r_conj = r.conjugate();
    let zero_offset = t.length_squared() <= 1e-24;
    match child_spec.joint.kind {
        JointKind::Fixed => Ok((
            Placement {
                parent: Some(child_id),
                translation_m: -(r_conj * t),
                rotation: r_conj,
            },
            JointSpec::FIXED,
            JointState::FIXED,
        )),
        JointKind::Prismatic { axis, min_m, max_m } => Ok((
            Placement {
                parent: Some(child_id),
                translation_m: -(r_conj * t),
                rotation: r_conj,
            },
            JointSpec {
                kind: JointKind::Prismatic {
                    axis: r * axis,
                    min_m: -max_m,
                    max_m: -min_m,
                },
                ..child_spec.joint
            },
            JointState::prismatic(-child_state.joint.q_scalar, -child_state.joint.q_vel_scalar),
        )),
        JointKind::Revolute {
            axis,
            min_rad,
            max_rad,
        } if zero_offset => Ok((
            Placement {
                parent: Some(child_id),
                translation_m: DVec3::ZERO,
                rotation: r_conj,
            },
            JointSpec {
                kind: JointKind::Revolute {
                    axis: r * axis,
                    min_rad: -max_rad,
                    max_rad: -min_rad,
                },
                ..child_spec.joint
            },
            JointState::revolute(-child_state.joint.q_scalar, -child_state.joint.q_vel_scalar),
        )),
        JointKind::Spherical {
            cone_half_angle_rad,
        } if zero_offset => {
            let q_conj = child_state.joint.rotation.conjugate();
            Ok((
                Placement {
                    parent: Some(child_id),
                    translation_m: DVec3::ZERO,
                    rotation: r_conj,
                },
                JointSpec {
                    kind: JointKind::Spherical {
                        cone_half_angle_rad,
                    },
                    ..child_spec.joint
                },
                JointState::spherical(
                    (r * q_conj * r_conj).normalize(),
                    -(r * (q_conj * child_state.joint.angular_vel_rad_s)),
                ),
            ))
        }
        _ => Err(MbdError::UnsupportedRerootJoint { id: child_id }),
    }
}

fn partition_ids(marks: &[bool]) -> (Vec<ComponentId>, Vec<ComponentId>) {
    let mut parent = Vec::new();
    let mut child = Vec::new();
    for (idx, &in_child) in marks.iter().enumerate() {
        let id = ComponentId(idx as u32);
        if in_child {
            child.push(id);
        } else {
            parent.push(id);
        }
    }
    (parent, child)
}

fn build_id_map(ids: &[ComponentId]) -> Vec<(ComponentId, ComponentId)> {
    ids.iter()
        .enumerate()
        .map(|(new_idx, &old_id)| (old_id, ComponentId(new_idx as u32)))
        .collect()
}

fn lookup_mapped(
    map: &[(ComponentId, ComponentId)],
    target: ComponentId,
) -> Result<ComponentId, MbdError> {
    map.iter()
        .find(|&&(old, _)| old == target)
        .map(|&(_, new_id)| new_id)
        .ok_or(MbdError::UnknownComponent { id: target })
}
