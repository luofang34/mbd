//! Zero-degree-of-freedom solve: every joint fixed, so the tree is one rigid
//! body and its acceleration follows from the composite inertia alone.

use glam::{DMat3, DVec3};

use super::FloatingBaseState;
use super::articulated::{AbaBuffers, assign_slot};
use super::chain::LinkData;
use crate::MultibodyError;
use crate::spatial::{SpatialForce, SpatialMotion};

pub(crate) fn solve_rigid(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
    buf: &mut AbaBuffers,
) -> Result<SpatialMotion, MultibodyError> {
    let n = links.len();
    buf.reset(n);
    let (inertias, wrenches) = (&mut buf.ia, &mut buf.pa);
    let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
    let g_root = r0.transpose() * gravity_world;

    for (idx, link) in links.iter().enumerate() {
        assign_slot(inertias, idx, link.i_local);
        assign_slot(wrenches, idx, link.ext_force_local);
    }
    for idx in (1..n).rev() {
        let (Some(link), Some(&i_c), Some(&f_c)) =
            (links.get(idx), inertias.get(idx), wrenches.get(idx))
        else {
            continue;
        };
        let Some(p_idx) = link.parent_order else {
            continue;
        };
        let i_shift = i_c.transform_to_parent(&link.x_parent_from_child);
        let f_shift = link.x_parent_from_child.apply_force_to_parent(f_c);
        if let Some(i_p) = inertias.get_mut(p_idx) {
            *i_p += i_shift;
        }
        if let Some(f_p) = wrenches.get_mut(p_idx) {
            *f_p += f_shift;
        }
    }

    let i_total = inertias
        .first()
        .copied()
        .ok_or(MultibodyError::SingularMassMatrix)?;
    let f_ext = wrenches.first().copied().unwrap_or(SpatialForce::ZERO);
    let v0 = base_state.spatial_velocity_root();
    let f_grav = i_total.mul_motion(SpatialMotion::new(DVec3::ZERO, g_root));
    let bias = v0.cross_force(i_total.mul_motion(v0));
    i_total
        .solve(f_ext + f_grav - bias)
        .ok_or(MultibodyError::SingularMassMatrix)
}
