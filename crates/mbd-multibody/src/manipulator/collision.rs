use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{ComponentId, PoseTable};

use crate::MultibodyError;

/// Swept-sphere capsule attached to a component link for self-collision and
/// spacecraft keep-out hull clearance checks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkCapsule {
    /// Owning component link.
    pub component: ComponentId,
    /// Capsule segment start in local component frame, m.
    pub start_local_m: DVec3,
    /// Capsule segment end in local component frame, m.
    pub end_local_m: DVec3,
    /// Swept radius, m (`>= 0`).
    pub radius_m: f64,
}

/// Minimum signed surface clearance across all attached capsule pairs not listed
/// in `ignored_pairs` (negative value indicates penetration depth).
pub fn minimum_capsule_clearance(
    poses: &PoseTable,
    capsules: &[LinkCapsule],
    ignored_pairs: &[(usize, usize)],
) -> Result<f64, MultibodyError> {
    let mut world = Vec::with_capacity(capsules.len());
    for cap in capsules {
        let pose = poses.get(cap.component)?;
        let a = pose.position_root_m + pose.rotation_root_from_link * cap.start_local_m;
        let b = pose.position_root_m + pose.rotation_root_from_link * cap.end_local_m;
        world.push((pose.attached, a, b, cap.radius_m));
    }
    let mut min_clear = f64::INFINITY;
    for (i, &(att_i, a0, a1, r_i)) in world.iter().enumerate() {
        if !att_i {
            continue;
        }
        for (j, &(att_j, b0, b1, r_j)) in world.iter().enumerate().skip(i.wrapping_add(1)) {
            if !att_j
                || ignored_pairs
                    .iter()
                    .any(|&(u, v)| (u == i && v == j) || (u == j && v == i))
            {
                continue;
            }
            let dist = segment_segment_distance(a0, a1, b0, b1);
            min_clear = min_clear.min(dist - (r_i + r_j));
        }
    }
    Ok(min_clear)
}

/// Closest Euclidean distance between 3D line segments `[p1, q1]` and `[p2, q2]`.
fn segment_segment_distance(p1: DVec3, q1: DVec3, p2: DVec3, q2: DVec3) -> f64 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (s, t) = if a <= 1e-18 && e <= 1e-18 {
        (0.0, 0.0)
    } else if a <= 1e-18 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-18 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom != 0.0 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    (c1 - c2).length()
}
