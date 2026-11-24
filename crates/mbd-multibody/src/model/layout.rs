//! Flat continuous-state layout of a prepared model.
//!
//! ```text
//! [ base position 3 | base attitude 4 | joint positions ... |
//!   base linear rate 3 | base angular rate 3 | joint rates ... | element states ... ]
//! ```
//!
//! The base position and linear rate are in world axes, so the position
//! rate is the linear rate itself; the base angular rate is in root axes.
//! Joint blocks follow chain order. A revolute or prismatic joint holds one
//! coordinate and one rate; a spherical joint a unit quaternion and a
//! placed-frame angular rate; a compliant joint a translation, a unit
//! quaternion, a linear rate and an angular rate. Quaternions are stored
//! `x y z w`.

use alloc::vec::Vec;

use glam::{DQuat, DVec3};
use mbd_core::{JointKind, JointState};

/// Physical kind of one flat state component, which selects its tolerance
/// and its characteristic magnitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateKind {
    /// Translation, m.
    Length,
    /// One component of a unit quaternion.
    Attitude,
    /// Hinge angle, rad.
    Angle,
    /// Translational rate, m/s.
    LinearRate,
    /// Rotational rate, rad/s.
    AngularRate,
    /// Continuous state owned by a force element.
    Auxiliary,
}

/// Offsets of one link's joint coordinates and rates in the flat state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct LinkSlots {
    pub(crate) q: usize,
    pub(crate) v: usize,
}

/// Position and rate layout of a prepared model, excluding element states,
/// which follow at [`StateLayout::len`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StateLayout {
    nq: usize,
    nv: usize,
    kinds: Vec<StateKind>,
    quaternions: Vec<usize>,
}

/// Flat offset of the base position.
pub(crate) const BASE_POSITION: usize = 0;
/// Flat offset of the base attitude quaternion.
pub(crate) const BASE_ATTITUDE: usize = 3;
const BASE_Q: usize = 7;
const BASE_V: usize = 6;

impl StateLayout {
    /// Lays out the base and the joints of `kinds`, in chain order, returning
    /// each link's offsets.
    pub(crate) fn new(kinds: impl Iterator<Item = JointKind> + Clone) -> (Self, Vec<LinkSlots>) {
        let nq = BASE_Q + kinds.clone().map(|k| joint_q_len(&k)).sum::<usize>();
        let mut slots = Vec::new();
        let mut state_kinds = alloc::vec![StateKind::Length; 3];
        state_kinds.extend([StateKind::Attitude; 4]);
        let mut quaternions = alloc::vec![BASE_ATTITUDE];
        let (mut q, mut v) = (BASE_Q, nq + BASE_V);
        let mut rate_kinds = alloc::vec![StateKind::LinearRate; 3];
        rate_kinds.extend([StateKind::AngularRate; 3]);
        for kind in kinds {
            slots.push(LinkSlots { q, v });
            match kind {
                JointKind::Fixed => {}
                JointKind::Revolute { .. } => {
                    state_kinds.push(StateKind::Angle);
                    rate_kinds.push(StateKind::AngularRate);
                }
                JointKind::Prismatic { .. } => {
                    state_kinds.push(StateKind::Length);
                    rate_kinds.push(StateKind::LinearRate);
                }
                JointKind::Spherical { .. } => {
                    quaternions.push(q);
                    state_kinds.extend([StateKind::Attitude; 4]);
                    rate_kinds.extend([StateKind::AngularRate; 3]);
                }
                JointKind::Compliant6Dof { .. } => {
                    quaternions.push(q + 3);
                    state_kinds.extend([StateKind::Length; 3]);
                    state_kinds.extend([StateKind::Attitude; 4]);
                    rate_kinds.extend([StateKind::LinearRate; 3]);
                    rate_kinds.extend([StateKind::AngularRate; 3]);
                }
            }
            q += joint_q_len(&kind);
            v += joint_v_len(&kind);
        }
        let nv = rate_kinds.len();
        state_kinds.extend(rate_kinds);
        let layout = Self {
            nq,
            nv,
            kinds: state_kinds,
            quaternions,
        };
        (layout, slots)
    }

    /// Number of position components (`q`, quaternions counted as four).
    pub const fn position_len(&self) -> usize {
        self.nq
    }

    /// Number of rate components (`v`); also the number of degrees of freedom.
    pub const fn velocity_len(&self) -> usize {
        self.nv
    }

    /// Length of the position and rate blocks together; element states
    /// start at this offset.
    pub const fn len(&self) -> usize {
        self.nq + self.nv
    }

    /// Whether the layout holds no component at all (never true for a
    /// prepared model, whose base is always present).
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Kind of every position and rate component, in flat order.
    pub fn kinds(&self) -> &[StateKind] {
        &self.kinds
    }

    /// Flat offsets of the four-component unit quaternions. An integrator
    /// must renormalise these after every update of the state.
    pub fn quaternion_offsets(&self) -> &[usize] {
        &self.quaternions
    }

    /// Renormalises every quaternion block of `x`: the retraction onto the
    /// state manifold.
    pub fn retract(&self, x: &mut [f64]) {
        for &at in &self.quaternions {
            write_quat(x, at, read_quat(x, at).normalize());
        }
    }
}

pub(crate) const fn joint_q_len(kind: &JointKind) -> usize {
    match kind {
        JointKind::Fixed => 0,
        JointKind::Revolute { .. } | JointKind::Prismatic { .. } => 1,
        JointKind::Spherical { .. } => 4,
        JointKind::Compliant6Dof { .. } => 7,
    }
}

pub(crate) const fn joint_v_len(kind: &JointKind) -> usize {
    match kind {
        JointKind::Fixed => 0,
        JointKind::Revolute { .. } | JointKind::Prismatic { .. } => 1,
        JointKind::Spherical { .. } => 3,
        JointKind::Compliant6Dof { .. } => 6,
    }
}

/// Reads one component; an offset outside `x` reads as zero, which callers
/// exclude by checking the state length once up front.
pub(crate) fn read1(x: &[f64], at: usize) -> f64 {
    x.get(at).copied().unwrap_or(0.0)
}

pub(crate) fn read3(x: &[f64], at: usize) -> DVec3 {
    match x.get(at..at.saturating_add(3)) {
        Some(&[a, b, c]) => DVec3::new(a, b, c),
        _ => DVec3::ZERO,
    }
}

pub(crate) fn read_quat(x: &[f64], at: usize) -> DQuat {
    match x.get(at..at.saturating_add(4)) {
        Some(&[a, b, c, d]) => DQuat::from_xyzw(a, b, c, d),
        _ => DQuat::IDENTITY,
    }
}

pub(crate) fn write1(x: &mut [f64], at: usize, value: f64) {
    if let Some(slot) = x.get_mut(at) {
        *slot = value;
    }
}

pub(crate) fn write3(x: &mut [f64], at: usize, value: DVec3) {
    if let Some(slot) = x.get_mut(at..at.saturating_add(3)) {
        slot.copy_from_slice(&value.to_array());
    }
}

pub(crate) fn write_quat(x: &mut [f64], at: usize, value: DQuat) {
    if let Some(slot) = x.get_mut(at..at.saturating_add(4)) {
        slot.copy_from_slice(&value.to_array());
    }
}

/// Joint state of a link read from the flat state; fields the joint kind
/// does not use keep their neutral values.
pub(crate) fn read_joint(kind: &JointKind, slots: LinkSlots, x: &[f64]) -> JointState {
    let mut joint = JointState::FIXED;
    match kind {
        JointKind::Fixed => {}
        JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
            joint.q_scalar = read1(x, slots.q);
            joint.q_vel_scalar = read1(x, slots.v);
        }
        JointKind::Spherical { .. } => {
            joint.rotation = read_quat(x, slots.q);
            joint.angular_vel_rad_s = read3(x, slots.v);
        }
        JointKind::Compliant6Dof { .. } => {
            joint.translation_m = read3(x, slots.q);
            joint.rotation = read_quat(x, slots.q + 3);
            joint.linear_vel_m_s = read3(x, slots.v);
            joint.angular_vel_rad_s = read3(x, slots.v + 3);
        }
    }
    joint
}

/// Writes the fields of `joint` its kind uses into the flat state.
pub(crate) fn write_joint(kind: &JointKind, slots: LinkSlots, joint: &JointState, x: &mut [f64]) {
    match kind {
        JointKind::Fixed => {}
        JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
            write1(x, slots.q, joint.q_scalar);
            write1(x, slots.v, joint.q_vel_scalar);
        }
        JointKind::Spherical { .. } => {
            write_quat(x, slots.q, joint.rotation);
            write3(x, slots.v, joint.angular_vel_rad_s);
        }
        JointKind::Compliant6Dof { .. } => {
            write3(x, slots.q, joint.translation_m);
            write_quat(x, slots.q + 3, joint.rotation);
            write3(x, slots.v, joint.linear_vel_m_s);
            write3(x, slots.v + 3, joint.angular_vel_rad_s);
        }
    }
}
