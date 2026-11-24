//! Soft-capture docking: a compliant ring between two ports whose stand-off
//! and settle dwell are continuous states and whose phase changes are
//! discrete events.

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, WrenchAtSite};

use crate::MultibodyError;
use crate::element::{ElementContext, ForceElement};
use crate::model::layout::{read1, write1};
use crate::model::{Kinematics, Loads, ModelScale};
use crate::spatial::SpatialForce;

/// Phases of an orbital rendezvous & docking / capture mechanism (`DockingCapture`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DockingPhase {
    /// Mechanisms open and ready outside contact envelope.
    Open,
    /// Soft-capture ring in compliant 6-DoF spring-damper contact.
    SoftContact,
    /// Relative motion settled below threshold; ready for probe retraction.
    SoftDamped,
    /// Retracting capture ring to bring structural docking latches flush.
    Retracting,
    /// Structurally hard-mated (`graft_subtree` locked into single multibody tree).
    HardLatched,
    /// Unlatched and pushing apart under undocking spring impulse.
    Undocking,
}

const PHASES: [DockingPhase; 6] = [
    DockingPhase::Open,
    DockingPhase::SoftContact,
    DockingPhase::SoftDamped,
    DockingPhase::Retracting,
    DockingPhase::HardLatched,
    DockingPhase::Undocking,
];

/// Configuration for compliant soft-to-hard spacecraft docking capture.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingSpec {
    /// Lateral capture envelope radius, m.
    pub capture_radius_m: f64,
    /// Angular alignment tolerance inside capture envelope, rad.
    pub capture_angle_rad: f64,
    /// Maximum axial closing speed accepted for capture, m/s.
    pub max_closing_speed_m_s: f64,
    /// Velocity threshold below which soft capture enters [`DockingPhase::SoftDamped`], m/s.
    pub damping_vel_threshold_m_s: f64,
    /// Angular rate threshold for [`DockingPhase::SoftDamped`], rad/s.
    pub damping_rate_threshold_rad_s: f64,
    /// Retraction speed along docking normal during [`DockingPhase::Retracting`], m/s.
    pub retraction_speed_m_s: f64,
    /// Linear soft-capture ring compliance stiffness, N/m.
    pub linear_stiffness_n_m: f64,
    /// Linear soft-capture viscous damping, N·s/m.
    pub linear_damping_n_s_m: f64,
    /// Rotational soft-capture ring compliance stiffness, N·m/rad.
    pub angular_stiffness_nm_rad: f64,
    /// Rotational soft-capture damping, N·m·s/rad.
    pub angular_damping_nm_s_rad: f64,
    /// Separation spring impulse when undocking, N·s.
    pub undock_impulse_ns: f64,
    /// Smallest stand-off the ring holds at capture, m, so retraction always
    /// has travel to take up.
    pub min_capture_standoff_m: f64,
    /// Time the relative motion must stay below the damping thresholds
    /// before retraction starts, s.
    pub damped_dwell_s: f64,
    /// Remaining stand-off below which retraction counts as complete, m.
    pub latch_standoff_tolerance_m: f64,
    /// Port separation within which the structural latches can engage, m.
    pub latch_distance_m: f64,
}

impl DockingSpec {
    /// Minimum capture stand-off of a human-scale docking ring, m.
    pub const DEFAULT_MIN_CAPTURE_STANDOFF_M: f64 = 0.05;
    /// Settle dwell of a human-scale docking ring, s.
    pub const DEFAULT_DAMPED_DWELL_S: f64 = 0.05;
    /// Retraction completion tolerance of a human-scale docking ring, m.
    pub const DEFAULT_LATCH_STANDOFF_TOLERANCE_M: f64 = 1e-4;
    /// Latch engagement distance of a human-scale docking ring, m.
    pub const DEFAULT_LATCH_DISTANCE_M: f64 = 0.05;

    /// Capture and latch distances as fractions of the capture radius
    /// (`1/8`, `1/4000` and `1/8`, the ratios of the human-scale defaults
    /// at a 0.4 m ring) and the dwell as the time to cross the minimum
    /// stand-off at the retraction speed.
    pub fn with_scaled_latching(mut self) -> Self {
        self.min_capture_standoff_m = self.capture_radius_m / 8.0;
        self.latch_standoff_tolerance_m = self.capture_radius_m / 4000.0;
        self.latch_distance_m = self.capture_radius_m / 8.0;
        if self.retraction_speed_m_s > 0.0 {
            self.damped_dwell_s = self.min_capture_standoff_m / self.retraction_speed_m_s;
        }
        self
    }
}

/// State of a [`DockingCapture`](DockingSpec) port pair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingState {
    /// Current mechanism phase.
    pub phase: DockingPhase,
    /// Current target retraction axial stand-off distance, m.
    pub standoff_m: f64,
    /// Accumulated dwell time below the damping thresholds, s.
    pub dwell_s: f64,
}

impl Default for DockingState {
    fn default() -> Self {
        Self {
            phase: DockingPhase::Open,
            standoff_m: 0.0,
            dwell_s: 0.0,
        }
    }
}

/// World pose and motion of one docking port.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PortKinematics {
    /// Port position in world axes, m.
    pub position_world_m: DVec3,
    /// World-from-port rotation.
    pub rotation_world: DQuat,
    /// Port velocity in world axes, m/s.
    pub velocity_world_m_s: DVec3,
    /// Port angular velocity in world axes, rad/s.
    pub angular_vel_world_rad_s: DVec3,
}

/// Ring wrenches and continuous-state rates of a capture at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingEvaluation {
    /// Soft-capture spatial wrench acting ON chaser (`A`) in world axes about port A.
    pub world_wrench_on_a: SpatialForce,
    /// Soft-capture spatial wrench acting ON target (`B`) in world axes about port B.
    pub world_wrench_on_b: SpatialForce,
    /// Rate of the stand-off, m/s (negative while retracting).
    pub standoff_rate_m_s: f64,
    /// Rate of the dwell timer: one while in soft contact below the damping
    /// thresholds, zero otherwise.
    pub dwell_rate: f64,
}

/// Outcome of the boundary events of a capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DockingTransition {
    /// Phase after the events.
    pub phase: DockingPhase,
    /// True on the boundary where retraction completes and
    /// [`super::hard_mate_vessels`] should lock the trees.
    pub ready_for_hard_mate: bool,
    /// Whether the phase or a state changed.
    pub changed: bool,
}

impl DockingSpec {
    /// Ring wrenches and state rates at one state: a pure function of its
    /// arguments, valid at any integrator stage. `axis_a_local` is the
    /// docking axis in port A's frame.
    pub fn evaluate(
        &self,
        state: &DockingState,
        a: &PortKinematics,
        b: &PortKinematics,
        axis_a_local: DVec3,
    ) -> DockingEvaluation {
        let engaged = matches!(
            state.phase,
            DockingPhase::SoftContact | DockingPhase::SoftDamped | DockingPhase::Retracting
        );
        if !engaged {
            return DockingEvaluation {
                world_wrench_on_a: SpatialForce::ZERO,
                world_wrench_on_b: SpatialForce::ZERO,
                standoff_rate_m_s: 0.0,
                dwell_rate: 0.0,
            };
        }
        let delta_pos = b.position_world_m - a.position_world_m;
        let delta_vel = b.velocity_world_m_s - a.velocity_world_m_s;
        let rel_rot = a.rotation_world.conjugate() * b.rotation_world;
        let rel_omega = b.angular_vel_world_rad_s - a.angular_vel_world_rad_s;
        let axis_world = (a.rotation_world * axis_a_local).normalize_or(DVec3::X);
        let target_pos_b = a.position_world_m + axis_world * state.standoff_m;
        let f_on_b = (target_pos_b - b.position_world_m) * self.linear_stiffness_n_m
            - delta_vel * self.linear_damping_n_s_m;
        let ax = a.rotation_world * rel_rot.to_scaled_axis();
        let t_on_b =
            (-ax) * self.angular_stiffness_nm_rad - rel_omega * self.angular_damping_nm_s_rad;
        let settled = state.phase == DockingPhase::SoftContact
            && delta_vel.length() <= self.damping_vel_threshold_m_s
            && rel_omega.length() <= self.damping_rate_threshold_rad_s;
        let retracting = state.phase == DockingPhase::Retracting && state.standoff_m > 0.0;
        DockingEvaluation {
            // Action-reaction about A and B: F_A = -F_B, and tau_A = -tau_B - (B - A) x F_B.
            world_wrench_on_a: SpatialForce::new(-t_on_b - delta_pos.cross(f_on_b), -f_on_b),
            world_wrench_on_b: SpatialForce::new(t_on_b, f_on_b),
            standoff_rate_m_s: if retracting {
                -self.retraction_speed_m_s
            } else {
                0.0
            },
            dwell_rate: if settled { 1.0 } else { 0.0 },
        }
    }

    /// Applies the discrete events of a capture at an accepted step
    /// boundary: capture inside the envelope, the start of retraction after
    /// the settle dwell, and the hard latch once retraction completes.
    pub fn transition(
        &self,
        state: &mut DockingState,
        a: &PortKinematics,
        b: &PortKinematics,
        axis_a_local: DVec3,
    ) -> DockingTransition {
        let before = *state;
        let delta_pos = b.position_world_m - a.position_world_m;
        let dist = delta_pos.length();
        let mut ready = false;
        match state.phase {
            DockingPhase::Open => {
                let delta_vel = b.velocity_world_m_s - a.velocity_world_m_s;
                let (_, align) = (a.rotation_world.conjugate() * b.rotation_world).to_axis_angle();
                if dist <= self.capture_radius_m
                    && align <= self.capture_angle_rad
                    && delta_vel.length() <= self.max_closing_speed_m_s
                {
                    let axis_world = (a.rotation_world * axis_a_local).normalize_or(DVec3::X);
                    state.phase = DockingPhase::SoftContact;
                    state.standoff_m = delta_pos.dot(axis_world).max(self.min_capture_standoff_m);
                    state.dwell_s = 0.0;
                }
            }
            DockingPhase::SoftContact | DockingPhase::SoftDamped => {
                if state.dwell_s >= self.damped_dwell_s {
                    state.phase = DockingPhase::Retracting;
                }
            }
            DockingPhase::Retracting => {
                state.standoff_m = state.standoff_m.max(0.0);
                ready = state.standoff_m <= self.latch_standoff_tolerance_m
                    && dist <= self.latch_distance_m;
                if ready {
                    state.phase = DockingPhase::HardLatched;
                }
            }
            DockingPhase::HardLatched | DockingPhase::Undocking => {}
        }
        DockingTransition {
            phase: state.phase,
            ready_for_hard_mate: ready,
            changed: *state != before,
        }
    }
}

impl DockingState {
    /// Reads the state from an element's continuous block and mode block.
    pub fn from_blocks(state: &[f64], modes: &[f64]) -> Self {
        let code = read1(modes, 0);
        Self {
            phase: (0u8..)
                .zip(PHASES)
                .find_map(|(k, phase)| (f64::from(k) == code).then_some(phase))
                .unwrap_or(DockingPhase::Open),
            standoff_m: read1(state, 0),
            dwell_s: read1(state, 1),
        }
    }

    /// Writes the state into an element's continuous block and mode block.
    pub fn write_blocks(&self, state: &mut [f64], modes: &mut [f64]) {
        write1(state, 0, self.standoff_m);
        write1(state, 1, self.dwell_s);
        let code = (0u8..)
            .zip(PHASES)
            .find_map(|(k, phase)| (phase == self.phase).then_some(f64::from(k)));
        write1(modes, 0, code.unwrap_or(0.0));
    }
}

/// Where a docking port is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PortAnchor {
    /// A port fixed to a component of the model, sharing its axes.
    Link {
        /// Component carrying the port.
        component: ComponentId,
        /// Port offset in the component's axes, m.
        offset_local_m: DVec3,
    },
    /// A port on another vehicle, whose motion the host holds over the step.
    World(PortKinematics),
}

/// A soft-capture ring between two ports as a force element. Its continuous
/// states are the stand-off and the settle dwell; its mode is the phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingCapture {
    /// Ring properties.
    pub spec: DockingSpec,
    /// Chaser port.
    pub port_a: PortAnchor,
    /// Target port.
    pub port_b: PortAnchor,
    /// Docking axis in port A's frame.
    pub axis_a_local: DVec3,
}

impl DockingCapture {
    fn port(anchor: &PortAnchor, kin: &Kinematics<'_>) -> Result<PortKinematics, MultibodyError> {
        match *anchor {
            PortAnchor::World(port) => Ok(port),
            PortAnchor::Link {
                component,
                offset_local_m,
            } => {
                let p = kin.point(component, offset_local_m)?;
                Ok(PortKinematics {
                    position_world_m: p.position_world_m,
                    rotation_world: p.rotation_quat(),
                    velocity_world_m_s: p.velocity_world_m_s,
                    angular_vel_world_rad_s: p.angular_vel_world_rad_s,
                })
            }
        }
    }

    fn apply(
        anchor: &PortAnchor,
        wrench_world: SpatialForce,
        kin: &Kinematics<'_>,
        loads: &mut Loads<'_>,
    ) -> Result<(), MultibodyError> {
        let PortAnchor::Link {
            component,
            offset_local_m,
        } = *anchor
        else {
            return Ok(());
        };
        let p = kin.point(component, offset_local_m)?;
        loads.add_wrench(&WrenchAtSite {
            site: component,
            point_local_m: offset_local_m,
            force_local_n: p.to_link(wrench_world.force),
            torque_local_nm: p.to_link(wrench_world.torque),
        })
    }
}

impl<I: ?Sized> ForceElement<I> for DockingCapture {
    fn state_len(&self) -> usize {
        2
    }

    fn mode_len(&self) -> usize {
        1
    }

    fn has_transitions(&self) -> bool {
        true
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        DockingState::default().write_blocks(state, modes);
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        let or = |v: f64, fallback: f64| if v > 0.0 { v } else { fallback };
        write1(scale, 0, or(self.spec.capture_radius_m, model.length_m()));
        write1(scale, 1, or(self.spec.damped_dwell_s, model.time_s()));
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let typed = DockingState::from_blocks(state, modes);
        let a = Self::port(&self.port_a, ctx.kinematics)?;
        let b = Self::port(&self.port_b, ctx.kinematics)?;
        let out = self.spec.evaluate(&typed, &a, &b, self.axis_a_local);
        write1(state_dot, 0, out.standoff_rate_m_s);
        write1(state_dot, 1, out.dwell_rate);
        Self::apply(&self.port_a, out.world_wrench_on_a, ctx.kinematics, loads)?;
        Self::apply(&self.port_b, out.world_wrench_on_b, ctx.kinematics, loads)
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let mut typed = DockingState::from_blocks(state, modes);
        let a = Self::port(&self.port_a, ctx.kinematics)?;
        let b = Self::port(&self.port_b, ctx.kinematics)?;
        let out = self.spec.transition(&mut typed, &a, &b, self.axis_a_local);
        typed.write_blocks(state, modes);
        Ok(out.changed)
    }
}
