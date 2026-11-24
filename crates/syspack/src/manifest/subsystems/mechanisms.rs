use serde::{Deserialize, Serialize};

/// Discrete or compliant actuation mechanism attached to a component key.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MechanismRecord {
    /// Unique mechanism identifier within this package.
    pub id: String,
    /// Host component key owning the mechanism.
    pub site: String,
    /// Specific mechanism family and parameters.
    pub kind: MechanismKindRecord,
}

/// Physical mechanism definitions aligned with `mbd-multibody::mechanism`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum MechanismKindRecord {
    /// One-shot pyrotechnic / spring stage or strap separation impulse.
    ImpulseRelease {
        /// Release impulse magnitude, N·s.
        impulse_ns: f64,
        /// Unit separation impulse direction in `site` local axes.
        direction_local: [f64; 3],
        /// Optional child component key separated when fired.
        #[serde(default)]
        target_child: Option<String>,
    },
    /// Guided rail deployment / ejection seat catapult rail.
    RailSlide {
        /// Unit rail travel direction in `site` local axes.
        rail_axis_local: [f64; 3],
        /// Total guide length before free exit, m.
        rail_length_m: f64,
        /// Active drive force along rail, N.
        drive_force_n: f64,
        /// Sliding friction coefficient.
        friction_coeff: f64,
    },
    /// Spring/torque-driven fairing or SLA petal hinge swing release.
    HingeSwing {
        /// Hinge axis in `site` local frame.
        hinge_axis_local: [f64; 3],
        /// Actuator opening torque, N·m.
        torque_nm: f64,
        /// Opening angle at which hinge unlatches/releases, rad.
        release_angle_rad: f64,
    },
    /// Androgynous or probe-drogue docking port specification.
    DockingPort {
        /// Port center offset in `site` local axes, m.
        port_offset_local_m: [f64; 3],
        /// Unit outward port normal in `site` local axes.
        outward_normal_local: [f64; 3],
        /// Soft-capture radial tolerance, m.
        max_radial_m: f64,
        /// Soft-capture axial gap window, m.
        max_axial_gap_m: f64,
        /// Maximum port axis angular misalignment, rad.
        max_angle_rad: f64,
        /// Maximum relative closing speed, m/s.
        max_speed_m_s: f64,
        /// Undocking push-off impulse, N·s.
        undock_impulse_ns: f64,
    },
    /// Passive grapple fixture (e.g., FRGF on HST) or capture target point.
    GrappleFixture {
        /// Grapple pin offset in `site` local frame, m.
        offset_local_m: [f64; 3],
        /// Unit grapple axis in `site` local frame.
        axis_local: [f64; 3],
    },
    /// Flexible tether or Sky Crane winch cable assembly.
    TetherWinch {
        /// Anchor point on `site` in local axes, m.
        anchor_local_m: [f64; 3],
        /// Initial paid-out tether length, m.
        initial_length_m: f64,
        /// Maximum tether length, m.
        max_length_m: f64,
        /// Cable axial stiffness, N/m.
        spring_k_n_m: f64,
        /// Cable axial damping, N·s/m.
        damper_c_n_s_m: f64,
        /// Peak tensile strength before snap/release, N.
        max_tension_n: f64,
        /// Optional thermal burn-through cut temperature, K (e.g., Vostok straps).
        #[serde(default)]
        melt_temp_k: Option<f64>,
        /// Automatic cutter release upon payload touchdown (e.g., Mars Sky Crane).
        #[serde(default)]
        auto_cut_on_touchdown: bool,
    },
}

/// Multi-DoF robotic manipulator arm definition mapped onto `ComponentTree` joints.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ManipulatorRecord {
    /// Human-readable manipulator identifier (e.g. `"canadarm"`, `"perseverance_arm"`).
    pub name: String,
    /// Base mounting component key.
    pub base_site: String,
    /// Ordered list of articulated joint link component keys from base to wrist.
    pub joint_sites: Vec<String>,
    /// End-effector tool tip offset in the final wrist link frame, m.
    pub end_effector_offset_m: [f64; 3],
}

/// Driven/braked/steered planetary rover or landing gear wheel specification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WheelRecord {
    /// Hub or bogie component key owning this wheel.
    pub site: String,
    /// Wheel hub contact offset in `site` local axes, m.
    pub hub_offset_local_m: [f64; 3],
    /// Unit spin axle in `site` local axes.
    pub spin_axis_local: [f64; 3],
    /// Unit steer axis in `site` local axes.
    pub steer_axis_local: [f64; 3],
    /// Tire / wheel radius, m.
    pub radius_m: f64,
    /// Maximum drive motor torque, N·m.
    pub max_drive_torque_nm: f64,
    /// Maximum brake torque, N·m.
    pub max_brake_torque_nm: f64,
    /// Tire cornering stiffness, N/rad.
    pub cornering_stiffness_n_rad: f64,
}
