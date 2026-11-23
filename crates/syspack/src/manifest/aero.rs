use serde::{Deserialize, Serialize};

mod builders;

/// One aerodynamic element mounted on a named component key in a multibody system.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AeroElementRecord {
    /// Component key in `SyspackManifest::components` owning this surface/body.
    pub site: String,
    /// Aerodynamic physics model attached to `site`.
    pub kind: AeroElementKindRecord,
}

/// Unified multi-regime aerodynamic element definitions corresponding 1-to-1 to [`mbd_aero`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum AeroElementKindRecord {
    /// Multi-regime blunt reentry capsule or aeroshell.
    BluntCapsule {
        /// Heat-shield base radius, m.
        base_radius_m: f64,
        /// Heat-shield nose radius, m.
        nose_radius_m: f64,
        /// Moment reference point in local component axes, m.
        reference_point_body_m: [f64; 3],
        /// Normal-force slope wrt sin(alpha) cos(alpha).
        normal_force_slope: f64,
        /// Newtonian axial drag coefficient CA0.
        axial_drag_coeff: f64,
        /// Pitch/yaw damping derivative Cmq.
        pitch_damping_coeff: f64,
        /// Optional override Sutton-Graves constant (`None` derives from live gas).
        #[serde(default)]
        sutton_graves_k: Option<f64>,
        /// Optional subsonic continuum drag coefficient.
        #[serde(default)]
        subsonic_drag_coeff: Option<f64>,
        /// Transonic wave-drag bump at Mach 1.05.
        #[serde(default)]
        transonic_drag_rise: f64,
        /// Optional free-molecular drag coefficient.
        #[serde(default)]
        free_molecular_drag_coeff: Option<f64>,
    },
    /// Slender rocket core / finned stage body.
    BodyAndFin {
        /// Reference cross-sectional area, m².
        ref_area_m2: f64,
        /// Reference length/diameter, m.
        ref_length_m: f64,
        /// Reference point in component axes, m.
        reference_point_body_m: [f64; 3],
        /// Nose stagnation radius, m.
        nose_radius_m: f64,
        /// Subsonic CD0.
        cd0_subsonic: f64,
        /// Supersonic CD0.
        cd0_supersonic: f64,
        /// Transonic drag peak increment.
        transonic_drag_peak: f64,
        /// Normal force slope CN_alpha.
        cn_alpha_body: f64,
        /// Pitch/yaw damping derivative.
        pitch_damping_coeff: f64,
        /// Roll damping derivative.
        roll_damping_coeff: f64,
        /// Controllable fin area, m².
        fin_area_m2: f64,
        /// Moment arm aft to fin hinge line, m.
        fin_arm_m: f64,
        /// Fin lift slope per rad.
        fin_cl_alpha: f64,
        /// Fin stall deflection clamp, rad.
        fin_stall_rad: f64,
        /// Optional control channel index for pitch fins.
        #[serde(default)]
        pitch_control_channel: Option<usize>,
        /// Optional control channel index for yaw fins.
        #[serde(default)]
        yaw_control_channel: Option<usize>,
        /// Optional control channel index for roll fins.
        #[serde(default)]
        roll_control_channel: Option<usize>,
    },
    /// Swept / variable-sweep wing, canard, tailplane, or vertical fin.
    WingSurface {
        /// Planform reference area, m².
        area_m2: f64,
        /// Mean aerodynamic chord, m.
        mean_chord_m: f64,
        /// Span, m.
        span_m: f64,
        /// Aspect ratio.
        aspect_ratio: f64,
        /// Baseline quarter-chord sweep angle, rad.
        sweep_quarter_chord_rad: f64,
        /// Optional control channel modulating sweep angle.
        #[serde(default)]
        sweep_control_channel: Option<usize>,
        /// Incompressible lift slope per rad.
        cl_alpha_incompressible: f64,
        /// Zero-AoA lift coefficient.
        cl0: f64,
        /// Maximum CL before stall.
        cl_max: f64,
        /// Stall angle of attack magnitude, rad.
        alpha_stall_rad: f64,
        /// Parasitic CD0.
        cd0: f64,
        /// Oswald span efficiency.
        oswald_efficiency: f64,
        /// Transonic wave-drag factor.
        wave_drag_factor: f64,
        /// Critical divergence Mach number.
        critical_mach: f64,
        /// Zero-lift pitching moment Cm0.
        cm0: f64,
        /// Aerodynamic center position in component axes, m.
        ac_position_body_m: [f64; 3],
        /// Unit chordwise forward axis in component frame.
        chord_direction_body: [f64; 3],
        /// Unit positive-lift normal in component frame.
        lift_normal_body: [f64; 3],
        /// Optional flap control channel.
        #[serde(default)]
        flap_channel: Option<usize>,
        /// Flap lift increment per rad.
        #[serde(default)]
        flap_dcl: f64,
        /// Flap drag increment per rad.
        #[serde(default)]
        flap_dcd: f64,
        /// Optional aileron/differential channel.
        #[serde(default)]
        aileron_channel: Option<usize>,
        /// Aileron lift increment per rad.
        #[serde(default)]
        aileron_dcl: f64,
    },
    /// Blade-element momentum rotor or propeller disk.
    RotorBem {
        /// Outer radius, m.
        radius_m: f64,
        /// Hub cutout radius, m.
        hub_radius_m: f64,
        /// Blade count.
        blade_count: u8,
        /// Mean chord, m.
        chord_m: f64,
        /// Root collective pitch twist, rad.
        root_pitch_rad: f64,
        /// Tip collective pitch twist, rad.
        tip_pitch_rad: f64,
        /// Section lift slope per rad.
        cl_alpha: f64,
        /// Section max CL.
        cl_max: f64,
        /// High-Reynolds section CD0.
        cd0_high_re: f64,
        /// Low-Reynolds viscous penalty factor.
        low_re_drag_factor: f64,
        /// Critical blade-tip Mach number.
        critical_tip_mach: f64,
        /// Coaxial interference factor.
        coaxial_interference_factor: f64,
        /// Spin direction sign (+1 or -1).
        rotation_direction: f64,
        /// Hub center position in component frame, m.
        hub_position_body_m: [f64; 3],
        /// Unit shaft thrust axis in component frame.
        shaft_axis_body: [f64; 3],
        /// Radial blade stations (`4..=32`).
        radial_stations: u8,
        /// Baseline rotor spin speed, rad/s.
        #[serde(default)]
        initial_omega_rad_s: f64,
        /// Initial collective offset, rad.
        #[serde(default)]
        initial_collective_rad: f64,
    },
    /// Free-molecular Sentman thermospheric / VLEO panel.
    RarefiedPanel {
        /// Exposed panel area, m².
        area_m2: f64,
        /// Outward unit normal in component axes.
        normal_body: [f64; 3],
        /// Pressure center in component axes, m.
        center_body_m: [f64; 3],
        /// Whether both sides are exposed to flow.
        double_sided: bool,
        /// Wall temperature, K.
        surface_temperature_k: f64,
        /// Normal thermal accommodation coefficient `[0, 1]`.
        energy_accommodation: f64,
        /// Tangential momentum accommodation `[0, 1]`.
        tangential_accommodation: f64,
    },
    /// Subsonic / supersonic disk-gap-band (DGB) parachute canopy.
    ParachuteCanopy {
        /// Nominal canopy reference area, m².
        nominal_area_m2: f64,
        /// Subsonic CD.
        cd_subsonic: f64,
        /// Supersonic CD.
        cd_supersonic: f64,
        /// Restoring normal-force slope.
        cn_alpha: f64,
        /// Riser attach point in component axes, m.
        riser_attach_body_m: [f64; 3],
        /// Unit canopy axis pointing aft toward skirt.
        canopy_axis_body: [f64; 3],
        /// Filling time constant, s.
        inflation_time_constant_s: f64,
        /// Forebody wake velocity deficit multiplier `(0, 1]`.
        wake_velocity_ratio: f64,
        /// Rotational damping coeff.
        rotational_damping_coeff: f64,
    },
    /// Ejected dummy / crew occupant drag and damping model.
    HumanDummy {
        /// Frontal reference area, m².
        frontal_area_m2: f64,
        /// Characteristic length, m.
        reference_length_m: f64,
        /// Bluff drag coefficient.
        drag_coeff: f64,
        /// Tumbling rotational damping coeff.
        damping_coeff: f64,
        /// Reference point in local axes, m.
        reference_point_body_m: [f64; 3],
    },
}
