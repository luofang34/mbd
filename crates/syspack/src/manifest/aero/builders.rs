use glam::DVec3;
use mbd_aero::{
    AeroElement, BluntCapsuleAero, BluntCapsuleParams, BodyAndFinAero, BodyAndFinParams,
    HumanDummyAero, HumanDummyParams, ParachuteCanopyAero, ParachuteParams, ParachuteState,
    RarefiedPanelAero, RarefiedPanelParams, RotorBemAero, RotorBemParams, RotorState,
    WingSurfaceAero, WingSurfaceParams,
};

use super::AeroElementKindRecord;
use crate::SyspackError;

impl AeroElementKindRecord {
    /// Builds and validates the corresponding [`mbd_aero::AeroElement`].
    pub fn build(&self) -> Result<AeroElement, SyspackError> {
        match *self {
            Self::BluntCapsule { .. } | Self::BodyAndFin { .. } => self.build_capsule_or_body(),
            Self::WingSurface { .. } => self.build_wing_surface(),
            Self::RotorBem { .. } => self.build_rotor_bem(),
            Self::RarefiedPanel { .. } | Self::ParachuteCanopy { .. } | Self::HumanDummy { .. } => {
                self.build_rarefied_chute_or_dummy()
            }
        }
    }

    fn build_capsule_or_body(&self) -> Result<AeroElement, SyspackError> {
        match *self {
            Self::BluntCapsule {
                base_radius_m,
                nose_radius_m,
                reference_point_body_m,
                normal_force_slope,
                axial_drag_coeff,
                pitch_damping_coeff,
                sutton_graves_k,
                subsonic_drag_coeff,
                transonic_drag_rise,
                free_molecular_drag_coeff,
            } => Ok(AeroElement::BluntCapsule(
                BluntCapsuleAero::new(BluntCapsuleParams {
                    base_radius_m,
                    nose_radius_m,
                    reference_point_body_m: DVec3::from_array(reference_point_body_m),
                    normal_force_slope,
                    axial_drag_coeff,
                    pitch_damping_coeff,
                    sutton_graves_k,
                    subsonic_drag_coeff,
                    transonic_drag_rise,
                    free_molecular_drag_coeff,
                })
                .map_err(SyspackError::InvalidAero)?,
            )),
            Self::BodyAndFin {
                ref_area_m2,
                ref_length_m,
                reference_point_body_m,
                nose_radius_m,
                cd0_subsonic,
                cd0_supersonic,
                transonic_drag_peak,
                cn_alpha_body,
                pitch_damping_coeff,
                roll_damping_coeff,
                fin_area_m2,
                fin_arm_m,
                fin_cl_alpha,
                fin_stall_rad,
                pitch_control_channel,
                yaw_control_channel,
                roll_control_channel,
            } => Ok(AeroElement::BodyAndFin(
                BodyAndFinAero::new(BodyAndFinParams {
                    ref_area_m2,
                    ref_length_m,
                    reference_point_body_m: DVec3::from_array(reference_point_body_m),
                    nose_radius_m,
                    cd0_subsonic,
                    cd0_supersonic,
                    transonic_drag_peak,
                    cn_alpha_body,
                    pitch_damping_coeff,
                    roll_damping_coeff,
                    fin_area_m2,
                    fin_arm_m,
                    fin_cl_alpha,
                    fin_stall_rad,
                    pitch_control_channel,
                    yaw_control_channel,
                    roll_control_channel,
                })
                .map_err(SyspackError::InvalidAero)?,
            )),
            _ => Err(SyspackError::InvalidSubsystem {
                detail: "internal dispatch mismatch in aero builder".into(),
            }),
        }
    }

    fn build_wing_surface(&self) -> Result<AeroElement, SyspackError> {
        let Self::WingSurface {
            area_m2,
            mean_chord_m,
            span_m,
            aspect_ratio,
            sweep_quarter_chord_rad,
            sweep_control_channel,
            cl_alpha_incompressible,
            cl0,
            cl_max,
            alpha_stall_rad,
            cd0,
            oswald_efficiency,
            wave_drag_factor,
            critical_mach,
            cm0,
            ac_position_body_m,
            chord_direction_body,
            lift_normal_body,
            flap_channel,
            flap_dcl,
            flap_dcd,
            aileron_channel,
            aileron_dcl,
        } = *self
        else {
            return Err(SyspackError::InvalidSubsystem {
                detail: "internal dispatch mismatch in wing builder".into(),
            });
        };
        Ok(AeroElement::WingSurface(
            WingSurfaceAero::new(WingSurfaceParams {
                area_m2,
                mean_chord_m,
                span_m,
                aspect_ratio,
                sweep_quarter_chord_rad,
                sweep_control_channel,
                cl_alpha_incompressible,
                cl0,
                cl_max,
                alpha_stall_rad,
                cd0,
                oswald_efficiency,
                wave_drag_factor,
                critical_mach,
                cm0,
                ac_position_body_m: DVec3::from_array(ac_position_body_m),
                chord_direction_body: DVec3::from_array(chord_direction_body),
                lift_normal_body: DVec3::from_array(lift_normal_body),
                flap_channel,
                flap_dcl,
                flap_dcd,
                aileron_channel,
                aileron_dcl,
            })
            .map_err(SyspackError::InvalidAero)?,
        ))
    }

    fn build_rotor_bem(&self) -> Result<AeroElement, SyspackError> {
        let Self::RotorBem {
            radius_m,
            hub_radius_m,
            blade_count,
            chord_m,
            root_pitch_rad,
            tip_pitch_rad,
            cl_alpha,
            cl_max,
            cd0_high_re,
            low_re_drag_factor,
            critical_tip_mach,
            coaxial_interference_factor,
            rotation_direction,
            hub_position_body_m,
            shaft_axis_body,
            radial_stations,
            initial_omega_rad_s,
            initial_collective_rad,
        } = *self
        else {
            return Err(SyspackError::InvalidSubsystem {
                detail: "internal dispatch mismatch in rotor builder".into(),
            });
        };
        Ok(AeroElement::RotorBem(
            RotorBemAero::new(
                RotorBemParams {
                    radius_m,
                    hub_radius_m,
                    blade_count,
                    chord_m,
                    root_pitch_rad,
                    tip_pitch_rad,
                    cl_alpha,
                    cl_max,
                    cd0_high_re,
                    low_re_drag_factor,
                    critical_tip_mach,
                    coaxial_interference_factor,
                    rotation_direction,
                    hub_position_body_m: DVec3::from_array(hub_position_body_m),
                    shaft_axis_body: DVec3::from_array(shaft_axis_body),
                    radial_stations,
                },
                RotorState {
                    omega_rad_s: initial_omega_rad_s,
                    collective_rad: initial_collective_rad,
                    cyclic_lat_rad: 0.0,
                    cyclic_lon_rad: 0.0,
                },
            )
            .map_err(SyspackError::InvalidAero)?,
        ))
    }

    fn build_rarefied_chute_or_dummy(&self) -> Result<AeroElement, SyspackError> {
        match *self {
            Self::RarefiedPanel {
                area_m2,
                normal_body,
                center_body_m,
                double_sided,
                surface_temperature_k,
                energy_accommodation,
                tangential_accommodation,
            } => Ok(AeroElement::RarefiedPanel(
                RarefiedPanelAero::new(RarefiedPanelParams {
                    area_m2,
                    normal_body: DVec3::from_array(normal_body),
                    center_body_m: DVec3::from_array(center_body_m),
                    double_sided,
                    surface_temperature_k,
                    energy_accommodation,
                    tangential_accommodation,
                })
                .map_err(SyspackError::InvalidAero)?,
            )),
            Self::ParachuteCanopy {
                nominal_area_m2,
                cd_subsonic,
                cd_supersonic,
                cn_alpha,
                riser_attach_body_m,
                canopy_axis_body,
                inflation_time_constant_s,
                wake_velocity_ratio,
                rotational_damping_coeff,
            } => Ok(AeroElement::ParachuteCanopy(
                ParachuteCanopyAero::new(
                    ParachuteParams {
                        nominal_area_m2,
                        cd_subsonic,
                        cd_supersonic,
                        cn_alpha,
                        riser_attach_body_m: DVec3::from_array(riser_attach_body_m),
                        canopy_axis_body: DVec3::from_array(canopy_axis_body),
                        inflation_time_constant_s,
                        wake_velocity_ratio,
                        rotational_damping_coeff,
                    },
                    ParachuteState::default(),
                )
                .map_err(SyspackError::InvalidAero)?,
            )),
            Self::HumanDummy {
                frontal_area_m2,
                reference_length_m,
                drag_coeff,
                damping_coeff,
                reference_point_body_m,
            } => Ok(AeroElement::HumanDummy(
                HumanDummyAero::new(HumanDummyParams {
                    frontal_area_m2,
                    reference_length_m,
                    drag_coeff,
                    damping_coeff,
                    reference_point_body_m: DVec3::from_array(reference_point_body_m),
                })
                .map_err(SyspackError::InvalidAero)?,
            )),
            _ => Err(SyspackError::InvalidSubsystem {
                detail: "internal dispatch mismatch in rarefied builder".into(),
            }),
        }
    }
}
