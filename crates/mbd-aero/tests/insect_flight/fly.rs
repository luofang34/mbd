//! A fruit-fly-like vehicle built from published *Drosophila* morphology and
//! hovering kinematics: a body and two wings, each wing on a stroke hinge
//! and a pitch hinge, driven along prescribed angles by a joint servo.
//!
//! Sources for every number that is not a modelling choice:
//! Fry, Sayaman & Dickinson 2005, J. Exp. Biol. 208:2303, Table 1 (free
//! hovering means): wingbeat frequency 218 Hz, stroke amplitude 140°, wing
//! length 2.39 mm; their morphometric regressions (p. 2306) for body mass
//! `M_b = −2.796·10⁻⁷ + 4.982·10⁻⁴·R` kg and wing area
//! `S = −2.023·10⁻⁶ + 1.748·10⁻³·R` m², and `r̂₂²(S) = 0.35`.

use std::f64::consts::PI;

use glam::{DMat3, DVec3};
use mbd_aero::{
    AeroElement, AeroElementKind, AeroEnvironment, AeroInputs, AirSample, FlappingWingAero,
    FlappingWingParams,
};
use mbd_core::{
    AmbientGas, ComponentId, ComponentSpec, ComponentState, ComponentTree, DynamicMassContribution,
    JointSpec, Placement,
};
use mbd_multibody::{
    ElementContext, ForceElement, JointLoadInput, Loads, MultibodyError, SpatialMat6,
    SurfaceMedium, SurfaceQuery, SurfaceSample,
};

/// Blade elements per wing.
const STRIPS: usize = 20;
/// Servo bandwidth as a multiple of the wingbeat angular frequency: well
/// above the motion it tracks, well inside RK4 stability at 100 steps per
/// beat (`ω_n h ≈ 0.63`).
const SERVO_BANDWIDTH: f64 = 10.0;

/// Body, right stroke link, right wing, left stroke link, left wing.
pub const BODY: ComponentId = ComponentId(0);
pub const WINGS: [ComponentId; 2] = [ComponentId(2), ComponentId(4)];
const STROKES: [ComponentId; 2] = [ComponentId(1), ComponentId(3)];

/// Everything that defines one flapping fly at one size.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlySpec {
    pub wing_length_m: f64,
    pub frequency_hz: f64,
    /// Peak-to-peak stroke angle Φ, rad.
    pub stroke_amplitude_rad: f64,
    /// Wing pitch from vertical at mid-stroke: the mid-stroke angle of
    /// attack is `π/2` minus this.
    pub pitch_amplitude_rad: f64,
    /// Lead of wing rotation ahead of stroke reversal, rad of wingbeat
    /// phase; zero is symmetric rotation.
    pub rotation_lead_rad: f64,
    pub body_mass_kg: f64,
    pub wing_mass_kg: f64,
    pub wing_area_m2: f64,
    pub air_density_kg_m3: f64,
    pub air_viscosity_pa_s: f64,
    pub gravity_m_s2: f64,
    /// Chord fraction of the pitch axis behind the leading edge.
    pub pitch_axis_chord_fraction: f64,
}

impl FlySpec {
    /// Hovering *Drosophila* (sources in the module comment). Modelling
    /// choices: wing mass 2.4 µg, within the 1–10 µg range of the task; a
    /// horizontal stroke plane; sinusoidal stroke and pitch; a 45°
    /// mid-stroke angle of attack; the pitch axis at the quarter chord.
    pub fn drosophila() -> Self {
        let r = 2.39e-3;
        let air = AmbientGas::earth_air(101_325.0, 293.15).unwrap();
        Self {
            wing_length_m: r,
            frequency_hz: 218.0,
            stroke_amplitude_rad: 140f64.to_radians(),
            pitch_amplitude_rad: 45f64.to_radians(),
            rotation_lead_rad: 0.0,
            body_mass_kg: -2.796e-7 + 4.982e-4 * r,
            wing_mass_kg: 2.4e-9,
            wing_area_m2: -2.023e-6 + 1.748e-3 * r,
            air_density_kg_m3: air.density_kg_m3,
            air_viscosity_pa_s: air.dynamic_viscosity_pa_s,
            gravity_m_s2: 9.81,
            pitch_axis_chord_fraction: 0.25,
        }
    }

    /// The same nondimensional system `k` times longer: frequency `f/k`
    /// keeps wing speeds, density is unchanged so masses scale by `k³`,
    /// viscosity by `k` keeps the Reynolds number and gravity by `1/k`
    /// keeps `g/(f²R)`.
    pub fn scaled(&self, k: f64) -> Self {
        Self {
            wing_length_m: self.wing_length_m * k,
            frequency_hz: self.frequency_hz / k,
            body_mass_kg: self.body_mass_kg * k * k * k,
            wing_mass_kg: self.wing_mass_kg * k * k * k,
            wing_area_m2: self.wing_area_m2 * k * k,
            air_viscosity_pa_s: self.air_viscosity_pa_s * k,
            gravity_m_s2: self.gravity_m_s2 / k,
            ..*self
        }
    }

    pub fn period_s(&self) -> f64 {
        1.0 / self.frequency_hz
    }

    pub fn total_mass_kg(&self) -> f64 {
        self.body_mass_kg + 2.0 * self.wing_mass_kg
    }

    pub fn weight_n(&self) -> f64 {
        self.total_mass_kg() * self.gravity_m_s2
    }

    pub fn ambient(&self) -> AmbientGas {
        let mut air = AmbientGas::earth_air(101_325.0, 293.15).unwrap();
        air.density_kg_m3 = self.air_density_kg_m3;
        air.dynamic_viscosity_pa_s = self.air_viscosity_pa_s;
        air
    }

    /// Chords of a wing whose shape is the beta profile
    /// `c ∝ r̂^0.75 (1 − r̂)^0.5`, scaled to the wing area. Its second
    /// moment of area is `r̂₂² = (1.75·2.75)/(3.25·4.25) = 0.348`, the
    /// *Drosophila* value 0.35 of Fry et al. (2005).
    pub fn chords_m(&self) -> Vec<f64> {
        let shape: Vec<f64> = (0..STRIPS)
            .map(|k| {
                let r = (k as f64 + 0.5) / STRIPS as f64;
                r.powf(0.75) * (1.0 - r).sqrt()
            })
            .collect();
        let width = self.wing_length_m / STRIPS as f64;
        let scale = self.wing_area_m2 / (shape.iter().sum::<f64>() * width);
        shape.iter().map(|s| s * scale).collect()
    }
}

/// Mass properties of one link: mass, centre of mass and inertia about it.
#[derive(Debug, Clone, Copy)]
pub struct LinkMass {
    pub id: ComponentId,
    pub mass_kg: f64,
    pub cg_m: DVec3,
    pub inertia_cg: DMat3,
}

/// The fly: its tree, its aerodynamic elements and its mass table.
pub struct Fly {
    pub spec: FlySpec,
    pub tree: ComponentTree,
    /// Wing masses, riding on the wing links.
    pub dynamic_masses: Vec<DynamicMassContribution>,
    pub wings: Vec<AeroElement>,
    pub masses: Vec<LinkMass>,
    pub servo: WingServo,
}

/// Span direction of each wing in its link axes; the chord points up
/// (`+z`) at zero pitch, so the plate faces the stroke direction.
const SPAN: [DVec3; 2] = [DVec3::NEG_Y, DVec3::Y];

impl Fly {
    pub fn new(spec: FlySpec) -> Self {
        let r = spec.wing_length_m;
        let chords = spec.chords_m();
        let wing = |span: DVec3| {
            FlappingWingAero::new(FlappingWingParams {
                root_link_m: DVec3::ZERO,
                span_axis_link: span,
                chord_axis_link: DVec3::Z,
                length_m: r,
                chords_m: chords.clone(),
                rotation_axis_chord_fraction: spec.pitch_axis_chord_fraction,
                center_of_pressure_chord_fraction: 0.25,
            })
            .unwrap()
        };
        let wing_masses = SPAN.map(|span| plate_mass(&spec, &chords, span));
        let body = LinkMass {
            id: BODY,
            mass_kg: spec.body_mass_kg,
            cg_m: DVec3::ZERO,
            inertia_cg: ellipsoid(spec.body_mass_kg, 0.52 * r, 0.17 * r),
        };
        let tree = build_tree(&spec, &body);
        let wing_masses: [LinkMass; 2] = [0, 1].map(|k| LinkMass {
            id: WINGS[k],
            ..wing_masses[k]
        });
        let dynamic_masses = wing_masses
            .iter()
            .map(|w| DynamicMassContribution {
                component: w.id,
                mass_kg: w.mass_kg,
                cg_local_m: w.cg_m,
                inertia_cg_local: w.inertia_cg,
            })
            .collect();
        let wings: Vec<AeroElement> = WINGS
            .iter()
            .zip(SPAN)
            .map(|(&id, span)| AeroElement::new(id, AeroElementKind::FlappingWing(wing(span))))
            .collect();
        let added = SPAN.map(|span| {
            wing(span)
                .evaluate(&still_flow(&spec))
                .unwrap()
                .added_inertia
        });
        let rigid = [0, 1].map(|k| {
            SpatialMat6::from_rigid_inertia(
                spec.wing_mass_kg,
                wing_masses[k].cg_m,
                wing_masses[k].inertia_cg,
            )
        });
        let pitch_inertia = body.inertia_cg.y_axis.y
            + 2.0 * spec.wing_mass_kg * wing_masses[0].cg_m.length_squared();
        let servo = WingServo::new(spec, rigid, added).stabilised(pitch_inertia);
        let mut masses = vec![body];
        masses.extend(wing_masses);
        Self {
            spec,
            tree,
            dynamic_masses,
            wings,
            masses,
            servo,
        }
    }
}

fn still_flow(spec: &FlySpec) -> mbd_aero::LinkFlow<'static> {
    mbd_aero::LinkFlow {
        velocity_air_link_m_s: DVec3::ZERO,
        angular_rate_link_rad_s: DVec3::ZERO,
        ambient: spec.ambient(),
        controls: &[],
    }
}

/// Solid prolate ellipsoid along `x` with semi-axes `a`, `b`, `b`.
fn ellipsoid(m: f64, a: f64, b: f64) -> DMat3 {
    DMat3::from_diagonal(DVec3::new(2.0 * b * b, a * a + b * b, a * a + b * b) * (m / 5.0))
}

/// Uniform thin plate over the blade elements, in the wing link at zero
/// pitch: span along `span`, chord along `z`, the pitch axis through the
/// link origin.
fn plate_mass(spec: &FlySpec, chords: &[f64], span: DVec3) -> LinkMass {
    let width = spec.wing_length_m / chords.len() as f64;
    let density = spec.wing_mass_kg / spec.wing_area_m2;
    let x0 = spec.pitch_axis_chord_fraction;
    let parts: Vec<(f64, DVec3, f64)> = chords
        .iter()
        .enumerate()
        .map(|(k, &c)| {
            let at = span * ((k as f64 + 0.5) * width) + DVec3::Z * ((x0 - 0.5) * c);
            (density * c * width, at, c)
        })
        .collect();
    let cg = parts.iter().map(|(m, p, _)| *p * *m).sum::<DVec3>() / spec.wing_mass_kg;
    let inertia = parts.iter().fold(DMat3::ZERO, |acc, &(m, p, c)| {
        let d = p - cg;
        let own = DMat3::from_diagonal(DVec3::new(c * c + width * width, c * c, width * width))
            * (m / 12.0);
        acc + own + (DMat3::IDENTITY * d.length_squared() - outer(d, d)) * m
    });
    LinkMass {
        id: ComponentId(0),
        mass_kg: spec.wing_mass_kg,
        cg_m: cg,
        inertia_cg: inertia,
    }
}

fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

/// Body at the root; each wing hangs from a massless stroke link on a
/// vertical hinge above and beside the body's centre of mass. Wing mass
/// rides as a distributed mass so its centre sits off the hinge axes.
fn build_tree(spec: &FlySpec, body: &LinkMass) -> ComponentTree {
    let r = spec.wing_length_m;
    let part = |placement, mass_kg, inertia, joint| ComponentSpec {
        name: "fly".to_owned(),
        placement,
        dry_mass_kg: mass_kg,
        dry_inertia_body: inertia,
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint,
    };
    let free = |axis| JointSpec::revolute(axis, -PI, PI);
    let idle = ComponentState::at_temperature(293.15);
    let root = part(
        Placement::ROOT,
        body.mass_kg,
        body.inertia_cg,
        JointSpec::FIXED,
    );
    let mut specs = vec![(root, idle)];
    for (side, span) in SPAN.iter().enumerate() {
        let hinge = DVec3::new(0.0, span.y * 0.15 * r, 0.125 * r);
        let stroke = ComponentId(1 + 2 * side as u32);
        let at_hinge = Placement::child_of(BODY, hinge);
        specs.push((part(at_hinge, 0.0, DMat3::ZERO, free(DVec3::Z)), idle));
        let on_stroke = Placement::child_of(stroke, DVec3::ZERO);
        specs.push((part(on_stroke, 0.0, DMat3::ZERO, free(*span)), idle));
    }
    ComponentTree::new(specs).unwrap()
}

/// Air everywhere plus, optionally, a flat floor at `z = 0`.
#[derive(Debug, Clone, Copy)]
pub struct FlyWorld {
    pub env: AeroEnvironment<'static>,
    pub floor: SurfaceSample,
}

impl FlyWorld {
    pub fn new(spec: &FlySpec) -> Self {
        Self {
            env: AeroEnvironment {
                air: AirSample {
                    ambient: spec.ambient(),
                    wind_world_m_s: DVec3::ZERO,
                },
                controls: &[],
            },
            floor: SurfaceSample {
                surface_point_world_m: DVec3::ZERO,
                outward_normal_world: DVec3::Z,
                medium: SurfaceMedium::Solid,
                mu_static_override: None,
                mu_dynamic_override: None,
            },
        }
    }
}

impl AeroInputs for FlyWorld {
    fn air_at(&self, position_world_m: DVec3) -> AirSample {
        self.env.air_at(position_world_m)
    }

    fn controls(&self) -> &[f64] {
        self.env.controls()
    }
}

impl SurfaceQuery for FlyWorld {
    fn surface_at(&self, point_world_m: DVec3) -> SurfaceSample {
        self.floor.surface_at(point_world_m)
    }
}

/// Prescribed wing angles at time `t`: `(stroke, pitch)` angle and rate of
/// the right wing; the left wing mirrors them.
pub fn wing_angles(spec: &FlySpec, t: f64) -> [(f64, f64, f64); 2] {
    let w = 2.0 * PI * spec.frequency_hz;
    let half = 0.5 * spec.stroke_amplitude_rad;
    let (s, c) = (w * t).sin_cos();
    let (ps, pc) = (w * t + spec.rotation_lead_rad).sin_cos();
    let psi0 = spec.pitch_amplitude_rad;
    [
        (half * c, -half * w * s, -half * w * w * c),
        (psi0 * ps, psi0 * w * pc, -psi0 * w * w * ps),
    ]
}

/// Joint servo tracking the prescribed angles with critically damped
/// feedback. Each wing's stroke and pitch commands are multiplied by the
/// wing's 2×2 joint inertia at its current pitch, so the two coupled
/// hinges respond at the servo bandwidth in every direction. The loads are
/// unlimited joint loads, equal and opposite between wing and body.
#[derive(Debug, Clone, Copy)]
pub struct WingServo {
    spec: FlySpec,
    /// Rotational inertia of each wing about its hinge point, wing axes:
    /// the rigid wing alone and the entrained air alone.
    rigid: [DMat3; 2],
    added: [DMat3; 2],
    /// Include the entrained air in the joint inertia.
    pub air: bool,
    /// Hold the wings still at zero angles instead of flapping.
    pub hold: bool,
    /// Body pitch feedback: proportional (rad of stroke per rad of pitch)
    /// and derivative (rad of stroke per rad/s) gains; `None` flies open
    /// loop.
    pub pitch_gains: Option<(f64, f64)>,
}

impl WingServo {
    /// Pitch gains that place the body's pitch response at a tenth of the
    /// wingbeat frequency, critically damped: a stand-in for the pitch
    /// reflex of a real fly, which an uncontrolled hovering body lacks.
    /// Shifting the stroke by `δ` moves the lift, about the weight, fore or
    /// aft by `r₂ δ`, so the moment per radian is `W r₂` against the body's
    /// pitch inertia. The gains are this test's design, not measured.
    pub fn stabilised(mut self, body_pitch_inertia: f64) -> Self {
        let spec = &self.spec;
        let r2 = 0.59 * spec.wing_length_m;
        let authority = spec.weight_n() * r2 / body_pitch_inertia;
        let wc = 2.0 * PI * spec.frequency_hz / 10.0;
        self.pitch_gains = Some((wc * wc / authority, 2.0 * wc / authority));
        self
    }

    fn new(spec: FlySpec, rigid: [SpatialMat6; 2], added: [SpatialMat6; 2]) -> Self {
        Self {
            spec,
            rigid: rigid.map(|m| m.aa),
            added: added.map(|m| m.aa),
            air: true,
            hold: false,
            pitch_gains: None,
        }
    }

    /// Joint inertia of wing `side` at pitch `psi` about the stroke axis
    /// `z` and its pitch axis.
    fn joint_inertia(&self, side: usize, psi: f64) -> [[f64; 2]; 2] {
        let span = SPAN[side];
        let wing = if self.air {
            self.rigid[side] + self.added[side]
        } else {
            self.rigid[side]
        };
        let r = DMat3::from_quat(glam::DQuat::from_axis_angle(span, psi));
        let i = r * wing * r.transpose();
        let (z, a) = (DVec3::Z, span);
        [[z.dot(i * z), z.dot(i * a)], [a.dot(i * z), a.dot(i * a)]]
    }
}

/// The body rocks in pitch at the wingbeat frequency; feeding that back
/// would shrink the stroke. The pitch reflex sees the body pitch through
/// two first-order lags of half a wingbeat, which pass the slow divergence
/// and cut the rocking tenfold.
const PITCH_FILTER_BEATS: f64 = 0.5;

impl<I: ?Sized> ForceElement<I> for WingServo {
    /// Two filtered pitch angles, rad.
    fn state_len(&self) -> usize {
        2
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let wn = SERVO_BANDWIDTH * 2.0 * PI * self.spec.frequency_hz;
        let mut target = wing_angles(&self.spec, ctx.time_s);
        // Nose-down pitch is positive about y; shifting both strokes forward
        // moves the lift ahead of the centre of mass and pitches the nose
        // back up.
        let nose = ctx.kinematics.base().rotation_world_from_root * DVec3::X;
        let pitch = (-nose.z).atan2(nose.x);
        let tau = PITCH_FILTER_BEATS * self.spec.period_s();
        let (x1, x2) = (state[0], state[1]);
        state_dot[0] = (pitch - x1) / tau;
        state_dot[1] = (x1 - x2) / tau;
        if let Some((kp, kd)) = self.pitch_gains {
            target[0].0 += kp * x2 + kd * state_dot[1];
        }
        for side in 0..2 {
            // The left wing mirrors the right.
            let mirror = if side == 0 { 1.0 } else { -1.0 };
            let ids = [STROKES[side], WINGS[side]];
            let mut accel = [0.0; 2];
            for (dof, &id) in ids.iter().enumerate() {
                let (q_d, v_d, a_d) = if self.hold {
                    (0.0, 0.0, 0.0)
                } else {
                    target[dof]
                };
                let j = ctx.kinematics.joint(id)?;
                accel[dof] = mirror * a_d
                    + 2.0 * wn * (mirror * v_d - j.q_vel_scalar)
                    + wn * wn * (mirror * q_d - j.q_scalar);
            }
            let psi = ctx.kinematics.joint(WINGS[side])?.q_scalar;
            let m = self.joint_inertia(side, psi);
            for (dof, &id) in ids.iter().enumerate() {
                let effort = m[dof][0] * accel[0] + m[dof][1] * accel[1];
                loads.add_joint_load(&JointLoadInput::scalar(id, effort))?;
            }
        }
        Ok(())
    }
}
