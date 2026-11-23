//! Read-outs of a fly state: the wings' circulatory loads, the impulse of
//! the entrained air, and the energy and momentum of the bodies.

use glam::{DMat3, DVec3};
use mbd_aero::AeroElementKind;
use mbd_multibody::{
    ContactPointSpec, FloatingBaseState, Kinematics, ModelScale, ModelState, MultibodyModel,
    SpatialMotion, UniformGravity, Workspace,
};

use super::fly::{Fly, FlyWorld, WingServo};

/// The elements of a fly: gravity, the wing servo, the wings' air loads
/// and any contact points.
pub type FlyElements = (
    UniformGravity,
    WingServo,
    Vec<mbd_aero::AeroElement>,
    Vec<ContactPointSpec>,
);

pub fn elements(fly: &Fly, air: bool, contacts: Vec<ContactPointSpec>) -> FlyElements {
    let g = DVec3::new(0.0, 0.0, -fly.spec.gravity_m_s2);
    let wings = if air { fly.wings.clone() } else { Vec::new() };
    let mut servo = fly.servo;
    servo.air = air;
    (UniformGravity(g), servo, wings, contacts)
}

/// The model at the fly's own scale: wing length, fly mass and wingbeat
/// period.
pub fn model(fly: &Fly) -> MultibodyModel {
    let spec = &fly.spec;
    let scale = ModelScale::new(spec.wing_length_m, spec.total_mass_kg(), spec.period_s()).unwrap();
    MultibodyModel::new(&fly.tree, &fly.dynamic_masses, scale).unwrap()
}

/// State with the body at `base` and the wings on their prescribed angles
/// at time `t`.
pub fn state_at(
    fly: &Fly,
    model: &MultibodyModel,
    elements: &FlyElements,
    world: &FlyWorld,
    base: &FloatingBaseState,
    t: f64,
) -> ModelState {
    let mut state = model.state(&fly.tree, base, elements, world).unwrap();
    let [stroke, pitch] = super::fly::wing_angles(&fly.spec, t);
    for (side, (q, v, _)) in [stroke, pitch].into_iter().enumerate() {
        let ids = if side == 0 {
            [mbd_core::ComponentId(1), mbd_core::ComponentId(3)]
        } else {
            super::fly::WINGS
        };
        for (k, id) in ids.into_iter().enumerate() {
            let m = if k == 0 { 1.0 } else { -1.0 };
            let joint = mbd_core::JointState::revolute(m * q, m * v);
            model.set_joint_state(&mut state, id, &joint).unwrap();
        }
    }
    state
}

/// Wing read-outs at one state, world axes.
#[derive(Debug, Clone, Copy, Default)]
pub struct WingSample {
    /// Translational plus rotational force, N.
    pub force: DVec3,
    /// Translational force alone, N.
    pub translational: DVec3,
    /// Lift part of the translational force, N.
    pub lift: DVec3,
    /// Drag part of the translational force, N.
    pub drag: DVec3,
    /// Rotational circulation force, N.
    pub rotational: DVec3,
    /// Moment of the circulatory forces about the world origin, N·m.
    pub moment_origin: DVec3,
    /// Power taken by the circulatory forces, W.
    pub power_w: f64,
    /// Linear impulse of the entrained air, kg·m/s.
    pub fluid_linear: DVec3,
    /// Angular impulse of the entrained air about the world origin.
    pub fluid_angular: DVec3,
    /// Kinetic energy of the entrained air, J.
    pub fluid_energy_j: f64,
    /// Mean of the wings' Reynolds numbers.
    pub reynolds: f64,
}

pub fn wings(fly: &Fly, kin: &Kinematics<'_>, world: &FlyWorld) -> WingSample {
    let mut out = WingSample::default();
    for e in &fly.wings {
        let AeroElementKind::FlappingWing(m) = &e.kind else {
            continue;
        };
        let flow = e.flow(kin, world).unwrap();
        let loads = m.evaluate(&flow).unwrap();
        let o = kin.point(e.component, DVec3::ZERO).unwrap();
        let r = o.rotation_world_from_link;
        let f = r * loads.wrench.force_link_n;
        out.force += f;
        out.translational += r * loads.translational_force_link_n;
        out.lift += r * loads.translational_lift_link_n;
        out.drag += r * loads.translational_drag_link_n;
        out.rotational += r * loads.rotational_force_link_n;
        out.moment_origin += r * loads.wrench.moment_ref_link_nm + o.position_world_m.cross(f);
        out.power_w += loads.aerodynamic_power_w;
        out.reynolds += 0.5 * loads.mean_reynolds;
        let twist = SpatialMotion::new(
            o.to_link(o.angular_vel_world_rad_s),
            o.to_link(o.velocity_world_m_s),
        );
        let h = loads.added_inertia.mul_motion(twist);
        let p = r * h.force;
        out.fluid_linear += p;
        out.fluid_angular += r * h.torque + o.position_world_m.cross(p);
        out.fluid_energy_j += 0.5 * h.dot(twist);
    }
    out
}

/// Linear momentum and angular momentum about the world origin of the
/// bodies alone, with the system centre of mass.
pub fn bodies(model: &MultibodyModel, state: &ModelState) -> (DVec3, DVec3, DVec3) {
    let mut work = Workspace::new();
    let m = model.system_momentum(state, &mut work).unwrap();
    let h_origin = m.angular_about_com_world_nms + m.com_world_m.cross(m.linear_world_kg_m_s);
    (m.linear_world_kg_m_s, h_origin, m.com_world_m)
}

/// Kinetic plus potential energy of the bodies, J.
pub fn body_energy(fly: &Fly, kin: &Kinematics<'_>) -> f64 {
    fly.masses
        .iter()
        .map(|l| {
            let p = kin.point(l.id, l.cg_m).unwrap();
            let r: DMat3 = p.rotation_world_from_link;
            let i_world = r * l.inertia_cg * r.transpose();
            let w = p.angular_vel_world_rad_s;
            0.5 * l.mass_kg * p.velocity_world_m_s.length_squared()
                + 0.5 * w.dot(i_world * w)
                + l.mass_kg * fly.spec.gravity_m_s2 * p.position_world_m.z
        })
        .sum()
}
