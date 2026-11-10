use glam::DVec3;

use crate::MbdError;

#[cfg(test)]
mod tests;

/// Metres in one inch, the unit of factory station drawings and of JSBSim's
/// structural frame.
pub const METRES_PER_INCH: f64 = 0.0254;

/// Which way the three coordinates of a station frame point on the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StationAxes {
    /// `x` aft, `y` right, `z` up: fuselage station, butt line and water
    /// line of a factory drawing, and JSBSim's structural frame.
    AftRightUp,
    /// `x` forward, `y` right, `z` down: vehicle body axes.
    ForwardRightDown,
    /// `x` right, `y` forward, `z` up: the modelling convention of a
    /// Z-up scene.
    RightForwardUp,
}

impl StationAxes {
    /// A vector in these axes, expressed forward-right-down.
    fn forward_right_down(self, v: DVec3) -> DVec3 {
        match self {
            Self::AftRightUp => DVec3::new(-v.x, v.y, -v.z),
            Self::ForwardRightDown => v,
            Self::RightForwardUp => DVec3::new(v.y, v.x, -v.z),
        }
    }

    /// A forward-right-down vector, expressed in these axes. Each
    /// convention's map is its own inverse.
    fn in_these_axes(self, v: DVec3) -> DVec3 {
        self.forward_right_down(v)
    }
}

/// A coordinate frame a drawing, a handbook or a flight model states
/// positions in: its axes, its unit, and where its origin sits on the
/// vehicle.
///
/// Positions on one vehicle are stated in several such frames (a weight and
/// balance sheet in inches aft of a datum, a mesh in metres, a flight model
/// in its own structural inches). Declaring each frame once and converting
/// through body axes replaces a sign flip and a unit factor at every use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StationFrame {
    axes: StationAxes,
    metres_per_unit: f64,
    origin_body_m: DVec3,
}

impl StationFrame {
    /// Vehicle body axes in metres.
    pub const BODY: Self = Self {
        axes: StationAxes::ForwardRightDown,
        metres_per_unit: 1.0,
        origin_body_m: DVec3::ZERO,
    };

    /// A frame with `axes`, a unit `metres_per_unit` long, and its origin at
    /// `origin_body_m` in body axes.
    pub fn new(
        axes: StationAxes,
        metres_per_unit: f64,
        origin_body_m: DVec3,
    ) -> Result<Self, MbdError> {
        if !(metres_per_unit.is_finite() && metres_per_unit > 0.0) {
            return Err(MbdError::InvalidStationFrame {
                field: "metres_per_unit",
                value: metres_per_unit,
            });
        }
        if let Some(value) = origin_body_m
            .to_array()
            .into_iter()
            .find(|v| !v.is_finite())
        {
            return Err(MbdError::InvalidStationFrame {
                field: "origin_body_m",
                value,
            });
        }
        Ok(Self {
            axes,
            metres_per_unit,
            origin_body_m,
        })
    }

    /// A frame placed by one known point: `station` in this frame is the
    /// point `body_m` in body axes. This is how a drawing datum is usually
    /// known, for example "the firewall is fuselage station 0".
    pub fn through(
        axes: StationAxes,
        metres_per_unit: f64,
        station: DVec3,
        body_m: DVec3,
    ) -> Result<Self, MbdError> {
        let unplaced = Self::new(axes, metres_per_unit, DVec3::ZERO)?;
        Self::new(axes, metres_per_unit, body_m - unplaced.to_body_m(station))
    }

    /// The frame's axes.
    pub fn axes(&self) -> StationAxes {
        self.axes
    }

    /// Length of the frame's unit, metres.
    pub fn metres_per_unit(&self) -> f64 {
        self.metres_per_unit
    }

    /// The point `station` of this frame, in body axes, metres.
    pub fn to_body_m(&self, station: DVec3) -> DVec3 {
        self.origin_body_m + self.offset_to_body_m(station)
    }

    /// The body-axes point `body_m`, in this frame's coordinates and unit.
    pub fn station_at(&self, body_m: DVec3) -> DVec3 {
        self.offset_in_frame(body_m - self.origin_body_m)
    }

    /// A displacement stated in this frame, in body axes, metres. The origin
    /// does not enter.
    pub fn offset_to_body_m(&self, offset: DVec3) -> DVec3 {
        self.axes.forward_right_down(offset * self.metres_per_unit)
    }

    /// A body-axes displacement, in this frame's coordinates and unit.
    pub fn offset_in_frame(&self, offset_body_m: DVec3) -> DVec3 {
        self.axes.in_these_axes(offset_body_m) / self.metres_per_unit
    }

    /// The point `station` of this frame, in the coordinates of `other`.
    pub fn convert(&self, station: DVec3, other: &Self) -> DVec3 {
        other.station_at(self.to_body_m(station))
    }
}

/// A mass and where it sits, in the coordinates of whichever frame the
/// caller is working in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassStation {
    /// Mass, kg.
    pub mass_kg: f64,
    /// Position in the working frame.
    pub station: DVec3,
}

impl MassStation {
    /// The total mass of `stations` at their centre of gravity, all in one
    /// frame.
    pub fn combined(stations: &[Self]) -> Result<Self, MbdError> {
        let mut mass_kg = 0.0;
        let mut moment = DVec3::ZERO;
        for item in stations {
            if !(item.mass_kg.is_finite() && item.mass_kg >= 0.0) {
                return Err(MbdError::InvalidStationFrame {
                    field: "mass_kg",
                    value: item.mass_kg,
                });
            }
            mass_kg += item.mass_kg;
            moment += item.station * item.mass_kg;
        }
        if mass_kg <= 0.0 {
            return Err(MbdError::NoAttachedMass);
        }
        Ok(Self {
            mass_kg,
            station: moment / mass_kg,
        })
    }

    /// The same mass with its position in the coordinates of `to`.
    pub fn in_frame(&self, from: &StationFrame, to: &StationFrame) -> Self {
        Self {
            mass_kg: self.mass_kg,
            station: from.convert(self.station, to),
        }
    }
}
