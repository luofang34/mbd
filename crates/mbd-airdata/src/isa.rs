//! The ICAO standard atmosphere (identical to US Standard Atmosphere 1976
//! below 51 km geopotential), with a temperature offset for non-standard days.

#[cfg(test)]
mod tests;

/// Sea-level standard pressure, Pa.
pub const ISA_P0_PA: f64 = 101_325.0;
/// Sea-level standard temperature, K.
pub const ISA_T0_K: f64 = 288.15;
/// Sea-level standard density, kg/m³.
pub const ISA_RHO0_KG_M3: f64 = 1.225;
/// Sea-level standard speed of sound, m/s: the reference speed of the CAS law.
pub const ISA_SPEED_OF_SOUND0_M_S: f64 = 340.294;
/// Standard gravity, m/s², which defines geopotential altitude.
pub const ISA_G0_M_S2: f64 = 9.806_65;
/// Specific gas constant of dry air, J/(kg·K), as ICAO Doc 7488 fixes it.
pub const R_AIR_J_KG_K: f64 = 287.052_87;
/// Ratio of specific heats of air.
pub const GAMMA_AIR: f64 = 1.4;
/// Troposphere temperature lapse rate, K/m; temperature falls with height.
pub const ISA_LAPSE_RATE_K_M: f64 = -0.0065;

/// Radius the standard uses to relate geometric and geopotential altitude, m.
const EARTH_RADIUS_GEOPOTENTIAL_M: f64 = 6_356_766.0;

/// Layer bases as (geopotential altitude m, temperature K, lapse rate K/m).
/// Above the last base the layer is isothermal to 51 km and is extended
/// beyond it.
const LAYERS: [(f64, f64, f64); 5] = [
    (0.0, ISA_T0_K, ISA_LAPSE_RATE_K_M),
    (11_000.0, 216.65, 0.0),
    (20_000.0, 216.65, 0.001),
    (32_000.0, 228.65, 0.0028),
    (47_000.0, 270.65, 0.0),
];

/// The state of the air at one point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Atmosphere {
    /// Static pressure, Pa.
    pub pressure_pa: f64,
    /// Static temperature, K.
    pub temperature_k: f64,
    /// Density, kg/m³.
    pub density_kg_m3: f64,
    /// Speed of sound, m/s.
    pub speed_of_sound_m_s: f64,
}

/// One layer with its base pressure filled in.
#[derive(Debug, Clone, Copy)]
struct Layer {
    base_m: f64,
    base_k: f64,
    lapse_k_m: f64,
    base_pa: f64,
}

impl Layer {
    fn temperature_k(&self, altitude_m: f64) -> f64 {
        self.base_k + self.lapse_k_m * (altitude_m - self.base_m)
    }

    fn pressure_pa(&self, altitude_m: f64) -> f64 {
        if self.lapse_k_m == 0.0 {
            let scale_m = R_AIR_J_KG_K * self.base_k / ISA_G0_M_S2;
            self.base_pa * libm::exp(-(altitude_m - self.base_m) / scale_m)
        } else {
            let exponent = -ISA_G0_M_S2 / (R_AIR_J_KG_K * self.lapse_k_m);
            self.base_pa * libm::pow(self.temperature_k(altitude_m) / self.base_k, exponent)
        }
    }

    fn altitude_m(&self, pressure_pa: f64) -> f64 {
        if self.lapse_k_m == 0.0 {
            let scale_m = R_AIR_J_KG_K * self.base_k / ISA_G0_M_S2;
            self.base_m - scale_m * libm::log(pressure_pa / self.base_pa)
        } else {
            let exponent = -R_AIR_J_KG_K * self.lapse_k_m / ISA_G0_M_S2;
            let temperature_k = self.base_k * libm::pow(pressure_pa / self.base_pa, exponent);
            self.base_m + (temperature_k - self.base_k) / self.lapse_k_m
        }
    }
}

/// The layers with base pressures chained up from sea level, so pressure is
/// continuous across every boundary.
fn layers() -> [Layer; 5] {
    let mut base_pa = ISA_P0_PA;
    let mut previous: Option<Layer> = None;
    LAYERS.map(|(base_m, base_k, lapse_k_m)| {
        if let Some(below) = previous {
            base_pa = below.pressure_pa(base_m);
        }
        let layer = Layer {
            base_m,
            base_k,
            lapse_k_m,
            base_pa,
        };
        previous = Some(layer);
        layer
    })
}

/// The layer containing geopotential `altitude_m`; below sea level the
/// troposphere is extended downwards.
fn layer_at_altitude(altitude_m: f64) -> Layer {
    let all = layers();
    let [mut found, ..] = all;
    for layer in all {
        if altitude_m >= layer.base_m {
            found = layer;
        }
    }
    found
}

/// Geopotential altitude, m, of a geometric altitude above mean sea level.
/// The standard's layers are defined in geopotential altitude.
pub fn geopotential_altitude_m(geometric_m: f64) -> f64 {
    EARTH_RADIUS_GEOPOTENTIAL_M * geometric_m / (EARTH_RADIUS_GEOPOTENTIAL_M + geometric_m)
}

/// The speed of sound in air at `temperature_k`, m/s.
pub fn speed_of_sound_m_s(temperature_k: f64) -> f64 {
    libm::sqrt(GAMMA_AIR * R_AIR_J_KG_K * temperature_k)
}

/// The atmosphere at a pressure altitude (geopotential, m) on a day
/// `delta_t_k` warmer than standard. Pressure follows the standard at that
/// pressure altitude, as ISA+ΔT tables are tabulated; the offset changes
/// temperature, density and the speed of sound.
pub fn isa_atmosphere(pressure_altitude_m: f64, delta_t_k: f64) -> Atmosphere {
    let layer = layer_at_altitude(pressure_altitude_m);
    let pressure_pa = layer.pressure_pa(pressure_altitude_m);
    let temperature_k = layer.temperature_k(pressure_altitude_m) + delta_t_k;
    Atmosphere {
        pressure_pa,
        temperature_k,
        density_kg_m3: pressure_pa / (R_AIR_J_KG_K * temperature_k),
        speed_of_sound_m_s: speed_of_sound_m_s(temperature_k),
    }
}

/// The geopotential altitude, m, at which the standard atmosphere has
/// `pressure_pa`: what an altimeter set to 1013.25 hPa reads. Pressures above
/// sea-level standard give negative altitudes.
pub fn pressure_altitude_m(pressure_pa: f64) -> f64 {
    let all = layers();
    let [mut found, ..] = all;
    for layer in all {
        if pressure_pa <= layer.base_pa {
            found = layer;
        }
    }
    found.altitude_m(pressure_pa)
}

/// The standard atmosphere's vertical pressure gradient `dp/dh`, Pa/m, at
/// static pressure `pressure_pa`: negative, since pressure falls with height.
/// A vertical speed indicator is calibrated against this gradient.
pub fn pressure_lapse_pa_m(pressure_pa: f64) -> f64 {
    let altitude_m = pressure_altitude_m(pressure_pa);
    let temperature_k = layer_at_altitude(altitude_m).temperature_k(altitude_m);
    -pressure_pa * ISA_G0_M_S2 / (R_AIR_J_KG_K * temperature_k)
}
