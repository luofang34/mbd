use glam::{DQuat, DVec3};

use crate::MbdError;

mod arinc429;
mod resolver;

pub use arinc429::{Arinc429BnrTransducer, Arinc429Fields};
pub use resolver::ResolverTransducer;

#[cfg(test)]
mod tests;

const I2C_ADDRESS_MAX: u8 = 0x7F;
const CAN_STANDARD_ID_MAX: u32 = 0x7FF;
const CAN_EXTENDED_ID_MAX: u32 = 0x1FFF_FFFF;
const SHORT_PAYLOAD_MAX: u8 = 8;
const MIL_STD_1553_ADDRESS_MAX: u8 = 31;
const MIL_STD_1553_WORDS_MAX: u8 = 32;
const AGC_CHANNEL_MAX: u16 = 0o777;
const AGC_WORD_MAX: u16 = 0x7FFF;

/// Tier 1: a physical value in SI units, the fastest and most abstract port.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EngineeringValue {
    /// A real quantity.
    Scalar(f64),
    /// A three-axis quantity.
    Vec3(DVec3),
    /// A rotation.
    Quat(DQuat),
    /// An on/off state.
    Discrete(bool),
    /// A count or enumerated state.
    Integer(i64),
}

/// Tier 2: a frame as it crosses a digital bus, for bus-accurate models and
/// emulated computers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DigitalBusFrame {
    /// SPI register transfer.
    Spi {
        /// Bus instance.
        bus_id: u8,
        /// Chip-select line.
        cs: u8,
        /// Device register.
        register: u8,
        /// Transfer bytes; only the first `len` are meaningful.
        payload: [u8; 8],
        /// Bytes used in `payload`.
        len: u8,
    },
    /// I2C register transfer.
    I2c {
        /// Bus instance.
        bus_id: u8,
        /// 7-bit device address.
        addr_7bit: u8,
        /// Device register.
        register: u8,
        /// Transfer bytes; only the first `len` are meaningful.
        payload: [u8; 8],
        /// Bytes used in `payload`.
        len: u8,
    },
    /// Classic CAN data frame.
    Can {
        /// Bus instance.
        bus_id: u8,
        /// 11-bit identifier, or 29-bit when `extended`.
        id: u32,
        /// Whether `id` uses the 29-bit format.
        extended: bool,
        /// Frame bytes; only the first `dlc` are meaningful.
        data: [u8; 8],
        /// Data length code.
        dlc: u8,
    },
    /// ARINC 429 word; [`Arinc429Fields`] packs and unpacks `word`.
    Arinc429 {
        /// Receiver channel.
        channel: u8,
        /// Word with ARINC bit 1 in the least significant bit.
        word: u32,
    },
    /// MIL-STD-1553 message.
    MilStd1553 {
        /// Remote terminal address.
        rt_address: u8,
        /// Subaddress.
        subaddress: u8,
        /// Data words carried, 1 to 32.
        word_count: u8,
        /// Message data; only the first `word_count` are meaningful.
        words: [u16; 32],
    },
    /// Incremental pulse train from an Apollo coupling data unit, as the
    /// guidance computer counts IMU and optics motion.
    AgcCduPulses {
        /// Which coupling data unit.
        axis: u8,
        /// Signed pulses since the previous frame.
        delta_pulses: i16,
    },
    /// Apollo guidance computer I/O channel word, including the DSKY.
    AgcChannelWord {
        /// I/O channel number.
        channel: u16,
        /// 15-bit channel contents.
        value_15bit: u16,
    },
}

impl DigitalBusFrame {
    /// Checks every field fits its protocol's bit width, and that an ARINC
    /// 429 word has odd parity. The variants are plain data, so a frame from
    /// outside the simulation can hold values the bus could not carry.
    pub fn validate(&self) -> Result<(), MbdError> {
        match *self {
            Self::Spi { len, .. } => check_max("spi.len", len.into(), SHORT_PAYLOAD_MAX.into()),
            Self::I2c { addr_7bit, len, .. } => {
                check_max("i2c.addr_7bit", addr_7bit.into(), I2C_ADDRESS_MAX.into())?;
                check_max("i2c.len", len.into(), SHORT_PAYLOAD_MAX.into())
            }
            Self::Can {
                id, extended, dlc, ..
            } => {
                let id_max = if extended {
                    CAN_EXTENDED_ID_MAX
                } else {
                    CAN_STANDARD_ID_MAX
                };
                check_max("can.id", id.into(), id_max.into())?;
                check_max("can.dlc", dlc.into(), SHORT_PAYLOAD_MAX.into())
            }
            Self::Arinc429 { word, .. } => Arinc429Fields::unpack(word).map(|_| ()),
            Self::MilStd1553 {
                rt_address,
                subaddress,
                word_count,
                ..
            } => validate_1553(rt_address, subaddress, word_count),
            Self::AgcCduPulses { .. } => Ok(()),
            Self::AgcChannelWord {
                channel,
                value_15bit,
            } => {
                check_max("agc.channel", channel.into(), AGC_CHANNEL_MAX.into())?;
                check_max("agc.value_15bit", value_15bit.into(), AGC_WORD_MAX.into())
            }
        }
    }
}

fn validate_1553(rt_address: u8, subaddress: u8, word_count: u8) -> Result<(), MbdError> {
    let address_max = MIL_STD_1553_ADDRESS_MAX.into();
    check_max("1553.rt_address", rt_address.into(), address_max)?;
    check_max("1553.subaddress", subaddress.into(), address_max)?;
    check_range(
        "1553.word_count",
        word_count.into(),
        1,
        MIL_STD_1553_WORDS_MAX.into(),
    )
}

fn check_max(field: &'static str, value: i64, max: i64) -> Result<(), MbdError> {
    check_range(field, value, 0, max)
}

fn check_range(field: &'static str, value: i64, min: i64, max: i64) -> Result<(), MbdError> {
    if (min..=max).contains(&value) {
        return Ok(());
    }
    Err(MbdError::FieldOutOfRange {
        field,
        value,
        min,
        max,
    })
}

/// Tier 3: the state of a physical pin or pin group, for electrical models
/// and hardware in the loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElectricalPinState {
    /// Analog line.
    AnalogVoltage {
        /// Line voltage, V.
        volts: f64,
        /// Current drawn from the line, A.
        current_draw_a: f64,
    },
    /// Digital line.
    LogicLevel(bool),
    /// Resolver windings at the excitation carrier's phase; see
    /// [`ElectricalPinState::from_angle_rad`].
    ResolverSinCos {
        /// Excitation voltage on the rotor.
        excitation_v: f64,
        /// Sine stator winding voltage.
        sin_v: f64,
        /// Cosine stator winding voltage.
        cos_v: f64,
    },
}

/// Converts between two port tiers. `Upper` is the more abstract tier;
/// [`Transducer::encode`] moves down towards the wire and
/// [`Transducer::decode`] back up.
pub trait Transducer {
    /// The more abstract port value.
    type Upper;
    /// The closer-to-hardware port value.
    type Lower;

    /// Converts towards the hardware tier.
    fn encode(&self, upper: &Self::Upper) -> Result<Self::Lower, MbdError>;

    /// Converts towards the engineering tier.
    fn decode(&self, lower: &Self::Lower) -> Result<Self::Upper, MbdError>;
}
