use super::{DigitalBusFrame, EngineeringValue, Transducer, check_max, check_range};
use crate::{MbdError, math};

const SDI_SHIFT: u32 = 8;
const DATA_SHIFT: u32 = 10;
const SSM_SHIFT: u32 = 29;
const PARITY_SHIFT: u32 = 31;
const LABEL_MASK: u32 = 0xFF;
const TWO_BIT_MASK: u32 = 0b11;
const DATA_MASK: u32 = (1 << 19) - 1;
const BNR_MAX: i32 = (1 << 18) - 1;
const BNR_MIN: i32 = -(1 << 18);
/// Sign/status matrix of a BNR word in normal operation.
const SSM_BNR_NORMAL: u8 = 0b11;
/// Five BCD digits, the leading one limited to the 3 bits left for it.
const BCD_MAX: u32 = 79_999;
const BCD_DIGIT_SHIFTS: [u32; 5] = [0, 4, 8, 12, 16];

/// The fields of an ARINC 429 word.
///
/// Words are held with ARINC bit 1 in the least significant bit, so the label
/// is bit-reversed in the word: ARINC transmits the label's most significant
/// bit first and every other field least significant bit first. `label` is
/// the value the octal label reads as, so label 203 is `0o203` here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Arinc429Fields {
    /// Octal label.
    pub label: u8,
    /// Source/destination identifier, 2 bits.
    pub sdi: u8,
    /// Data field, 19 bits; [`Arinc429Fields::bnr_data`] and
    /// [`Arinc429Fields::bcd_data`] fill it.
    pub data: u32,
    /// Sign/status matrix, 2 bits.
    pub ssm: u8,
}

impl Arinc429Fields {
    /// Assembles the word and sets its parity bit so the total of set bits is
    /// odd. Fields that do not fit are an error, never truncated.
    pub fn pack(&self) -> Result<u32, MbdError> {
        check_max("arinc429.sdi", self.sdi.into(), TWO_BIT_MASK.into())?;
        check_max("arinc429.data", self.data.into(), DATA_MASK.into())?;
        check_max("arinc429.ssm", self.ssm.into(), TWO_BIT_MASK.into())?;
        let body = u32::from(self.label.reverse_bits())
            | u32::from(self.sdi) << SDI_SHIFT
            | self.data << DATA_SHIFT
            | u32::from(self.ssm) << SSM_SHIFT;
        let parity = u32::from(body.count_ones().is_multiple_of(2));
        Ok(body | parity << PARITY_SHIFT)
    }

    /// Splits a received word, rejecting one with even parity.
    pub fn unpack(word: u32) -> Result<Self, MbdError> {
        if word.count_ones().is_multiple_of(2) {
            return Err(MbdError::Arinc429Parity { word });
        }
        let two_bits = |shift| ((word >> shift) & TWO_BIT_MASK) as u8;
        Ok(Self {
            label: ((word & LABEL_MASK) as u8).reverse_bits(),
            sdi: two_bits(SDI_SHIFT),
            data: (word >> DATA_SHIFT) & DATA_MASK,
            ssm: two_bits(SSM_SHIFT),
        })
    }

    /// Data field holding `counts` as 19-bit two's complement BNR, the top
    /// bit being the sign.
    pub fn bnr_data(counts: i32) -> Result<u32, MbdError> {
        check_range(
            "arinc429.bnr",
            counts.into(),
            BNR_MIN.into(),
            BNR_MAX.into(),
        )?;
        Ok(counts.cast_unsigned() & DATA_MASK)
    }

    /// The BNR value of the data field, in counts.
    pub fn bnr_counts(&self) -> i32 {
        let unused_bits = u32::BITS - DATA_MASK.count_ones();
        (self.data << unused_bits).cast_signed() >> unused_bits
    }

    /// Data field holding `value` as five BCD digits.
    pub fn bcd_data(value: u32) -> Result<u32, MbdError> {
        check_max("arinc429.bcd", value.into(), BCD_MAX.into())?;
        let mut remaining = value;
        let mut data = 0;
        for shift in BCD_DIGIT_SHIFTS {
            data |= (remaining % 10) << shift;
            remaining /= 10;
        }
        Ok(data)
    }

    /// The BCD value of the data field.
    pub fn bcd_value(&self) -> Result<u32, MbdError> {
        let mut value = 0;
        for shift in BCD_DIGIT_SHIFTS.into_iter().rev() {
            let digit = (self.data >> shift) & 0xF;
            if digit > 9 {
                return Err(MbdError::Arinc429Bcd { data: self.data });
            }
            value = value * 10 + digit;
        }
        Ok(value)
    }
}

impl DigitalBusFrame {
    /// An [`DigitalBusFrame::Arinc429`] frame carrying `fields`.
    pub fn arinc429(channel: u8, fields: &Arinc429Fields) -> Result<Self, MbdError> {
        Ok(Self::Arinc429 {
            channel,
            word: fields.pack()?,
        })
    }

    /// The fields of an [`DigitalBusFrame::Arinc429`] frame.
    pub fn arinc429_fields(&self) -> Result<Arinc429Fields, MbdError> {
        match *self {
            Self::Arinc429 { word, .. } => Arinc429Fields::unpack(word),
            _ => Err(MbdError::WrongPortKind {
                expected: "Arinc429",
            }),
        }
    }
}

/// A [`EngineeringValue::Scalar`] at the engineering tier, a BNR word on one
/// ARINC 429 channel and label at the bus tier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arinc429BnrTransducer {
    /// Channel the word travels on.
    pub channel: u8,
    /// Octal label of the word.
    pub label: u8,
    /// Source/destination identifier.
    pub sdi: u8,
    /// Engineering value of one data count.
    pub lsb_weight: f64,
}

impl Transducer for Arinc429BnrTransducer {
    type Upper = EngineeringValue;
    type Lower = DigitalBusFrame;

    /// Quantises to the nearest count; a value beyond the field's range is an
    /// error, as a transmitter would flag it rather than wrap it.
    fn encode(&self, upper: &EngineeringValue) -> Result<DigitalBusFrame, MbdError> {
        let EngineeringValue::Scalar(value) = *upper else {
            return Err(MbdError::WrongPortKind { expected: "Scalar" });
        };
        let counts = math::round(value / self.lsb_weight);
        if !counts.is_finite() {
            return Err(MbdError::NonFiniteSignal {
                field: "arinc429.bnr",
            });
        }
        DigitalBusFrame::arinc429(
            self.channel,
            &Arinc429Fields {
                label: self.label,
                sdi: self.sdi,
                data: Arinc429Fields::bnr_data(counts as i32)?,
                ssm: SSM_BNR_NORMAL,
            },
        )
    }

    fn decode(&self, lower: &DigitalBusFrame) -> Result<EngineeringValue, MbdError> {
        let fields = lower.arinc429_fields()?;
        if let DigitalBusFrame::Arinc429 { channel, .. } = *lower
            && (channel != self.channel || fields.label != self.label)
        {
            return Err(MbdError::Arinc429Routing {
                channel,
                label: fields.label,
            });
        }
        if fields.ssm != SSM_BNR_NORMAL {
            return Err(MbdError::Arinc429Status {
                label: fields.label,
                ssm: fields.ssm,
            });
        }
        Ok(EngineeringValue::Scalar(
            f64::from(fields.bnr_counts()) * self.lsb_weight,
        ))
    }
}
