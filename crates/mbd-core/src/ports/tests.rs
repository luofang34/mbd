use std::f64::consts::{FRAC_PI_2, PI};

use super::*;

fn fields() -> Arinc429Fields {
    Arinc429Fields {
        label: 0o203,
        sdi: 0b10,
        data: 0x5_A5A5,
        ssm: 0b11,
    }
}

#[test]
fn arinc429_fields_round_trip_with_odd_parity() {
    for (label, sdi, data, ssm) in [
        (0o000, 0, 0, 0),
        (0o377, 3, (1 << 19) - 1, 3),
        (0o203, 2, 0x5_A5A5, 3),
        (0o001, 1, 1, 1),
    ] {
        let original = Arinc429Fields {
            label,
            sdi,
            data,
            ssm,
        };
        let word = original.pack().unwrap();
        assert_eq!(word.count_ones() % 2, 1, "{word:#x}");
        assert_eq!(Arinc429Fields::unpack(word), Ok(original));
    }
}

#[test]
fn arinc429_fields_sit_at_their_standard_bits() {
    let word = Arinc429Fields {
        label: 0,
        sdi: 0b01,
        data: 1,
        ssm: 0b10,
    }
    .pack()
    .unwrap();
    assert_eq!(word & (1 << 8), 1 << 8, "SDI starts at ARINC bit 9");
    assert_eq!(word & (1 << 10), 1 << 10, "data starts at ARINC bit 11");
    assert_eq!(word & (0b11 << 29), 0b10 << 29, "SSM is ARINC bits 30-31");
    assert_eq!(word >> 31, 0, "three set bits are already odd");
}

#[test]
fn arinc429_label_is_bit_reversed_in_the_word() {
    let word = fields().pack().unwrap();
    assert_eq!(word & 0xFF, 0b1100_0001);
    assert_eq!(0o203u8, 0b1000_0011);
}

#[test]
fn arinc429_any_single_bit_error_fails_parity() {
    let word = fields().pack().unwrap();
    for bit in 0..32 {
        let corrupted = word ^ (1 << bit);
        assert_eq!(
            Arinc429Fields::unpack(corrupted),
            Err(MbdError::Arinc429Parity { word: corrupted })
        );
    }
}

#[test]
fn arinc429_oversized_fields_are_rejected_not_truncated() {
    for bad in [
        Arinc429Fields { sdi: 4, ..fields() },
        Arinc429Fields { ssm: 4, ..fields() },
        Arinc429Fields {
            data: 1 << 19,
            ..fields()
        },
    ] {
        assert!(matches!(bad.pack(), Err(MbdError::FieldOutOfRange { .. })));
    }
}

#[test]
fn arinc429_bnr_is_nineteen_bit_twos_complement() {
    for counts in [0, 1, -1, 1234, -1234, (1 << 18) - 1, -(1 << 18)] {
        let data = Arinc429Fields::bnr_data(counts).unwrap();
        assert!(data < 1 << 19);
        let unpacked = Arinc429Fields { data, ..fields() };
        assert_eq!(unpacked.bnr_counts(), counts);
    }
    assert_eq!(Arinc429Fields::bnr_data(-1), Ok((1 << 19) - 1));
    assert!(Arinc429Fields::bnr_data(1 << 18).is_err());
    assert!(Arinc429Fields::bnr_data(-(1 << 18) - 1).is_err());
}

#[test]
fn arinc429_bcd_holds_five_digits() {
    assert_eq!(Arinc429Fields::bcd_data(12_345), Ok(0x1_2345));
    for value in [0, 9, 10, 12_345, 79_999] {
        let data = Arinc429Fields::bcd_data(value).unwrap();
        assert!(data < 1 << 19);
        assert_eq!(Arinc429Fields { data, ..fields() }.bcd_value(), Ok(value));
    }
    assert!(Arinc429Fields::bcd_data(80_000).is_err());
    let data = 0x0_00A0;
    assert_eq!(
        Arinc429Fields { data, ..fields() }.bcd_value(),
        Err(MbdError::Arinc429Bcd { data })
    );
}

#[test]
fn arinc429_bnr_transducer_round_trips_engineering_values() {
    let adc = Arinc429BnrTransducer {
        channel: 2,
        label: 0o205,
        sdi: 0,
        lsb_weight: 0.125,
    };
    let value = EngineeringValue::Scalar(-1234.5);
    let frame = adc.encode(&value).unwrap();
    assert!(frame.validate().is_ok());
    assert_eq!(adc.decode(&frame), Ok(value));

    let beyond_range = EngineeringValue::Scalar(1e9);
    assert!(adc.encode(&beyond_range).is_err());
    assert_eq!(
        adc.encode(&EngineeringValue::Scalar(f64::NAN)),
        Err(MbdError::NonFiniteSignal {
            field: "arinc429.bnr"
        })
    );
    assert_eq!(
        adc.encode(&EngineeringValue::Discrete(true)),
        Err(MbdError::WrongPortKind { expected: "Scalar" })
    );
}

#[test]
fn arinc429_bnr_transducer_refuses_misrouted_or_invalid_words() {
    let adc = Arinc429BnrTransducer {
        channel: 2,
        label: 0o205,
        sdi: 0,
        lsb_weight: 1.0,
    };
    let frame = adc.encode(&EngineeringValue::Scalar(1.0)).unwrap();
    let other_channel = Arinc429BnrTransducer { channel: 3, ..adc };
    assert!(matches!(
        other_channel.decode(&frame),
        Err(MbdError::Arinc429Routing { channel: 2, .. })
    ));
    let failed = Arinc429Fields {
        label: 0o205,
        sdi: 0,
        data: 1,
        ssm: 0b00,
    };
    let failed = DigitalBusFrame::arinc429(2, &failed).unwrap();
    assert_eq!(
        adc.decode(&failed),
        Err(MbdError::Arinc429Status {
            label: 0o205,
            ssm: 0
        })
    );
}

#[test]
fn resolver_round_trips_angles_in_every_quadrant() {
    let mut angle = -PI + 1e-3;
    while angle < PI {
        let pins = ElectricalPinState::from_angle_rad(26.0, angle);
        assert!((pins.decode_angle_rad().unwrap() - angle).abs() < 1e-12);
        angle += 0.0137;
    }
}

#[test]
fn resolver_wraps_multi_turn_shafts_and_uses_the_excitation_phase() {
    let pins = ElectricalPinState::from_angle_rad(26.0, 2.0 * PI + FRAC_PI_2);
    assert!((pins.decode_angle_rad().unwrap() - FRAC_PI_2).abs() < 1e-12);

    let negative_half_cycle = ElectricalPinState::from_angle_rad(-26.0, 1.0);
    assert!((negative_half_cycle.decode_angle_rad().unwrap() - 1.0).abs() < 1e-12);
}

#[test]
fn resolver_without_signal_reports_loss_instead_of_an_angle() {
    for lost in [
        ElectricalPinState::from_angle_rad(0.0, 1.0),
        ElectricalPinState::ResolverSinCos {
            excitation_v: 26.0,
            sin_v: 0.0,
            cos_v: 0.0,
        },
        ElectricalPinState::ResolverSinCos {
            excitation_v: 26.0,
            sin_v: f64::NAN,
            cos_v: 1.0,
        },
    ] {
        assert!(matches!(
            lost.decode_angle_rad(),
            Err(MbdError::ResolverSignalLost { .. })
        ));
    }
    assert_eq!(
        ElectricalPinState::LogicLevel(true).decode_angle_rad(),
        Err(MbdError::WrongPortKind {
            expected: "ResolverSinCos"
        })
    );
}

#[test]
fn resolver_transducer_crosses_between_tiers() {
    let resolver = ResolverTransducer { excitation_v: 26.0 };
    let pins = resolver.encode(&EngineeringValue::Scalar(0.75)).unwrap();
    let EngineeringValue::Scalar(angle) = resolver.decode(&pins).unwrap() else {
        panic!("a resolver decodes to a scalar");
    };
    assert!((angle - 0.75).abs() < 1e-12);
    assert_eq!(
        resolver.encode(&EngineeringValue::Integer(1)),
        Err(MbdError::WrongPortKind { expected: "Scalar" })
    );
}

fn out_of_range(frame: DigitalBusFrame) -> bool {
    matches!(frame.validate(), Err(MbdError::FieldOutOfRange { .. }))
}

fn can(id: u32, extended: bool, dlc: u8) -> DigitalBusFrame {
    DigitalBusFrame::Can {
        bus_id: 0,
        id,
        extended,
        data: [0; 8],
        dlc,
    }
}

fn mil_std(rt_address: u8, subaddress: u8, word_count: u8) -> DigitalBusFrame {
    DigitalBusFrame::MilStd1553 {
        rt_address,
        subaddress,
        word_count,
        words: [0; 32],
    }
}

#[test]
fn bus_frames_are_checked_against_their_protocol_widths() {
    assert_eq!(can(0x7FF, false, 8).validate(), Ok(()));
    assert_eq!(can(0x1FFF_FFFF, true, 0).validate(), Ok(()));
    assert!(out_of_range(can(0x800, false, 8)));
    assert!(out_of_range(can(0x2000_0000, true, 8)));
    assert!(out_of_range(can(1, false, 9)));

    assert_eq!(mil_std(31, 31, 32).validate(), Ok(()));
    assert!(out_of_range(mil_std(32, 0, 1)));
    assert!(out_of_range(mil_std(0, 32, 1)));
    assert!(out_of_range(mil_std(0, 0, 0)));
    assert!(out_of_range(mil_std(0, 0, 33)));

    let i2c = |addr_7bit, len| DigitalBusFrame::I2c {
        bus_id: 0,
        addr_7bit,
        register: 0,
        payload: [0; 8],
        len,
    };
    assert_eq!(i2c(0x7F, 8).validate(), Ok(()));
    assert!(out_of_range(i2c(0x80, 1)));
    assert!(out_of_range(i2c(1, 9)));

    let spi = DigitalBusFrame::Spi {
        bus_id: 0,
        cs: 0,
        register: 0,
        payload: [0; 8],
        len: 9,
    };
    assert!(out_of_range(spi));
}

#[test]
fn agc_frames_and_arinc_words_are_checked() {
    let channel = |channel, value_15bit| DigitalBusFrame::AgcChannelWord {
        channel,
        value_15bit,
    };
    assert_eq!(channel(0o777, 0x7FFF).validate(), Ok(()));
    assert!(out_of_range(channel(0o1000, 0)));
    assert!(out_of_range(channel(0, 0x8000)));

    let pulses = DigitalBusFrame::AgcCduPulses {
        axis: 2,
        delta_pulses: -40,
    };
    assert_eq!(pulses.validate(), Ok(()));
    assert_eq!(
        pulses.arinc429_fields(),
        Err(MbdError::WrongPortKind {
            expected: "Arinc429"
        })
    );

    let even_parity = DigitalBusFrame::Arinc429 {
        channel: 0,
        word: 0,
    };
    assert_eq!(
        even_parity.validate(),
        Err(MbdError::Arinc429Parity { word: 0 })
    );
}
