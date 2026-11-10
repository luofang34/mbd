use super::*;

fn close(a: DVec3, b: DVec3) {
    assert!((a - b).length() < 1e-9, "{a} against {b}");
}

/// A light aeroplane whose handbook datum (fuselage station 0) is the
/// firewall, 1.0 m ahead of and 0.3 m above the body origin.
fn handbook() -> StationFrame {
    StationFrame::through(
        StationAxes::AftRightUp,
        METRES_PER_INCH,
        DVec3::ZERO,
        DVec3::new(1.0, 0.0, -0.3),
    )
    .unwrap()
}

#[test]
fn a_station_aft_of_the_datum_is_behind_it_in_body_axes() {
    let frame = handbook();
    // 40 in aft, 10 in right, 20 in above the datum.
    let body = frame.to_body_m(DVec3::new(40.0, 10.0, 20.0));
    close(body, DVec3::new(1.0 - 1.016, 0.254, -0.3 - 0.508));
    close(frame.station_at(body), DVec3::new(40.0, 10.0, 20.0));
}

#[test]
fn each_axis_convention_maps_forward_right_and_up_correctly() {
    let forward = DVec3::X;
    let right = DVec3::Y;
    let up = -DVec3::Z;
    for (axes, f, r, u) in [
        (StationAxes::AftRightUp, -DVec3::X, DVec3::Y, DVec3::Z),
        (StationAxes::ForwardRightDown, DVec3::X, DVec3::Y, -DVec3::Z),
        (StationAxes::RightForwardUp, DVec3::Y, DVec3::X, DVec3::Z),
    ] {
        let frame = StationFrame::new(axes, 1.0, DVec3::ZERO).unwrap();
        close(frame.offset_to_body_m(f), forward);
        close(frame.offset_to_body_m(r), right);
        close(frame.offset_to_body_m(u), up);
        close(frame.offset_in_frame(forward), f);
        close(frame.offset_in_frame(right), r);
        close(frame.offset_in_frame(up), u);
    }
}

#[test]
fn converting_between_frames_goes_through_the_vehicle_not_the_numbers() {
    let handbook = handbook();
    // A flight model's structural frame: the same axes and unit, with its
    // origin 78.8787 in ahead of the handbook datum.
    let structural = StationFrame::through(
        StationAxes::AftRightUp,
        METRES_PER_INCH,
        DVec3::new(78.8787, 0.0, 0.0),
        handbook.to_body_m(DVec3::ZERO),
    )
    .unwrap();
    let cg = DVec3::new(47.3, 0.0, 0.0);
    close(
        handbook.convert(cg, &structural),
        DVec3::new(126.1787, 0.0, 0.0),
    );
    close(
        structural.convert(handbook.convert(cg, &structural), &handbook),
        cg,
    );
    // A mesh in metres, x right, y forward, z up, sharing the body origin.
    let mesh = StationFrame::new(StationAxes::RightForwardUp, 1.0, DVec3::ZERO).unwrap();
    close(
        handbook.convert(DVec3::new(40.0, 10.0, 20.0), &mesh),
        DVec3::new(0.254, 1.0 - 1.016, 0.3 + 0.508),
    );
    close(
        StationFrame::BODY.convert(DVec3::new(1.0, 2.0, 3.0), &StationFrame::BODY),
        DVec3::new(1.0, 2.0, 3.0),
    );
}

#[test]
fn a_displacement_ignores_the_origin() {
    let frame = handbook();
    close(
        frame.offset_to_body_m(DVec3::new(10.0, 0.0, 0.0)),
        DVec3::new(-0.254, 0.0, 0.0),
    );
    close(
        frame.to_body_m(DVec3::new(50.0, 0.0, 0.0)) - frame.to_body_m(DVec3::new(40.0, 0.0, 0.0)),
        frame.offset_to_body_m(DVec3::new(10.0, 0.0, 0.0)),
    );
}

#[test]
fn a_frame_with_no_usable_unit_or_origin_is_refused() {
    for unit in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            StationFrame::new(StationAxes::AftRightUp, unit, DVec3::ZERO),
            Err(MbdError::InvalidStationFrame {
                field: "metres_per_unit",
                ..
            })
        ));
    }
    assert!(matches!(
        StationFrame::new(StationAxes::AftRightUp, 1.0, DVec3::new(0.0, f64::NAN, 0.0)),
        Err(MbdError::InvalidStationFrame {
            field: "origin_body_m",
            ..
        })
    ));
}

#[test]
fn masses_combine_at_their_moment_arm_in_any_frame() {
    // A weight and balance sheet in pounds-free form: kg at inches aft of datum.
    let sheet = [
        MassStation {
            mass_kg: 770.0,
            station: DVec3::new(40.26, 0.0, 0.0),
        },
        MassStation {
            mass_kg: 154.0,
            station: DVec3::new(37.0, 0.0, 0.0),
        },
        MassStation {
            mass_kg: 144.0,
            station: DVec3::new(48.0, 0.0, 0.0),
        },
    ];
    let loaded = MassStation::combined(&sheet).unwrap();
    assert!((loaded.mass_kg - 1068.0).abs() < 1e-12);
    let arm = (770.0 * 40.26 + 154.0 * 37.0 + 144.0 * 48.0) / 1068.0;
    close(loaded.station, DVec3::new(arm, 0.0, 0.0));

    // Combining after conversion gives the converted centre of gravity.
    let frame = handbook();
    let in_body: Vec<_> = sheet
        .iter()
        .map(|item| item.in_frame(&frame, &StationFrame::BODY))
        .collect();
    close(
        MassStation::combined(&in_body).unwrap().station,
        loaded.in_frame(&frame, &StationFrame::BODY).station,
    );
}

#[test]
fn nothing_or_a_negative_mass_has_no_centre_of_gravity() {
    assert_eq!(MassStation::combined(&[]), Err(MbdError::NoAttachedMass));
    let negative = MassStation {
        mass_kg: -1.0,
        station: DVec3::ZERO,
    };
    assert!(matches!(
        MassStation::combined(&[negative]),
        Err(MbdError::InvalidStationFrame {
            field: "mass_kg",
            ..
        })
    ));
}
