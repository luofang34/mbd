use super::*;

fn deg(d: f64) -> f64 {
    d.to_radians()
}

#[test]
fn a_non_linear_dial_inverts() {
    // Expanded below 100, compressed above, as on a light-aircraft ASI.
    let scale = DialScale::new(&[
        (0.0, 0.0),
        (40.0, deg(30.0)),
        (100.0, deg(180.0)),
        (200.0, deg(330.0)),
    ])
    .unwrap();
    for value in [0.0, 20.0, 40.0, 75.0, 100.0, 160.0, 200.0] {
        assert!((scale.value_at(scale.angle_rad(value)) - value).abs() < 1e-12);
    }
    assert_eq!(scale.angle_rad(250.0), deg(330.0));
    assert_eq!(scale.value_range(), (0.0, 200.0));
    assert_eq!(scale.angle_range_rad(), (0.0, deg(330.0)));
}

#[test]
fn rejects_non_monotone_dials() {
    assert!(matches!(
        DialScale::new(&[(0.0, 0.0)]),
        Err(AirdataError::TooFewPoints { .. })
    ));
    assert!(matches!(
        DialScale::new(&[(0.0, 0.0), (1.0, 1.0), (2.0, 1.0)]),
        Err(AirdataError::NotIncreasing { index: 2, .. })
    ));
    assert!(matches!(
        DialScale::new(&[(0.0, 0.0), (0.0, 1.0)]),
        Err(AirdataError::NotIncreasing { index: 1, .. })
    ));
}
