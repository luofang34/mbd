use super::*;

#[test]
fn interpolates_and_holds_ends() {
    let table = PiecewiseLinear::new("t", &[(0.0, 0.0), (10.0, 100.0), (20.0, 120.0)]).unwrap();
    assert_eq!(table.eval(-5.0), 0.0);
    assert_eq!(table.eval(5.0), 50.0);
    assert_eq!(table.eval(15.0), 110.0);
    assert_eq!(table.eval(25.0), 120.0);
    assert!(table.is_strictly_increasing());
    assert_eq!(table.inverse(110.0), 15.0);
    assert_eq!(table.inverse(500.0), 20.0);
    assert_eq!(table.domain(), (0.0, 20.0));
}

#[test]
fn a_single_point_is_a_constant() {
    let table = PiecewiseLinear::new("t", &[(3.0, 7.0)]).unwrap();
    assert_eq!(table.eval(-1.0), 7.0);
    assert_eq!(table.eval(9.0), 7.0);
}

#[test]
fn rejects_bad_tables() {
    assert!(matches!(
        PiecewiseLinear::new("t", &[]),
        Err(AirdataError::TooFewPoints { .. })
    ));
    assert!(matches!(
        PiecewiseLinear::new("t", &[(0.0, 0.0), (0.0, 1.0)]),
        Err(AirdataError::NotIncreasing { index: 1, .. })
    ));
    assert!(matches!(
        PiecewiseLinear::new("t", &[(0.0, f64::NAN)]),
        Err(AirdataError::NonFinite { .. })
    ));
    let many: Vec<(f64, f64)> = (0..17).map(|i| (f64::from(i), 0.0)).collect();
    assert!(matches!(
        PiecewiseLinear::new("t", &many),
        Err(AirdataError::TooManyPoints { count: 17, .. })
    ));
}
