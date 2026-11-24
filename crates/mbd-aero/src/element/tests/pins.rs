//! Element-level pins: each model at flows spanning its regimes, against
//! values recorded from the previous implementation with the same
//! parameters. The arithmetic is unchanged, so agreement is to rounding.

use super::*;

const REL: f64 = 1e-15;

const CAPSULE0: Pin = (
    [-4.694, 0.0, 0.0],
    [
        -149.33515856631323,
        -0.32068288230509984,
        0.21378858820339988,
    ],
    [-0.0, 0.020048507609854938, -0.0150363807073912],
    18334.29981163782,
);
const BODYFIN0: Pin = (
    [-20.0, 0.0, 0.0],
    [-21.655801579694057, -4.653904465905723, 10.631773738197655],
    [5.387708421503692, 73.77757307917194, -59.015926377223764],
    56176.007400717026,
);
const WING0: Pin = (
    [1.2, 3.0, -0.4],
    [-432.28314351509925, 0.0, -368.8524887299258],
    [0.0, -341.17837893122976, -0.0],
    0.0,
);
const CHUTE0: Pin = (
    [-1.5, 0.0, 0.2],
    [-535.1081178951753, -37.83304314200058, 25.290837263577547],
    [-0.895279667605389, 1.790559335210778, -1.3429195014080835],
    0.0,
);
const DUMMY0: Pin = (
    [0.0, 0.1, -0.2],
    [
        -6.2943193335516066,
        -0.25177277334206427,
        0.16784851556137617,
    ],
    [
        -4.9439017310805335e-5,
        9.887803462161067e-5,
        -7.4158525966208e-5,
    ],
    0.0,
);
const CAPSULE1: Pin = (
    [-4.694, 0.0, 0.0],
    [-503094.945463059, -2079.790108795378, -5199.475271988445],
    [-0.0, 2832.97777541142, -2124.733331558565],
    1414.326230067211,
);
const BODYFIN1: Pin = (
    [-20.0, 0.0, 0.0],
    [-147538.8537503463, -21816.14676867525, -77775.94977810558],
    [24971.106483962052, 365863.774602559, -291824.5180942323],
    4333.473412322608,
);
const WING1: Pin = (
    [1.2, 3.0, -0.4],
    [94912.492120254, 0.0, -6460568.364497352],
    [0.0, -1610608.2908792952, -0.0],
    0.0,
);
const CHUTE1: Pin = (
    [-1.5, 0.0, 0.2],
    [-3148869.5179547216, -197638.02608456282, -501576.8196815979],
    [-126462.60417561399, 252925.20835122798, -189693.90626342097],
    0.0,
);
const DUMMY1: Pin = (
    [0.0, 0.1, -0.2],
    [
        -29647.538078310437,
        -1185.9015231324174,
        -2964.7538078310436,
    ],
    [-6.986038063543695, 13.97207612708739, -10.479057095315543],
    0.0,
);
const CAPSULE2: Pin = (
    [-4.694, 0.0, 0.0],
    [-1021583.3984031047, 5490.645887219799, -10981.291774439598],
    [-0.0, 1876.0073083069278, -1407.0054812301958],
    3285.59219420366,
);
const BODYFIN2: Pin = (
    [-20.0, 0.0, 0.0],
    [-472035.4360752062, 139591.6449047394, -173216.53677160764],
    [44391.94979030241, 622822.656828288, -497684.3252282942],
    10067.003011490247,
);
const WING2: Pin = (
    [1.2, 3.0, -0.4],
    [345092.35343985434, 0.0, -16836606.719870493],
    [0.0, -2823849.199198947, -0.0],
    0.0,
);
const CHUTE2: Pin = (
    [-1.5, 0.0, 0.2],
    [-5414288.755315212, 537410.6371595578, -1062423.7154534112],
    [-83765.9219685024, 167531.8439370048, -125648.8829527536],
    0.0,
);
const DUMMY2: Pin = (
    [0.0, 0.1, -0.2],
    [-51830.323654541295, 3141.231736638866, -6282.463473277732],
    [-4.6261776485045125, 9.252355297009025, -6.939266472756769],
    0.0,
);
const ROTOR_HOVER: Pin = (
    [0.1, -0.05, -0.3],
    [
        0.014197811098447095,
        0.07152091115058803,
        -3.9727610215439384,
    ],
    [
        -0.053110147806821446,
        -0.20031626560319185,
        -0.39004882526634355,
    ],
    0.0,
);
const ROTOR_SEA: Pin = (
    [0.1, -0.05, -0.3],
    [-400.728748074112, 2.4616451190301833, -2157.7887114824766],
    [682.0992194883161, 22.124969100601344, -165.19451659386021],
    0.0,
);
const PANEL_ORBIT: Pin = (
    [0.0, 2.5, 0.1],
    [
        -0.006963601024846662,
        -0.0001270770665306433,
        6.238802971814518e-5,
    ],
    [0.0, 0.0, 0.0],
    3.9186520391852517,
);
const PANEL_THIN: Pin = (
    [0.0, 2.5, 0.1],
    [-58.393010235698675, -2.154427601682953, -4.609944517308955],
    [0.0, 0.0, 0.0],
    31962.54839223524,
);

fn flows() -> [LinkFlow<'static>; 3] {
    let thin = AmbientGas::earth_air(0.02, 240.0).unwrap();
    let at = |v: DVec3, w: DVec3, ambient| LinkFlow {
        velocity_air_link_m_s: v,
        angular_rate_link_rad_s: w,
        ambient,
        controls: &CONTROLS,
    };
    [
        at(DVec3::new(7_500.0, 300.0, -200.0), rate(), thin),
        at(DVec3::new(250.0, 10.0, 25.0), rate(), sea()),
        at(DVec3::new(330.0, -20.0, 40.0), rate() * 0.5, sea()),
    ]
}

fn wrench(kind: AeroElementKind, flow: &LinkFlow<'_>) -> AeroWrench {
    AeroElement::new(ComponentId(0), kind)
        .loads(flow, &CHUTE_STATE, &CHUTE_MODES)
        .unwrap()
        .wrench
}

#[test]
fn continuum_elements_match_their_recorded_loads() {
    let pins = [
        [CAPSULE0, BODYFIN0, WING0, CHUTE0, DUMMY0],
        [CAPSULE1, BODYFIN1, WING1, CHUTE1, DUMMY1],
        [CAPSULE2, BODYFIN2, WING2, CHUTE2, DUMMY2],
    ];
    for (flow, row) in flows().iter().zip(&pins) {
        let kinds = [capsule(), body_fin(), wing(), chute(), dummy()];
        for (kind, pin) in kinds.into_iter().zip(row) {
            let label = format!("{kind:?}");
            assert_pin(&label, &wrench(kind, flow), pin, REL);
        }
    }
}

#[test]
fn rotor_and_rarefied_panel_match_their_recorded_loads() {
    let mars = AmbientGas::mars_air(610.0, 215.0).unwrap();
    let hover = LinkFlow {
        velocity_air_link_m_s: DVec3::new(3.0, -1.0, 0.5),
        angular_rate_link_rad_s: rate(),
        ambient: mars,
        controls: &CONTROLS,
    };
    assert_pin("rotor hover", &wrench(rotor(), &hover), &ROTOR_HOVER, REL);
    let [thin, at_sea, _] = flows();
    assert_pin("rotor sea", &wrench(rotor(), &at_sea), &ROTOR_SEA, REL);
    let orbit = LinkFlow {
        velocity_air_link_m_s: DVec3::new(7_600.0, 150.0, -900.0),
        ambient: AmbientGas::earth_air(1.0e-5, 900.0).unwrap(),
        ..hover
    };
    assert_pin("panel orbit", &wrench(panel(), &orbit), &PANEL_ORBIT, REL);
    assert_pin("panel thin", &wrench(panel(), &thin), &PANEL_THIN, REL);
}
