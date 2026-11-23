use mbd_core::JointStopSpec;
use serde_json::json;

use super::JointRecord;

fn revolute(extra: serde_json::Value) -> serde_json::Value {
    let mut body = json!({
        "axis_local": [0.0, 0.0, 1.0],
        "min_rad": -1.0,
        "max_rad": 1.0,
        "max_torque_nm": 5.0,
        "damping_nm_s_rad": 0.1
    });
    if let (Some(map), Some(more)) = (body.as_object_mut(), extra.as_object()) {
        map.extend(more.clone());
    }
    json!({ "Revolute": body })
}

#[test]
fn omitted_stop_uses_the_default_and_is_not_written_back() {
    let value = revolute(json!({}));
    let record: JointRecord = serde_json::from_value(value).unwrap();
    let (spec, _) = record.to_core_joint("hinge").unwrap();
    assert_eq!(spec.stop, JointStopSpec::DEFAULT);
    let written = serde_json::to_value(record).unwrap();
    assert!(written["Revolute"].get("stop").is_none(), "{written}");
}

#[test]
fn declared_stop_reaches_the_joint_spec() {
    let stop = json!({ "stop": { "natural_frequency_rad_s": 12.0, "damping_ratio": 0.4 } });
    let record: JointRecord = serde_json::from_value(revolute(stop)).unwrap();
    let (spec, _) = record.to_core_joint("hinge").unwrap();
    assert_eq!(
        spec.stop,
        JointStopSpec {
            natural_frequency_rad_s: 12.0,
            damping_ratio: 0.4
        }
    );
}

#[test]
fn nonpositive_stop_frequency_is_refused() {
    let stop = json!({ "stop": { "natural_frequency_rad_s": 0.0, "damping_ratio": 0.4 } });
    let record: JointRecord = serde_json::from_value(revolute(stop)).unwrap();
    assert!(record.to_core_joint("hinge").is_err());
}
