use std::collections::{BTreeSet, HashSet};
use std::path::Path;

use serde::{Deserialize, Serialize};

mod canopen;
pub use canopen::*;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestSuite {
    pub name: String,
    pub tests: Vec<TestCase>,
}

impl TestSuite {
    /// Write the suite to `path` as the JSON AmpTrace loads, creating its directory.
    pub fn write_json(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let path = path.as_ref();
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(self)?)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestCase {
    pub name: String,
    #[serde(default)]
    pub setup: Vec<TestStep>,
    pub steps: Vec<TestStep>,
    #[serde(default)]
    pub teardown: Vec<TestStep>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Requirement {
    Can,
    Rtt,
    /// CANopen master mode, for SDO and NMT steps.
    CanOpen,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "data")]
pub enum TestStep {
    /// Send a CAN message with the given signal values.
    SendCan(CanAction),
    /// Assert that a signal meets a condition within a timeout; fails the test if not.
    AssertSignal(SignalAssertion),
    /// Monitor a signal for the entire duration; fails at the end if any sample violated the condition.
    HoldSignal(SignalHold),
    /// Block until a signal meets a condition. Does not fail on timeout — use when
    /// synchronizing to device state before making assertions.
    WaitForSignal(SignalWait),
    /// Unconditional pause.
    Delay { ms: u64 },
    /// Call a postcard-rpc endpoint on the target over RTT. Requires a debugger connection.
    RpcCall(RpcAction),
    /// Write an object in a CANopen node's object dictionary.
    SdoWrite(SdoWriteAction),
    /// Read an object from a CANopen node and assert on it.
    SdoRead(SdoReadAssertion),
    /// Send an NMT command to a CANopen node, or to all of them.
    Nmt(NmtAction),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcAction {
    /// Endpoint path as registered in the firmware (e.g. "motor/set_velocity").
    pub path: String,
    /// JSON payload matching the endpoint's request type.
    pub payload: serde_json::Value,
    /// How long to wait for a response before failing the step.
    pub timeout_ms: u64,
}

impl TestStep {
    pub fn requirement(&self) -> Option<Requirement> {
        match self {
            TestStep::SendCan(_) => Some(Requirement::Can),
            TestStep::RpcCall(_) => Some(Requirement::Rtt),
            TestStep::SdoWrite(_) | TestStep::SdoRead(_) | TestStep::Nmt(_) => {
                Some(Requirement::CanOpen)
            }
            TestStep::AssertSignal(_)
            | TestStep::HoldSignal(_)
            | TestStep::WaitForSignal(_)
            | TestStep::Delay { .. } => None,
        }
    }

    /// The CANopen node this step addresses, if it addresses one. `None` for
    /// an NMT command to all nodes.
    pub fn canopen_node(&self) -> Option<u8> {
        match self {
            TestStep::SdoWrite(write) => Some(write.address.node_id),
            TestStep::SdoRead(read) => Some(read.address.node_id),
            TestStep::Nmt(action) if action.node_id != ALL_NODES => Some(action.node_id),
            _ => None,
        }
    }
}

impl TestCase {
    pub fn infer_requirements(&self) -> HashSet<Requirement> {
        self.setup
            .iter()
            .chain(&self.steps)
            .chain(&self.teardown)
            .filter_map(|s| s.requirement())
            .collect()
    }

    /// The CANopen nodes the test addresses, so a runner can check they are
    /// all ones it is set up to talk to.
    pub fn canopen_nodes(&self) -> BTreeSet<u8> {
        self.setup
            .iter()
            .chain(&self.steps)
            .chain(&self.teardown)
            .filter_map(|s| s.canopen_node())
            .collect()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CanAction {
    pub message_name: String,
    /// Standard CAN frame ID.
    pub id: u32,
    /// Raw CAN payload bytes, already encoded by the dbc-codegen type.
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalAssertion {
    pub signal_path: String,
    pub condition: Condition,
    pub timeout_ms: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalHold {
    pub signal_path: String,
    pub condition: Condition,
    pub duration_ms: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalWait {
    pub signal_path: String,
    pub condition: Condition,
    pub timeout_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Condition {
    Equals(f32),
    GreaterThan(f32),
    LessThan(f32),
    InRange(f32, f32),
    RelTol(f32, f32), // value, tolerance [ratio]
    /// True when the observed value has changed by at least this delta from the
    /// value recorded at the start of the step.
    ChangesBy(f32),
}

impl Condition {
    pub fn evaluate(&self, observed: f32, baseline: Option<f32>) -> bool {
        match self {
            // 1e-5 tolerance: survives f32 float conversions (f32 epsilon ~1.2e-7)
            // while still being tight enough for integer-valued sensor signals.
            Condition::Equals(expected) => (observed - expected).abs() < 1e-5,
            Condition::GreaterThan(threshold) => observed > *threshold,
            Condition::LessThan(threshold) => observed < *threshold,
            Condition::InRange(lo, hi) => observed >= *lo && observed <= *hi,
            Condition::RelTol(value, tol) => {
                let abs_tol = value.abs() * *tol;
                (observed - *value).abs() <= abs_tol
            }
            Condition::ChangesBy(delta) => match baseline {
                Some(base) => (observed - base).abs() >= *delta,
                None => false,
            },
        }
    }
}

/// Implement on each dbc-codegen TX message type.
///
/// `payload()` returns the raw CAN frame bytes already encoded by the
/// dbc-codegen type (i.e. `self.raw().to_vec()`). `ID` is the standard CAN
/// frame ID from the DBC file.
pub trait CanMessage {
    const NAME: &'static str;
    const ID: u32;
    fn payload(&self) -> Vec<u8>;
}

/// Build a `SendCan` step from any `CanMessage`.
pub fn send_can<M: CanMessage>(msg: M) -> TestStep {
    TestStep::SendCan(CanAction {
        message_name: M::NAME.to_string(),
        id: M::ID,
        payload: msg.payload(),
    })
}

/// Unconditional pause.
pub fn delay(ms: u64) -> TestStep {
    TestStep::Delay { ms }
}

/// Assert that `signal_path` meets `cond` within `timeout_ms`; fails the test if not.
pub fn assert_signal(
    signal_path: &str,
    cond: Condition,
    timeout_ms: u64,
    description: &str,
) -> TestStep {
    TestStep::AssertSignal(SignalAssertion {
        signal_path: signal_path.to_string(),
        condition: cond,
        timeout_ms,
        description: Some(description.to_string()),
    })
}

/// Implemented for each postcard-rpc endpoint type. Ties the request payload
/// type and the path string together so both are verified at compile time.
pub trait RpcEndpoint {
    type Request: Serialize;
    const PATH: &'static str;
}

/// Call a postcard-rpc endpoint on the target over RTT.
pub fn rpc_call<E: RpcEndpoint>(payload: E::Request, timeout_ms: u64) -> TestStep {
    TestStep::RpcCall(RpcAction {
        path: E::PATH.to_string(),
        payload: serde_json::to_value(payload).expect("RPC payload must be serializable"),
        timeout_ms,
    })
}

/// Monitor `signal_path` for the entire `duration_ms`; fails at the end if any sample violated `cond`.
pub fn hold_signal(
    signal_path: &str,
    cond: Condition,
    duration_ms: u64,
    description: &str,
) -> TestStep {
    TestStep::HoldSignal(SignalHold {
        signal_path: signal_path.to_string(),
        condition: cond,
        duration_ms,
        description: Some(description.to_string()),
    })
}

/// Block until `signal_path` meets `cond` within `timeout_ms`. Does not fail on timeout.
pub fn wait_signal(signal_path: &str, cond: Condition, timeout_ms: u64) -> TestStep {
    TestStep::WaitForSignal(SignalWait {
        signal_path: signal_path.to_string(),
        condition: cond,
        timeout_ms,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn condition_equals() {
        assert!(Condition::Equals(1.0).evaluate(1.0, None));
        assert!(!Condition::Equals(1.0).evaluate(2.0, None));
    }

    #[test]
    fn condition_in_range() {
        assert!(Condition::InRange(10.0, 20.0).evaluate(15.0, None));
        assert!(!Condition::InRange(10.0, 20.0).evaluate(25.0, None));
    }

    #[test]
    fn condition_changes_by() {
        assert!(Condition::ChangesBy(5.0).evaluate(10.0, Some(4.0)));
        assert!(!Condition::ChangesBy(5.0).evaluate(10.0, Some(6.0)));
        assert!(!Condition::ChangesBy(5.0).evaluate(10.0, None));
    }

    #[test]
    fn roundtrip_test_suite() {
        let suite = TestSuite {
            name: "smoke".into(),
            tests: vec![TestCase {
                name: "basic".into(),
                setup: vec![],
                steps: vec![
                    TestStep::SendCan(CanAction {
                        message_name: "TemperatureReading".into(),
                        id: 1,
                        payload: vec![0x4B, 0x00],
                    }),
                    TestStep::AssertSignal(SignalAssertion {
                        signal_path: "sensor.temperature".into(),
                        condition: Condition::GreaterThan(500.0),
                        timeout_ms: 500,
                        description: Some("Temperature reading high".into()),
                    }),
                    TestStep::Delay { ms: 100 },
                ],
                teardown: vec![TestStep::SendCan(CanAction {
                    message_name: "TemperatureReading".into(),
                    id: 1,
                    payload: vec![0x00, 0x00],
                })],
            }],
        };

        let json = serde_json::to_string_pretty(&suite).unwrap();
        let roundtripped: TestSuite = serde_json::from_str(&json).unwrap();
        assert_eq!(roundtripped.tests.len(), 1);
        assert_eq!(roundtripped.tests[0].steps.len(), 3);
    }

    #[test]
    fn write_json_creates_the_directory_and_reads_back() {
        let suite = TestSuite {
            name: "written".into(),
            tests: vec![TestCase {
                name: "pause".into(),
                setup: vec![],
                steps: vec![delay(10)],
                teardown: vec![],
            }],
        };
        let dir = std::env::temp_dir().join(format!("hil_test_protocol_{}", std::process::id()));
        let path = dir.join("nested/suite.json");

        suite.write_json(&path).unwrap();
        let back: TestSuite =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(back.name, "written");
        assert_eq!(back.tests[0].steps.len(), 1);

        std::fs::remove_dir_all(dir).unwrap();
    }
}
