//! A CANopen test suite built with the SDO and NMT steps, printed as the JSON
//! AmpTrace loads:
//!
//! ```text
//! cargo run --example canopen_suite > canopen_suite.json
//! ```
//!
//! It only uses the standard communication objects in `cia301`, so it runs
//! against any CANopen node. The expected values are those of the WattStep
//! `canopen` reference bin; change them for another device.

use hil_test_protocol::*;

/// The node under test.
const DUT: u8 = 1;
/// How long to wait for each SDO response.
const SDO_TIMEOUT_MS: u64 = 300;

fn main() {
    let suite = TestSuite {
        name: "CANopen communication objects".into(),
        tests: vec![
            identity(),
            heartbeat_time_is_read_only(),
            nmt_stop_and_start(),
        ],
    };
    println!(
        "{}",
        serde_json::to_string_pretty(&suite).expect("a test suite serializes")
    );
}

fn identity() -> TestCase {
    TestCase {
        name: "Identity".into(),
        setup: vec![],
        steps: vec![
            assert_sdo(DUT, cia301::VendorId(0), SDO_TIMEOUT_MS),
            assert_sdo(DUT, cia301::ProductCode(1), SDO_TIMEOUT_MS),
            assert_sdo(DUT, cia301::RevisionNumber(1), SDO_TIMEOUT_MS),
            assert_sdo(
                DUT,
                cia301::DeviceName("WattStep inverter".into()),
                SDO_TIMEOUT_MS,
            ),
        ],
        teardown: vec![],
    }
}

fn heartbeat_time_is_read_only() -> TestCase {
    TestCase {
        name: "Heartbeat time is read-only".into(),
        setup: vec![],
        steps: vec![
            assert_sdo(DUT, cia301::ProducerHeartbeatTime(1000), SDO_TIMEOUT_MS),
            sdo_write_refused(
                DUT,
                cia301::ProducerHeartbeatTime(500),
                sdo_abort::READ_ONLY,
                SDO_TIMEOUT_MS,
            ),
            // The refused write must not have changed it.
            assert_sdo(DUT, cia301::ProducerHeartbeatTime(1000), SDO_TIMEOUT_MS),
        ],
        teardown: vec![],
    }
}

fn nmt_stop_and_start() -> TestCase {
    TestCase {
        name: "NMT stop and start".into(),
        setup: vec![],
        steps: vec![
            nmt(DUT, NmtCommand::Stop),
            delay(200),
            nmt(DUT, NmtCommand::Start),
            delay(200),
            // SDO works again once the node is out of Stopped.
            assert_sdo_condition::<cia301::ErrorRegister>(
                DUT,
                Condition::Equals(0.0),
                SDO_TIMEOUT_MS,
            ),
        ],
        // Leave the node running whatever happened above.
        teardown: vec![nmt(DUT, NmtCommand::Start)],
    }
}
