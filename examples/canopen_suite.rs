//! A CANopen test suite built with the SDO and NMT steps, printed as the JSON
//! AmpTrace loads:
//!
//! ```text
//! cargo run --example canopen_suite > canopen_suite.json
//! ```
//!
//! The objects it uses are standard communication objects (CiA 301), defined
//! here with `sdo_object!`; a device's own objects are defined the same way,
//! or generated from its object dictionary. The expected values are those of
//! the WattStep `canopen` reference bin; change them for another device.

use hil_test_protocol::*;

sdo_object!(
    /// 0x1001: a bit per error class, 0 when there is no error.
    ErrorRegister, 0x1001, 0, U8
);
sdo_object!(DeviceName, 0x1008, 0, VisibleString);
sdo_object!(
    /// 0x1017: the heartbeat period in ms.
    ProducerHeartbeatTime, 0x1017, 0, U16
);
sdo_object!(VendorId, 0x1018, 1, U32);
sdo_object!(ProductCode, 0x1018, 2, U32);
sdo_object!(RevisionNumber, 0x1018, 3, U32);

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
            assert_sdo(DUT, VendorId(0), SDO_TIMEOUT_MS),
            assert_sdo(DUT, ProductCode(1), SDO_TIMEOUT_MS),
            assert_sdo(DUT, RevisionNumber(1), SDO_TIMEOUT_MS),
            assert_sdo(DUT, DeviceName("WattStep inverter".into()), SDO_TIMEOUT_MS),
        ],
        teardown: vec![],
    }
}

fn heartbeat_time_is_read_only() -> TestCase {
    TestCase {
        name: "Heartbeat time is read-only".into(),
        setup: vec![],
        steps: vec![
            assert_sdo(DUT, ProducerHeartbeatTime(1000), SDO_TIMEOUT_MS),
            sdo_write_refused(
                DUT,
                ProducerHeartbeatTime(500),
                sdo_abort::READ_ONLY,
                SDO_TIMEOUT_MS,
            ),
            // The refused write must not have changed it.
            assert_sdo(DUT, ProducerHeartbeatTime(1000), SDO_TIMEOUT_MS),
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
            assert_sdo_condition::<ErrorRegister>(DUT, Condition::Equals(0.0), SDO_TIMEOUT_MS),
        ],
        // Leave the node running whatever happened above.
        teardown: vec![nmt(DUT, NmtCommand::Start)],
    }
}
