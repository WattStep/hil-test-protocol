//! CANopen test steps: SDO reads and writes, and NMT commands.
//!
//! A drive's process data (PDOs), heartbeat and EMCY are ordinary frames the DBC
//! describes, so they use `send_can` and the signal steps. The steps here are for
//! what a DBC cannot express: reading and writing objects in a node's object
//! dictionary (SDO), and commanding its NMT state.
//!
//! A test does not spell out indices or data types. Each object is a Rust type
//! implementing [`SdoObject`], and a value of that type is what is written or
//! expected:
//!
//! ```
//! use hil_test_protocol::*;
//!
//! sdo_enum!(
//!     /// 0x6060 Modes of operation
//!     ModesOfOperation, 0x6060, 0, I8 { ProfilePosition = 1, ProfileVelocity = 3 }
//! );
//! const DUT: u8 = 1;
//!
//! let steps = vec![
//!     nmt(DUT, NmtCommand::Start),
//!     sdo_write(DUT, ModesOfOperation::ProfileVelocity, 200),
//!     assert_sdo(DUT, cia301::ProductCode(1), 200),
//!     sdo_write_refused(DUT, cia301::DeviceType(0), sdo_abort::READ_ONLY, 200),
//! ];
//! ```

use serde::{Deserialize, Serialize};

use crate::{Condition, TestStep};

/// The type of an object's value, which sets its width on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SdoDataType {
    U8,
    U16,
    U32,
    I8,
    I16,
    I32,
    F32,
    VisibleString,
    Bytes,
}

/// A value read from or written to an object.
///
/// Typed because an SDO write carries its size, and a node refuses a write of
/// the wrong width (abort [`sdo_abort::LENGTH_MISMATCH`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SdoValue {
    U8(u8),
    U16(u16),
    U32(u32),
    I8(i8),
    I16(i16),
    I32(i32),
    F32(f32),
    VisibleString(String),
    Bytes(Vec<u8>),
}

impl SdoValue {
    pub fn data_type(&self) -> SdoDataType {
        match self {
            SdoValue::U8(_) => SdoDataType::U8,
            SdoValue::U16(_) => SdoDataType::U16,
            SdoValue::U32(_) => SdoDataType::U32,
            SdoValue::I8(_) => SdoDataType::I8,
            SdoValue::I16(_) => SdoDataType::I16,
            SdoValue::I32(_) => SdoDataType::I32,
            SdoValue::F32(_) => SdoDataType::F32,
            SdoValue::VisibleString(_) => SdoDataType::VisibleString,
            SdoValue::Bytes(_) => SdoDataType::Bytes,
        }
    }

    /// The value as sent on the bus. CANopen is little-endian.
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            SdoValue::U8(v) => v.to_le_bytes().to_vec(),
            SdoValue::U16(v) => v.to_le_bytes().to_vec(),
            SdoValue::U32(v) => v.to_le_bytes().to_vec(),
            SdoValue::I8(v) => v.to_le_bytes().to_vec(),
            SdoValue::I16(v) => v.to_le_bytes().to_vec(),
            SdoValue::I32(v) => v.to_le_bytes().to_vec(),
            SdoValue::F32(v) => v.to_le_bytes().to_vec(),
            SdoValue::VisibleString(s) => s.as_bytes().to_vec(),
            SdoValue::Bytes(b) => b.clone(),
        }
    }

    /// Decodes `data` as `data_type`. A numeric type must match the length
    /// exactly: a mismatch means the object is not the type the caller assumed.
    pub fn from_bytes(data_type: SdoDataType, data: &[u8]) -> Result<SdoValue, String> {
        Ok(match data_type {
            SdoDataType::U8 => SdoValue::U8(u8::from_le_bytes(exact(data)?)),
            SdoDataType::U16 => SdoValue::U16(u16::from_le_bytes(exact(data)?)),
            SdoDataType::U32 => SdoValue::U32(u32::from_le_bytes(exact(data)?)),
            SdoDataType::I8 => SdoValue::I8(i8::from_le_bytes(exact(data)?)),
            SdoDataType::I16 => SdoValue::I16(i16::from_le_bytes(exact(data)?)),
            SdoDataType::I32 => SdoValue::I32(i32::from_le_bytes(exact(data)?)),
            SdoDataType::F32 => SdoValue::F32(f32::from_le_bytes(exact(data)?)),
            SdoDataType::VisibleString => {
                // Some devices pad a string object with NULs to its full size.
                let text = data.split(|&b| b == 0).next().unwrap_or_default();
                SdoValue::VisibleString(String::from_utf8_lossy(text).into_owned())
            }
            SdoDataType::Bytes => SdoValue::Bytes(data.to_vec()),
        })
    }

    /// The value as a number for a [`Condition`], if it is numeric. A `u32` or
    /// `i32` above 2^24 loses precision as `f32`: compare those with
    /// [`SdoExpect::Equals`] instead.
    pub fn as_f32(&self) -> Option<f32> {
        match self {
            SdoValue::U8(v) => Some(f32::from(*v)),
            SdoValue::U16(v) => Some(f32::from(*v)),
            SdoValue::U32(v) => Some(*v as f32),
            SdoValue::I8(v) => Some(f32::from(*v)),
            SdoValue::I16(v) => Some(f32::from(*v)),
            SdoValue::I32(v) => Some(*v as f32),
            SdoValue::F32(v) => Some(*v),
            SdoValue::VisibleString(_) | SdoValue::Bytes(_) => None,
        }
    }
}

fn exact<const N: usize>(data: &[u8]) -> Result<[u8; N], String> {
    data.try_into()
        .map_err(|_| format!("Expected {N} bytes, the node sent {}", data.len()))
}

/// One object in a node's object dictionary, as a Rust type: the type knows the
/// object's index, sub-index and data type, and a value of it is a value of the
/// object. Tests then name objects and values instead of numbers.
///
/// Define objects with [`sdo_object!`](crate::sdo_object) and
/// [`sdo_enum!`](crate::sdo_enum), or generate the impls from the device's
/// object dictionary. The standard communication objects are in [`cia301`].
pub trait SdoObject {
    /// Shown in test reports instead of the index.
    const NAME: &'static str;
    const INDEX: u16;
    const SUB: u8;
    const DATA_TYPE: SdoDataType;
    fn value(&self) -> SdoValue;
}

/// Which object of which node a step reads or writes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SdoAddress {
    /// Node id, 1-127.
    pub node_id: u8,
    pub index: u16,
    pub sub: u8,
    /// The object's name, for test reports.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub object_name: Option<String>,
}

impl SdoAddress {
    fn of<O: SdoObject>(node_id: u8) -> SdoAddress {
        SdoAddress {
            node_id,
            index: O::INDEX,
            sub: O::SUB,
            object_name: Some(O::NAME.to_string()),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SdoWriteAction {
    pub address: SdoAddress,
    pub value: SdoValue,
    /// `None`: the write must succeed. `Some(code)`: the node must refuse it
    /// with this abort code, and the step fails if it accepts the write or
    /// aborts with another code.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect_abort: Option<u32>,
    /// How long to wait for each response from the node.
    pub timeout_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SdoReadAssertion {
    pub address: SdoAddress,
    pub expect: SdoExpect,
    /// How long to wait for each response from the node.
    pub timeout_ms: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// What an SDO read must return for the step to pass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SdoExpect {
    /// Exactly this value. Its type is the type the object is read as.
    Equals(SdoValue),
    /// A numeric value meeting `condition`, compared as `f32`.
    /// `Condition::ChangesBy` never holds: a read has no earlier value.
    Numeric {
        data_type: SdoDataType,
        condition: Condition,
    },
    /// The node refuses the read with this abort code.
    Abort(u32),
}

impl SdoExpect {
    /// The type to read the object as. `Bytes` when only an abort is expected.
    pub fn data_type(&self) -> SdoDataType {
        match self {
            SdoExpect::Equals(value) => value.data_type(),
            SdoExpect::Numeric { data_type, .. } => *data_type,
            SdoExpect::Abort(_) => SdoDataType::Bytes,
        }
    }

    /// Whether a value that was read meets the expectation. Always false for
    /// `Abort`: reading a value means the node did not refuse.
    pub fn is_met_by(&self, observed: &SdoValue) -> bool {
        match self {
            SdoExpect::Equals(expected) => observed == expected,
            SdoExpect::Numeric { condition, .. } => observed
                .as_f32()
                .is_some_and(|number| condition.evaluate(number, None)),
            SdoExpect::Abort(_) => false,
        }
    }
}

/// An NMT command: moves a node between its network states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NmtCommand {
    /// To Operational: PDOs are exchanged.
    Start,
    /// To Stopped: only NMT and heartbeat.
    Stop,
    /// To Pre-operational: SDO works, PDOs do not.
    PreOperational,
    /// Reset the application, as after power-on.
    ResetNode,
    /// Reset the communication parameters.
    ResetCommunication,
}

impl NmtCommand {
    /// The command specifier byte of the NMT frame.
    pub fn specifier(self) -> u8 {
        match self {
            NmtCommand::Start => 0x01,
            NmtCommand::Stop => 0x02,
            NmtCommand::PreOperational => 0x80,
            NmtCommand::ResetNode => 0x81,
            NmtCommand::ResetCommunication => 0x82,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NmtAction {
    /// Node id, 1-127, or [`ALL_NODES`].
    pub node_id: u8,
    pub command: NmtCommand,
}

/// The node id that addresses an NMT command to every node on the bus.
pub const ALL_NODES: u8 = 0;

/// Write `object`'s value to that object on `node_id`. Fails if the node
/// refuses the write or does not answer within `timeout_ms`.
pub fn sdo_write<O: SdoObject>(node_id: u8, object: O, timeout_ms: u64) -> TestStep {
    TestStep::SdoWrite(SdoWriteAction {
        address: SdoAddress::of::<O>(node_id),
        value: object.value(),
        expect_abort: None,
        timeout_ms,
    })
}

/// Write `object`'s value and require the node to refuse it with `abort_code`
/// (see [`sdo_abort`]). Fails if the write is accepted.
pub fn sdo_write_refused<O: SdoObject>(
    node_id: u8,
    object: O,
    abort_code: u32,
    timeout_ms: u64,
) -> TestStep {
    TestStep::SdoWrite(SdoWriteAction {
        address: SdoAddress::of::<O>(node_id),
        value: object.value(),
        expect_abort: Some(abort_code),
        timeout_ms,
    })
}

/// Read the object from `node_id` and require exactly `expected`'s value.
pub fn assert_sdo<O: SdoObject>(node_id: u8, expected: O, timeout_ms: u64) -> TestStep {
    TestStep::SdoRead(SdoReadAssertion {
        address: SdoAddress::of::<O>(node_id),
        expect: SdoExpect::Equals(expected.value()),
        timeout_ms,
        description: None,
    })
}

/// Read object `O` from `node_id` and require its numeric value to meet `cond`,
/// e.g. `assert_sdo_condition::<MaxTorque>(DUT, Condition::InRange(0.0, 2000.0), 200)`.
pub fn assert_sdo_condition<O: SdoObject>(
    node_id: u8,
    cond: Condition,
    timeout_ms: u64,
) -> TestStep {
    TestStep::SdoRead(SdoReadAssertion {
        address: SdoAddress::of::<O>(node_id),
        expect: SdoExpect::Numeric {
            data_type: O::DATA_TYPE,
            condition: cond,
        },
        timeout_ms,
        description: None,
    })
}

/// Read object `O` from `node_id` and require the node to refuse the read with
/// `abort_code` (see [`sdo_abort`]).
pub fn assert_sdo_read_refused<O: SdoObject>(
    node_id: u8,
    abort_code: u32,
    timeout_ms: u64,
) -> TestStep {
    TestStep::SdoRead(SdoReadAssertion {
        address: SdoAddress::of::<O>(node_id),
        expect: SdoExpect::Abort(abort_code),
        timeout_ms,
        description: None,
    })
}

/// Send an NMT command to `node_id`, or to every node with [`ALL_NODES`]. The
/// command has no reply: check the resulting state with `assert_signal` on the
/// node's heartbeat signal, allowing at least one heartbeat period.
pub fn nmt(node_id: u8, command: NmtCommand) -> TestStep {
    TestStep::Nmt(NmtAction { node_id, command })
}

/// Defines an object whose value is a number, a string or bytes, as a newtype
/// over that value: `sdo_object!(MaxTorque, 0x6072, 0, U16);` gives
/// `MaxTorque(pub u16)`, used as `sdo_write(node, MaxTorque(1500), 200)`.
///
/// The last argument is the [`SdoDataType`] variant.
#[macro_export]
macro_rules! sdo_object {
    ($(#[$meta:meta])* $name:ident, $index:expr, $sub:expr, $data_type:ident) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq)]
        pub struct $name(pub $crate::sdo_object!(@rust_type $data_type));

        impl $crate::SdoObject for $name {
            const NAME: &'static str = stringify!($name);
            const INDEX: u16 = $index;
            const SUB: u8 = $sub;
            const DATA_TYPE: $crate::SdoDataType = $crate::SdoDataType::$data_type;
            fn value(&self) -> $crate::SdoValue {
                $crate::SdoValue::$data_type(self.0.clone())
            }
        }
    };
    (@rust_type U8) => { u8 };
    (@rust_type U16) => { u16 };
    (@rust_type U32) => { u32 };
    (@rust_type I8) => { i8 };
    (@rust_type I16) => { i16 };
    (@rust_type I32) => { i32 };
    (@rust_type F32) => { f32 };
    (@rust_type VisibleString) => { ::std::string::String };
    (@rust_type Bytes) => { ::std::vec::Vec<u8> };
}

/// Defines an object whose values have names, as an enum:
/// `sdo_enum!(ModesOfOperation, 0x6060, 0, I8 { ProfilePosition = 1, ProfileVelocity = 3 });`
/// used as `sdo_write(node, ModesOfOperation::ProfileVelocity, 200)`.
///
/// The type before the braces is the integer [`SdoDataType`] variant the
/// object has on the wire.
#[macro_export]
macro_rules! sdo_enum {
    (
        $(#[$meta:meta])* $name:ident, $index:expr, $sub:expr,
        $data_type:ident { $($(#[$variant_meta:meta])* $variant:ident = $raw:expr),+ $(,)? }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum $name {
            $($(#[$variant_meta])* $variant = $raw),+
        }

        impl $crate::SdoObject for $name {
            const NAME: &'static str = stringify!($name);
            const INDEX: u16 = $index;
            const SUB: u8 = $sub;
            const DATA_TYPE: $crate::SdoDataType = $crate::SdoDataType::$data_type;
            fn value(&self) -> $crate::SdoValue {
                $crate::SdoValue::$data_type(*self as $crate::sdo_object!(@rust_type $data_type))
            }
        }
    };
}

/// SDO abort codes from CiA 301, for `sdo_write_refused` and
/// `assert_sdo_read_refused`, and what each means.
pub mod sdo_abort {
    pub const TOGGLE_BIT: u32 = 0x0503_0000;
    pub const TIMEOUT: u32 = 0x0504_0000;
    pub const INVALID_COMMAND: u32 = 0x0504_0001;
    pub const INVALID_BLOCK_SIZE: u32 = 0x0504_0002;
    pub const INVALID_SEQUENCE_NUMBER: u32 = 0x0504_0003;
    pub const CRC_ERROR: u32 = 0x0504_0004;
    pub const OUT_OF_MEMORY: u32 = 0x0504_0005;
    pub const UNSUPPORTED_ACCESS: u32 = 0x0601_0000;
    /// Reading a write-only object.
    pub const WRITE_ONLY: u32 = 0x0601_0001;
    /// Writing a read-only object.
    pub const READ_ONLY: u32 = 0x0601_0002;
    pub const NO_SUCH_OBJECT: u32 = 0x0602_0000;
    pub const NOT_MAPPABLE: u32 = 0x0604_0041;
    pub const PDO_TOO_LONG: u32 = 0x0604_0042;
    pub const PARAMETER_INCOMPATIBLE: u32 = 0x0604_0043;
    pub const DEVICE_INCOMPATIBLE: u32 = 0x0604_0047;
    pub const HARDWARE_ERROR: u32 = 0x0606_0000;
    /// The value written has the wrong width for the object.
    pub const LENGTH_MISMATCH: u32 = 0x0607_0010;
    pub const LENGTH_TOO_HIGH: u32 = 0x0607_0012;
    pub const LENGTH_TOO_LOW: u32 = 0x0607_0013;
    pub const NO_SUCH_SUB_INDEX: u32 = 0x0609_0011;
    pub const INVALID_VALUE: u32 = 0x0609_0030;
    pub const VALUE_TOO_HIGH: u32 = 0x0609_0031;
    pub const VALUE_TOO_LOW: u32 = 0x0609_0032;
    pub const MAX_LESS_THAN_MIN: u32 = 0x0609_0036;
    pub const RESOURCE_NOT_AVAILABLE: u32 = 0x060A_0023;
    pub const GENERAL_ERROR: u32 = 0x0800_0000;
    pub const CANNOT_STORE: u32 = 0x0800_0020;
    pub const LOCAL_CONTROL: u32 = 0x0800_0021;
    /// Refused in the present device state, e.g. a parameter change while the
    /// drive is enabled.
    pub const DEVICE_STATE: u32 = 0x0800_0022;
    pub const NO_OBJECT_DICTIONARY: u32 = 0x0800_0023;
    pub const NO_DATA: u32 = 0x0800_0024;

    /// What an abort code means.
    pub fn meaning(code: u32) -> &'static str {
        match code {
            TOGGLE_BIT => "toggle bit not alternated",
            TIMEOUT => "SDO protocol timed out",
            INVALID_COMMAND => "command specifier not valid or unknown",
            INVALID_BLOCK_SIZE => "invalid block size",
            INVALID_SEQUENCE_NUMBER => "invalid sequence number",
            CRC_ERROR => "CRC error",
            OUT_OF_MEMORY => "out of memory",
            UNSUPPORTED_ACCESS => "unsupported access to an object",
            WRITE_ONLY => "attempt to read a write-only object",
            READ_ONLY => "attempt to write a read-only object",
            NO_SUCH_OBJECT => "object does not exist in the object dictionary",
            NOT_MAPPABLE => "object cannot be mapped to the PDO",
            PDO_TOO_LONG => "the mapped objects would exceed the PDO length",
            PARAMETER_INCOMPATIBLE => "general parameter incompatibility",
            DEVICE_INCOMPATIBLE => "general internal incompatibility in the device",
            HARDWARE_ERROR => "access failed due to a hardware error",
            LENGTH_MISMATCH => {
                "data type does not match, length of service parameter does not match"
            }
            LENGTH_TOO_HIGH => "data type does not match, length of service parameter too high",
            LENGTH_TOO_LOW => "data type does not match, length of service parameter too low",
            NO_SUCH_SUB_INDEX => "sub-index does not exist",
            INVALID_VALUE => "invalid value for parameter",
            VALUE_TOO_HIGH => "value of parameter written too high",
            VALUE_TOO_LOW => "value of parameter written too low",
            MAX_LESS_THAN_MIN => "maximum value is less than minimum value",
            RESOURCE_NOT_AVAILABLE => "resource not available",
            GENERAL_ERROR => "general error",
            CANNOT_STORE => "data cannot be transferred or stored to the application",
            LOCAL_CONTROL => {
                "data cannot be transferred or stored to the application because of local control"
            }
            DEVICE_STATE => {
                "data cannot be transferred or stored to the application because of the present \
                 device state"
            }
            NO_OBJECT_DICTIONARY => "no object dictionary present",
            NO_DATA => "no data available",
            _ => "unknown abort code",
        }
    }
}

/// The communication objects every CANopen device has (CiA 301). Device
/// profile and manufacturer objects are defined by the firmware, from its
/// object dictionary.
pub mod cia301 {
    crate::sdo_object!(
        /// 0x1000: the device profile the device follows.
        DeviceType, 0x1000, 0, U32
    );
    crate::sdo_object!(
        /// 0x1001: a bit per error class, 0 when there is no error.
        ErrorRegister, 0x1001, 0, U8
    );
    crate::sdo_object!(
        /// 0x1008
        DeviceName, 0x1008, 0, VisibleString
    );
    crate::sdo_object!(
        /// 0x1009
        HardwareVersion, 0x1009, 0, VisibleString
    );
    crate::sdo_object!(
        /// 0x100A
        SoftwareVersion, 0x100A, 0, VisibleString
    );
    crate::sdo_object!(
        /// 0x1017: the heartbeat period in ms, 0 when the heartbeat is off.
        ProducerHeartbeatTime, 0x1017, 0, U16
    );
    crate::sdo_object!(
        /// 0x1018 sub 1
        VendorId, 0x1018, 1, U32
    );
    crate::sdo_object!(
        /// 0x1018 sub 2
        ProductCode, 0x1018, 2, U32
    );
    crate::sdo_object!(
        /// 0x1018 sub 3
        RevisionNumber, 0x1018, 3, U32
    );
    crate::sdo_object!(
        /// 0x1018 sub 4
        SerialNumber, 0x1018, 4, U32
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Requirement, TestCase};

    crate::sdo_enum!(
        /// 0x6060 Modes of operation
        ModesOfOperation, 0x6060, 0, I8 { Homing = 6, ProfileVelocity = 3, Manufacturer = -1 }
    );

    #[test]
    fn numeric_values_round_trip_little_endian() {
        let value = SdoValue::U32(0x1234_5678);
        assert_eq!(value.to_bytes(), vec![0x78, 0x56, 0x34, 0x12]);
        assert_eq!(
            SdoValue::from_bytes(SdoDataType::U32, &value.to_bytes()),
            Ok(value)
        );
        assert_eq!(
            SdoValue::from_bytes(SdoDataType::I16, &[0xFE, 0xFF]),
            Ok(SdoValue::I16(-2))
        );
    }

    #[test]
    fn numeric_read_of_wrong_length_is_an_error() {
        assert!(SdoValue::from_bytes(SdoDataType::U16, &[1, 2, 3, 4]).is_err());
    }

    #[test]
    fn visible_string_stops_at_nul_padding() {
        assert_eq!(
            SdoValue::from_bytes(SdoDataType::VisibleString, b"drive\0\0\0"),
            Ok(SdoValue::VisibleString("drive".into()))
        );
    }

    #[test]
    fn object_types_carry_address_type_and_value() {
        let TestStep::SdoWrite(write) = sdo_write(2, cia301::ProducerHeartbeatTime(500), 200)
        else {
            panic!("sdo_write builds an SdoWrite step");
        };
        assert_eq!(
            write.address,
            SdoAddress {
                node_id: 2,
                index: 0x1017,
                sub: 0,
                object_name: Some("ProducerHeartbeatTime".into()),
            }
        );
        assert_eq!(write.value, SdoValue::U16(500));
        assert_eq!(write.expect_abort, None);
    }

    #[test]
    fn enum_objects_write_their_raw_value() {
        assert_eq!(ModesOfOperation::ProfileVelocity.value(), SdoValue::I8(3));
        assert_eq!(ModesOfOperation::Manufacturer.value(), SdoValue::I8(-1));
        assert_eq!(ModesOfOperation::DATA_TYPE, SdoDataType::I8);
    }

    #[test]
    fn equals_compares_large_integers_exactly() {
        // 16_777_217 is not representable as f32, so a Condition could not tell
        // it from 16_777_216.
        let expect = SdoExpect::Equals(SdoValue::U32(16_777_217));
        assert!(expect.is_met_by(&SdoValue::U32(16_777_217)));
        assert!(!expect.is_met_by(&SdoValue::U32(16_777_216)));
    }

    #[test]
    fn numeric_expectation_uses_the_condition() {
        let expect = SdoExpect::Numeric {
            data_type: SdoDataType::U16,
            condition: Condition::InRange(100.0, 200.0),
        };
        assert!(expect.is_met_by(&SdoValue::U16(150)));
        assert!(!expect.is_met_by(&SdoValue::U16(250)));
        assert!(!expect.is_met_by(&SdoValue::VisibleString("150".into())));
    }

    #[test]
    fn canopen_steps_require_canopen_and_list_their_nodes() {
        let test = TestCase {
            name: "canopen".into(),
            setup: vec![nmt(ALL_NODES, NmtCommand::Start)],
            steps: vec![
                sdo_write(2, ModesOfOperation::Homing, 200),
                assert_sdo(1, cia301::ProductCode(1), 200),
                assert_sdo_condition::<cia301::ProducerHeartbeatTime>(
                    1,
                    Condition::GreaterThan(0.0),
                    200,
                ),
            ],
            teardown: vec![],
        };
        assert!(test.infer_requirements().contains(&Requirement::CanOpen));
        assert_eq!(
            test.canopen_nodes().into_iter().collect::<Vec<_>>(),
            vec![1, 2]
        );
    }

    #[test]
    fn canopen_steps_round_trip_through_json() {
        let steps = vec![
            sdo_write_refused(1, cia301::DeviceType(0), sdo_abort::READ_ONLY, 200),
            assert_sdo(1, cia301::DeviceName("drive".into()), 200),
            assert_sdo_read_refused::<cia301::DeviceType>(1, sdo_abort::WRITE_ONLY, 200),
            nmt(1, NmtCommand::PreOperational),
        ];
        let json = serde_json::to_string(&steps).unwrap();
        let back: Vec<TestStep> = serde_json::from_str(&json).unwrap();
        assert_eq!(serde_json::to_string(&back).unwrap(), json);

        let TestStep::SdoWrite(write) = &back[0] else {
            panic!("first step is an SdoWrite");
        };
        assert_eq!(write.expect_abort, Some(sdo_abort::READ_ONLY));
    }
}
