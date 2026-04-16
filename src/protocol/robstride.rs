use std::convert::TryInto;
use std::mem::size_of;

use socketcan::{CanFrame, EmbeddedFrame, Frame};

use crate::core::{ActuatorError, ParameterValue, Result};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CommunicationType {
    GetDeviceId = 0,
    OperationControl = 1,
    OperationStatus = 2,
    Enable = 3,
    Disable = 4,
    SetZeroPosition = 6,
    SetDeviceId = 7,
    ReadParameter = 17,
    WriteParameter = 18,
    FaultReport = 21,
    SaveParameters = 22,
    SetBaudrate = 23,
    ActiveReport = 24,
    SetProtocol = 25,
}

impl CommunicationType {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::GetDeviceId),
            1 => Some(Self::OperationControl),
            2 => Some(Self::OperationStatus),
            3 => Some(Self::Enable),
            4 => Some(Self::Disable),
            6 => Some(Self::SetZeroPosition),
            7 => Some(Self::SetDeviceId),
            17 => Some(Self::ReadParameter),
            18 => Some(Self::WriteParameter),
            21 => Some(Self::FaultReport),
            22 => Some(Self::SaveParameters),
            23 => Some(Self::SetBaudrate),
            24 => Some(Self::ActiveReport),
            25 => Some(Self::SetProtocol),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterDataType {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    F32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParameterType {
    pub id: u16,
    pub data_type: ParameterDataType,
    pub name: &'static str,
}

pub mod parameter {
    use super::{ParameterDataType, ParameterType};

    pub const MECHANICAL_OFFSET: ParameterType = ParameterType {
        id: 0x2005,
        data_type: ParameterDataType::F32,
        name: "mechOffset",
    };
    pub const MEASURED_POSITION: ParameterType = ParameterType {
        id: 0x3016,
        data_type: ParameterDataType::F32,
        name: "mechPos",
    };
    pub const MEASURED_VELOCITY: ParameterType = ParameterType {
        id: 0x3017,
        data_type: ParameterDataType::F32,
        name: "mechVel",
    };
    pub const MEASURED_TORQUE: ParameterType = ParameterType {
        id: 0x302C,
        data_type: ParameterDataType::F32,
        name: "torque_fdb",
    };
    pub const MODE: ParameterType = ParameterType {
        id: 0x7005,
        data_type: ParameterDataType::I8,
        name: "run_mode",
    };
    pub const IQ_TARGET: ParameterType = ParameterType {
        id: 0x7006,
        data_type: ParameterDataType::F32,
        name: "iq_ref",
    };
    pub const VELOCITY_TARGET: ParameterType = ParameterType {
        id: 0x700A,
        data_type: ParameterDataType::F32,
        name: "spd_ref",
    };
    pub const TORQUE_LIMIT: ParameterType = ParameterType {
        id: 0x700B,
        data_type: ParameterDataType::F32,
        name: "limit_torque",
    };
    pub const CURRENT_KP: ParameterType = ParameterType {
        id: 0x7010,
        data_type: ParameterDataType::F32,
        name: "cur_kp",
    };
    pub const CURRENT_KI: ParameterType = ParameterType {
        id: 0x7011,
        data_type: ParameterDataType::F32,
        name: "cur_ki",
    };
    pub const CURRENT_FILTER_GAIN: ParameterType = ParameterType {
        id: 0x7014,
        data_type: ParameterDataType::F32,
        name: "cur_filter_gain",
    };
    pub const POSITION_TARGET: ParameterType = ParameterType {
        id: 0x7016,
        data_type: ParameterDataType::F32,
        name: "loc_ref",
    };
    pub const VELOCITY_LIMIT: ParameterType = ParameterType {
        id: 0x7017,
        data_type: ParameterDataType::F32,
        name: "limit_spd",
    };
    pub const CURRENT_LIMIT: ParameterType = ParameterType {
        id: 0x7018,
        data_type: ParameterDataType::F32,
        name: "limit_cur",
    };
    pub const MECHANICAL_POSITION: ParameterType = ParameterType {
        id: 0x7019,
        data_type: ParameterDataType::F32,
        name: "mechPos",
    };
    pub const IQ_FILTERED: ParameterType = ParameterType {
        id: 0x701A,
        data_type: ParameterDataType::F32,
        name: "iqf",
    };
    pub const MECHANICAL_VELOCITY: ParameterType = ParameterType {
        id: 0x701B,
        data_type: ParameterDataType::F32,
        name: "mechVel",
    };
    pub const VBUS: ParameterType = ParameterType {
        id: 0x701C,
        data_type: ParameterDataType::F32,
        name: "VBUS",
    };
    pub const POSITION_KP: ParameterType = ParameterType {
        id: 0x701E,
        data_type: ParameterDataType::F32,
        name: "loc_kp",
    };
    pub const VELOCITY_KP: ParameterType = ParameterType {
        id: 0x701F,
        data_type: ParameterDataType::F32,
        name: "spd_kp",
    };
    pub const VELOCITY_KI: ParameterType = ParameterType {
        id: 0x7020,
        data_type: ParameterDataType::F32,
        name: "spd_ki",
    };
    pub const VELOCITY_FILTER_GAIN: ParameterType = ParameterType {
        id: 0x7021,
        data_type: ParameterDataType::F32,
        name: "spd_filter_gain",
    };
    pub const VEL_ACCELERATION_TARGET: ParameterType = ParameterType {
        id: 0x7022,
        data_type: ParameterDataType::F32,
        name: "acc_rad",
    };
    pub const PP_VELOCITY_MAX: ParameterType = ParameterType {
        id: 0x7024,
        data_type: ParameterDataType::F32,
        name: "vel_max",
    };
    pub const PP_ACCELERATION_TARGET: ParameterType = ParameterType {
        id: 0x7025,
        data_type: ParameterDataType::F32,
        name: "acc_set",
    };
    pub const EPSCAN_TIME: ParameterType = ParameterType {
        id: 0x7026,
        data_type: ParameterDataType::U16,
        name: "EPScan_time",
    };
    pub const CAN_TIMEOUT: ParameterType = ParameterType {
        id: 0x7028,
        data_type: ParameterDataType::U32,
        name: "canTimeout",
    };
    pub const ZERO_STATE: ParameterType = ParameterType {
        id: 0x7029,
        data_type: ParameterDataType::U8,
        name: "zero_sta",
    };
}

#[derive(Clone, Copy, Debug)]
pub struct MitLimits {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub kp: f64,
    pub kd: f64,
}

#[derive(Clone, Debug)]
pub struct ParsedFrame {
    pub communication_type: CommunicationType,
    pub extra_data: u16,
    pub device_id: u8,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct StatusFrame {
    pub actuator_id: u8,
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub temperature: f64,
    pub faults: Vec<String>,
    pub is_fault: bool,
}

pub fn make_id(communication_type: CommunicationType, extra_data: u16, device_id: u8) -> u32 {
    ((communication_type as u32) << 24) | ((extra_data as u32) << 8) | u32::from(device_id)
}

pub fn parse_frame(frame: &CanFrame) -> Option<ParsedFrame> {
    if frame.is_standard() {
        return None;
    }

    let raw_id = frame.raw_id();
    let communication_type = CommunicationType::from_u8(((raw_id >> 24) & 0x1F) as u8)?;

    Some(ParsedFrame {
        communication_type,
        extra_data: ((raw_id >> 8) & 0xFFFF) as u16,
        device_id: (raw_id & 0xFF) as u8,
        data: frame.data().to_vec(),
    })
}

pub fn mit_limits(model: &str) -> Option<MitLimits> {
    match model {
        "rs-00" | "rs-01" | "rs-02" | "rs-03" | "rs-04" | "rs-05" | "rs-06" => Some(MitLimits {
            position: 4.0 * std::f64::consts::PI,
            velocity: match model {
                "rs-00" => 50.0,
                "rs-01" | "rs-02" => 44.0,
                "rs-03" => 50.0,
                "rs-04" => 15.0,
                "rs-05" => 33.0,
                "rs-06" => 20.0,
                _ => unreachable!(),
            },
            torque: match model {
                "rs-03" | "rs-06" => 60.0,
                "rs-04" => 120.0,
                _ => 17.0,
            },
            kp: match model {
                "rs-03" | "rs-04" | "rs-06" => 5000.0,
                _ => 500.0,
            },
            kd: match model {
                "rs-03" | "rs-04" | "rs-06" => 100.0,
                _ => 5.0,
            },
        }),
        _ => None,
    }
}

pub fn decode_parameter_value(parameter: ParameterType, payload: &[u8]) -> Result<ParameterValue> {
    if payload.len() != size_of::<u32>() {
        return Err(ActuatorError::Protocol(format!(
            "invalid Robstride payload length for {}: {}",
            parameter.name,
            payload.len()
        )));
    }

    let bytes: [u8; 4] = payload.try_into().expect("checked len");
    let value = match parameter.data_type {
        ParameterDataType::U8 => ParameterValue::Integer(i64::from(bytes[0])),
        ParameterDataType::I8 => ParameterValue::Integer(i64::from(i8::from_le_bytes([bytes[0]]))),
        ParameterDataType::U16 => ParameterValue::Integer(i64::from(u16::from_le_bytes(
            bytes[..2].try_into().unwrap(),
        ))),
        ParameterDataType::I16 => ParameterValue::Integer(i64::from(i16::from_le_bytes(
            bytes[..2].try_into().unwrap(),
        ))),
        ParameterDataType::U32 => ParameterValue::Integer(i64::from(u32::from_le_bytes(bytes))),
        ParameterDataType::I32 => ParameterValue::Integer(i64::from(i32::from_le_bytes(bytes))),
        ParameterDataType::F32 => ParameterValue::Float(f32::from_le_bytes(bytes)),
    };
    Ok(value)
}

pub fn encode_parameter_value(parameter: ParameterType, value: ParameterValue) -> Result<[u8; 4]> {
    let bytes = match (parameter.data_type, value) {
        (ParameterDataType::U8, ParameterValue::Integer(value)) => {
            [u8::try_from(value).map_err(|_| integer_range_error(parameter, value, "u8"))?, 0, 0, 0]
        }
        (ParameterDataType::I8, ParameterValue::Integer(value)) => {
            let signed =
                i8::try_from(value).map_err(|_| integer_range_error(parameter, value, "i8"))?;
            [signed as u8, 0, 0, 0]
        }
        (ParameterDataType::U16, ParameterValue::Integer(value)) => {
            let raw = u16::try_from(value)
                .map_err(|_| integer_range_error(parameter, value, "u16"))?
                .to_le_bytes();
            [raw[0], raw[1], 0, 0]
        }
        (ParameterDataType::I16, ParameterValue::Integer(value)) => {
            let raw = i16::try_from(value)
                .map_err(|_| integer_range_error(parameter, value, "i16"))?
                .to_le_bytes();
            [raw[0], raw[1], 0, 0]
        }
        (ParameterDataType::U32, ParameterValue::Integer(value)) => u32::try_from(value)
            .map_err(|_| integer_range_error(parameter, value, "u32"))?
            .to_le_bytes(),
        (ParameterDataType::I32, ParameterValue::Integer(value)) => i32::try_from(value)
            .map_err(|_| integer_range_error(parameter, value, "i32"))?
            .to_le_bytes(),
        (ParameterDataType::F32, ParameterValue::Float(value)) => value.to_le_bytes(),
        _ => {
            return Err(ActuatorError::Protocol(format!(
                "parameter {} received a value with the wrong type",
                parameter.name
            )));
        }
    };

    Ok(bytes)
}

fn integer_range_error(parameter: ParameterType, value: i64, target_type: &str) -> ActuatorError {
    ActuatorError::Protocol(format!(
        "parameter {} value {value} does not fit in {target_type}",
        parameter.name
    ))
}

pub fn decode_status(model: &str, parsed: &ParsedFrame) -> Result<StatusFrame> {
    let limits = mit_limits(model)
        .ok_or_else(|| ActuatorError::Protocol(format!("unsupported Robstride model {model:?}")))?;
    let actuator_id = (parsed.extra_data & 0xFF) as u8;

    let mut faults = Vec::new();
    if (parsed.extra_data >> 13) & 0x01 == 1 {
        faults.push("uncalibrated".to_string());
    }
    if (parsed.extra_data >> 12) & 0x01 == 1 {
        faults.push("stall overload".to_string());
    }
    if (parsed.extra_data >> 11) & 0x01 == 1 {
        faults.push("magnetic encoder fault".to_string());
    }
    if (parsed.extra_data >> 10) & 0x01 == 1 {
        faults.push("overtemperature".to_string());
    }
    if (parsed.extra_data >> 9) & 0x01 == 1 {
        faults.push("gate driver fault".to_string());
    }
    if (parsed.extra_data >> 8) & 0x01 == 1 {
        faults.push("undervoltage".to_string());
    }

    if parsed.communication_type == CommunicationType::FaultReport {
        if parsed.data.len() != 8 {
            return Err(ActuatorError::Protocol(format!(
                "invalid Robstride fault payload length: {}",
                parsed.data.len()
            )));
        }

        let fault_value = u32::from_le_bytes(parsed.data[..4].try_into().unwrap());
        let warning_value = u32::from_le_bytes(parsed.data[4..8].try_into().unwrap());

        if fault_value & 0x01 != 0 {
            faults.push("overtemperature".to_string());
        }
        if (fault_value >> 1) & 0x01 != 0 {
            faults.push("gate driver fault".to_string());
        }
        if (fault_value >> 2) & 0x01 != 0 {
            faults.push("undervoltage".to_string());
        }
        if (fault_value >> 3) & 0x01 != 0 {
            faults.push("overvoltage".to_string());
        }
        if (fault_value >> 7) & 0x01 != 0 {
            faults.push("uncalibrated".to_string());
        }
        if (fault_value >> 14) & 0x01 != 0 {
            faults.push("stall overload".to_string());
        }
        if warning_value & 0x01 != 0 {
            faults.push("overtemperature warning".to_string());
        }

        return Ok(StatusFrame {
            actuator_id,
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            temperature: 0.0,
            faults,
            is_fault: true,
        });
    }

    if parsed.data.len() != 8 {
        return Err(ActuatorError::Protocol(format!(
            "invalid Robstride status payload length: {}",
            parsed.data.len()
        )));
    }

    let position_u16 = u16::from_be_bytes(parsed.data[0..2].try_into().unwrap());
    let velocity_u16 = u16::from_be_bytes(parsed.data[2..4].try_into().unwrap());
    let torque_u16 = u16::from_be_bytes(parsed.data[4..6].try_into().unwrap());
    let temperature_u16 = u16::from_be_bytes(parsed.data[6..8].try_into().unwrap());

    let position = (f64::from(position_u16) / 32_767.0 - 1.0) * limits.position;
    let velocity = (f64::from(velocity_u16) / 32_767.0 - 1.0) * limits.velocity;
    let torque = (f64::from(torque_u16) / 32_767.0 - 1.0) * limits.torque;
    let temperature = f64::from(temperature_u16) * 0.1;

    Ok(StatusFrame {
        actuator_id,
        position,
        velocity,
        torque,
        temperature,
        faults,
        is_fault: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_parameter_value_rejects_out_of_range_integer_values() {
        let error = encode_parameter_value(
            ParameterType {
                id: 0x0001,
                data_type: ParameterDataType::I16,
                name: "test_i16",
            },
            ParameterValue::Integer(i64::from(i16::MAX) + 1),
        )
        .unwrap_err();
        assert!(matches!(error, ActuatorError::Protocol(_)));
    }

    #[test]
    fn encode_parameter_value_preserves_valid_integer_values() {
        let bytes = encode_parameter_value(
            ParameterType {
                id: 0x0001,
                data_type: ParameterDataType::I32,
                name: "test_i32",
            },
            ParameterValue::Integer(-42),
        )
        .unwrap();
        assert_eq!(bytes, (-42i32).to_le_bytes());
    }
}
