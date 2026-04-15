use std::f64::consts::PI;

use socketcan::{CanFrame, EmbeddedFrame, Frame};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SitoModelConfig {
    pub counts_per_rad: f64,
    pub gear_ratio: f64,
    pub torque_constant: f64,
}

pub const CAN_ID_BASE: u32 = 0x0506_0000;

pub mod communication {
    pub const RESET: u8 = 0x00;
    pub const SELECT_MODE: u8 = 0x01;
    pub const SET_MIT_CURRENT_VELOCITY_POSITION: u8 = 0x09;
    pub const SET_MIT_KP_KD: u8 = 0x45;
    pub const FEEDBACK_0: u8 = 0xB0;
    pub const FEEDBACK_1: u8 = 0xB1;
    pub const FEEDBACK_2: u8 = 0xB2;
    pub const FEEDBACK_3: u8 = 0xB3;
}

pub mod mode {
    pub const POSITION: u8 = 0x08;
    pub const MIT: u8 = 0x09;
}

#[derive(Clone, Debug)]
pub struct ParsedFeedback {
    pub actuator_id: u8,
    pub message_type: u8,
    pub data: Vec<u8>,
}

pub fn model_config(model: &str) -> Option<SitoModelConfig> {
    let counts_per_rad = 65_536.0 / (2.0 * PI);
    match model {
        "TA40-50" => Some(SitoModelConfig {
            counts_per_rad,
            gear_ratio: 51.0,
            torque_constant: 0.00009,
        }),
        "TA40-100" => Some(SitoModelConfig {
            counts_per_rad,
            gear_ratio: 101.0,
            torque_constant: 0.00009,
        }),
        _ => None,
    }
}

pub fn make_id(device_id: u8, message_type: u8) -> u32 {
    CAN_ID_BASE | (u32::from(device_id) << 8) | u32::from(message_type)
}

pub fn parse_feedback(frame: &CanFrame) -> Option<ParsedFeedback> {
    if frame.is_standard() {
        return None;
    }

    let raw_id = frame.raw_id();
    if raw_id & 0xFFFF_0000 != CAN_ID_BASE {
        return None;
    }

    Some(ParsedFeedback {
        actuator_id: ((raw_id >> 8) & 0xFF) as u8,
        message_type: (raw_id & 0xFF) as u8,
        data: frame.data().to_vec(),
    })
}
