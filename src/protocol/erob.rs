use std::f64::consts::PI;

use socketcan::{CanFrame, EmbeddedFrame, Frame};

pub const CLIENT_ID_BASE: u32 = 0x640;
pub const SERVER_ID_BASE: u32 = 0x5C0;
pub const STATUS_SUCCESS: u8 = 0x3E;
pub const COUNTS_PER_RAD: f64 = 524_287.0 / (2.0 * PI);

pub mod command {
    pub const START_MOTION: u8 = 0x83;
    pub const STOP_MOTION: u8 = 0x84;
    pub const SAVE_PARAMS: u8 = 0xE8;
}

pub mod parameter {
    pub const ACTUAL_POSITION: u16 = 0x0002;
    pub const ACTUAL_SPEED: u16 = 0x0005;
    pub const MOTOR_CURRENT: u16 = 0x0008;
    pub const ERROR_CODE: u16 = 0x001F;
    pub const RUN_STATUS: u16 = 0x0020;
    pub const POWER_TEMP: u16 = 0x0026;
    pub const CONTROL_MODE: u16 = 0x004E;
    pub const POSITION_LOOP_GAIN: u16 = 0x0064;
    pub const SPEED_LOOP_GAIN: u16 = 0x0066;
    pub const SPEED_LOOP_INTEGRAL: u16 = 0x0067;
    pub const MAX_POSITION_ERROR: u16 = 0x0054;
    pub const MAX_SPEED: u16 = 0x0055;
    pub const TARGET_POSITION: u16 = 0x0086;
    pub const RELATIVE_POSITION: u16 = 0x0087;
    pub const ACCELERATION: u16 = 0x0088;
    pub const DECELERATION: u16 = 0x0089;
    pub const TARGET_SPEED: u16 = 0x008A;
    pub const MOTION_MODE: u16 = 0x008D;
    pub const PID_ADJUSTMENT: u16 = 0x0124;
}

pub mod mode {
    pub const TORQUE: u8 = 1;
    pub const SPEED: u8 = 2;
    pub const POSITION: u8 = 3;
}

#[derive(Debug, Clone)]
pub struct Response {
    pub actuator_id: u8,
    pub payload: Vec<u8>,
}

pub fn client_id(device_id: u8) -> u32 {
    CLIENT_ID_BASE | u32::from(device_id)
}

pub fn server_id(device_id: u8) -> u32 {
    SERVER_ID_BASE | u32::from(device_id)
}

pub fn parse_response(frame: &CanFrame) -> Option<Response> {
    if !frame.is_standard() {
        return None;
    }

    let raw_id = frame.raw_id();
    if !(SERVER_ID_BASE..=(SERVER_ID_BASE | 0xFF)).contains(&raw_id) {
        return None;
    }

    let data = frame.data();
    if data.is_empty() {
        return None;
    }

    if data[data.len() - 1] != STATUS_SUCCESS {
        return None;
    }

    let payload = if data.len() == 1 {
        Vec::new()
    } else {
        data[..data.len() - 1].to_vec()
    };

    Some(Response {
        actuator_id: (raw_id & 0xFF) as u8,
        payload,
    })
}
