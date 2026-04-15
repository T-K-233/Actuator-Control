use std::io;
use std::time::Duration;

use socketcan::{CanFrame, CanSocket, Frame, Socket};

use crate::core::{ActuatorError, Result};

pub fn open_socket_pair(channel: &str, read_timeout: Duration) -> Result<(CanSocket, CanSocket)> {
    let tx = CanSocket::open(channel)?;
    let rx = CanSocket::open(channel)?;
    rx.set_read_timeout(read_timeout)?;
    Ok((tx, rx))
}

pub fn build_frame(raw_id: u32, data: &[u8]) -> Result<CanFrame> {
    CanFrame::from_raw_id(raw_id, data).ok_or_else(|| {
        ActuatorError::Protocol(format!(
            "failed to construct CAN frame id=0x{raw_id:08X} len={}",
            data.len()
        ))
    })
}

pub fn is_timeout_error(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
    )
}
