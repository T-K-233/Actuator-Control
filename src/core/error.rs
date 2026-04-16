use std::fmt;
use std::time::Duration;

pub type Result<T> = std::result::Result<T, ActuatorError>;

#[derive(Debug)]
pub enum ActuatorError {
    AlreadyConnected(String),
    NotConnected(String),
    UnknownActuator(String),
    DuplicateActuatorId(u8),
    InvalidCalibration(f64),
    Busy(String),
    UnsupportedModel {
        backend: &'static str,
        actuator: String,
        model: String,
    },
    Timeout {
        description: String,
        timeout: Duration,
    },
    Protocol(String),
    Io(std::io::Error),
}

impl fmt::Display for ActuatorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlreadyConnected(channel) => {
                write!(f, "bus on channel {channel:?} is already connected")
            }
            Self::NotConnected(channel) => write!(f, "bus on channel {channel:?} is not connected"),
            Self::UnknownActuator(actuator) => write!(f, "unknown actuator {actuator:?}"),
            Self::DuplicateActuatorId(id) => {
                write!(f, "actuator id {id} is configured more than once")
            }
            Self::InvalidCalibration(direction) => {
                write!(f, "calibration direction must be +/-1.0, got {direction}")
            }
            Self::Busy(message) => write!(f, "{message}"),
            Self::UnsupportedModel {
                backend,
                actuator,
                model,
            } => write!(
                f,
                "unsupported {backend} model {model:?} for actuator {actuator:?}"
            ),
            Self::Timeout {
                description,
                timeout,
            } => write!(f, "timed out waiting for {description} after {timeout:?}"),
            Self::Protocol(message) => write!(f, "{message}"),
            Self::Io(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for ActuatorError {}

impl From<std::io::Error> for ActuatorError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}
