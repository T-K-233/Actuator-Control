//! Rust CAN actuator control for eRob, Robstride, and Sito actuators.
//!
//! The crate follows the same broad shape as `rustypot`: explicit low-level
//! protocol modules plus high-level controller types that own the bus-facing
//! runtime.

pub mod core;
pub mod protocol;

mod erob;
mod python;
mod robstride;
mod sito;

use pyo3::prelude::*;

pub use core::{
    Actuator, ActuatorError, ActuatorState, CachedActuatorState, Calibration, ParameterValue,
    Result,
};
pub use erob::ErobBus;
pub use protocol::robstride::{
    CommunicationType as RobstrideCommunicationType, ParameterType as RobstrideParameterType,
};
pub use robstride::RobstrideBus;
pub use sito::SitoBus;

#[pymodule]
fn _rust(_py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    python::register_python_module(module)
}
