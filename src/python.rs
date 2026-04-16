use std::collections::HashMap;
use std::time::Duration;

use pyo3::exceptions::{PyKeyError, PyOSError, PyRuntimeError, PyTimeoutError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyModule, PyType};

use crate::protocol::robstride;
use crate::{
    Actuator, ActuatorError, ActuatorState, Calibration, ErobBus, ParameterValue, Result,
    RobstrideBus, SitoBus,
};

#[derive(FromPyObject)]
struct PyActuatorInput {
    #[pyo3(item)]
    id: u8,
    #[pyo3(item)]
    model: String,
}

#[derive(FromPyObject)]
struct PyCalibrationInput {
    #[pyo3(item)]
    direction: Option<f64>,
    #[pyo3(item)]
    homing_offset: Option<f64>,
}

/// Python binding for the eRob CAN backend.
#[pyclass(unsendable, module = "actuator_control._rust", name = "_ErobBus")]
pub struct PyErobBus {
    inner: ErobBus,
}

/// Python binding for the Robstride CAN backend.
#[pyclass(unsendable, module = "actuator_control._rust", name = "_RobstrideBus")]
pub struct PyRobstrideBus {
    inner: RobstrideBus,
}

/// Python binding for the Sito CAN backend.
#[pyclass(unsendable, module = "actuator_control._rust", name = "_SitoBus")]
pub struct PySitoBus {
    inner: SitoBus,
}

#[pymethods]
impl PyErobBus {
    #[new]
    #[pyo3(signature = (channel, actuators, calibration=None, bitrate=1_000_000))]
    /// Create an eRob backend instance.
    ///
    /// Args:
    ///     channel: CAN interface name.
    ///     actuators: Actuator metadata keyed by logical name.
    ///     calibration: Optional per-actuator calibration mapping.
    ///     bitrate: CAN bitrate in bits per second.
    fn new(
        channel: String,
        actuators: HashMap<String, PyActuatorInput>,
        calibration: Option<HashMap<String, PyCalibrationInput>>,
        bitrate: u32,
    ) -> PyResult<Self> {
        let mut inner = ErobBus::new(channel, convert_actuators(actuators)).map_err(map_error)?;
        inner = inner.with_bitrate(bitrate);
        inner = inner.with_calibrations(convert_calibrations(calibration)?);
        Ok(Self { inner })
    }

    /// Open the CAN backend and start receiving frames.
    fn connect(&mut self) -> PyResult<()> {
        self.inner.connect().map_err(map_error)
    }

    #[pyo3(signature = (disable_torque=true))]
    /// Disconnect the backend.
    ///
    /// Args:
    ///     disable_torque: Whether to disable actuators before closing.
    fn disconnect(&mut self, disable_torque: bool) -> PyResult<()> {
        self.inner.disconnect(disable_torque).map_err(map_error)
    }

    /// Enable one actuator by logical name.
    fn enable(&self, actuator: &str) -> PyResult<()> {
        self.inner.enable(actuator).map_err(map_error)
    }

    /// Disable one actuator by logical name.
    fn disable(&self, actuator: &str) -> PyResult<()> {
        self.inner.disable(actuator).map_err(map_error)
    }

    /// Read one eRob parameter as an integer.
    fn read(&self, actuator: &str, parameter: u16) -> PyResult<i64> {
        let value = self.inner.read(actuator, parameter).map_err(map_error)?;
        match value {
            ParameterValue::Integer(value) => Ok(value),
            ParameterValue::Float(_) => Err(PyRuntimeError::new_err(
                "eRob read returned a float unexpectedly",
            )),
        }
    }

    /// Write one integer eRob parameter.
    fn write(&self, actuator: &str, parameter: u16, value: i32) -> PyResult<()> {
        self.inner
            .write(actuator, parameter, value)
            .map_err(map_error)
    }

    /// Update MIT-mode proportional and derivative gains.
    fn write_mit_kp_kd(&self, actuator: &str, kp: f64, kd: f64) -> PyResult<()> {
        self.inner
            .write_mit_kp_kd(actuator, kp, kd)
            .map_err(map_error)
    }

    #[pyo3(signature = (actuator, position, velocity=0.0, torque=0.0))]
    /// Send one MIT control command.
    ///
    /// Args:
    ///     actuator: Logical actuator name.
    ///     position: Target output position in radians.
    ///     velocity: Target output velocity in radians per second.
    ///     torque: Feedforward output torque in newton-meters.
    fn write_mit_control(
        &self,
        actuator: &str,
        position: f64,
        velocity: f64,
        torque: f64,
    ) -> PyResult<()> {
        self.inner
            .write_mit_control(actuator, position, velocity, torque)
            .map_err(map_error)
    }

    /// Trigger an asynchronous state refresh.
    fn request_state(&self, actuator: &str) -> PyResult<()> {
        self.inner.request_state(actuator).map_err(map_error)
    }

    /// Return the last cached actuator state tuple.
    fn read_state(&self, actuator: &str) -> PyResult<Option<(f64, f64, f64, f64, Vec<String>)>> {
        state_to_tuple(self.inner.read_state(actuator))
    }

    /// Return the number of transmitted CAN frames.
    fn read_tx_counter(&self) -> u64 {
        self.inner.tx_counter()
    }

    /// Return the number of received CAN frames.
    fn read_rx_counter(&self) -> u64 {
        self.inner.rx_counter()
    }

    /// Return the configured CAN bitrate.
    fn bitrate(&self) -> u32 {
        self.inner.bitrate()
    }
}

#[pymethods]
impl PyRobstrideBus {
    #[new]
    #[pyo3(signature = (channel, actuators, calibration=None, bitrate=1_000_000))]
    /// Create a Robstride backend instance.
    ///
    /// Args:
    ///     channel: CAN interface name.
    ///     actuators: Actuator metadata keyed by actuator name.
    ///     calibration: Optional per-actuator calibration mapping.
    ///     bitrate: CAN bitrate in bits per second.
    fn new(
        channel: String,
        actuators: HashMap<String, PyActuatorInput>,
        calibration: Option<HashMap<String, PyCalibrationInput>>,
        bitrate: u32,
    ) -> PyResult<Self> {
        let mut inner =
            RobstrideBus::new(channel, convert_actuators(actuators)).map_err(map_error)?;
        inner = inner.with_bitrate(bitrate);
        inner = inner.with_calibrations(convert_calibrations(calibration)?);
        Ok(Self { inner })
    }

    /// Open the CAN backend and start receiving frames.
    fn connect(&mut self) -> PyResult<()> {
        self.inner.connect().map_err(map_error)
    }

    #[pyo3(signature = (disable_torque=true))]
    /// Disconnect the backend.
    ///
    /// Args:
    ///     disable_torque: Whether to disable actuators before closing.
    fn disconnect(&mut self, disable_torque: bool) -> PyResult<()> {
        self.inner.disconnect(disable_torque).map_err(map_error)
    }

    /// Enable one actuator by logical name.
    fn enable(&self, actuator: &str) -> PyResult<()> {
        self.inner.enable(actuator).map_err(map_error)
    }

    /// Disable one actuator by logical name.
    fn disable(&self, actuator: &str) -> PyResult<()> {
        self.inner.disable(actuator).map_err(map_error)
    }

    /// Read one Robstride parameter.
    fn read(&self, py: Python<'_>, actuator: &str, parameter: u16) -> PyResult<PyObject> {
        let parameter = robstride_parameter(parameter)?;
        let value = self.inner.read(actuator, parameter).map_err(map_error)?;
        match value {
            ParameterValue::Integer(value) => Ok(value.into_py(py)),
            ParameterValue::Float(value) => Ok(value.into_py(py)),
        }
    }

    /// Write one integer Robstride parameter.
    fn write_integer(&self, actuator: &str, parameter: u16, value: i64) -> PyResult<()> {
        let parameter = robstride_parameter(parameter)?;
        self.inner
            .write(actuator, parameter, ParameterValue::Integer(value))
            .map_err(map_error)
    }

    /// Write one floating-point Robstride parameter.
    fn write_float(&self, actuator: &str, parameter: u16, value: f32) -> PyResult<()> {
        let parameter = robstride_parameter(parameter)?;
        self.inner
            .write(actuator, parameter, ParameterValue::Float(value))
            .map_err(map_error)
    }

    /// Update MIT-mode proportional and derivative gains.
    fn write_mit_kp_kd(&self, actuator: &str, kp: f64, kd: f64) -> PyResult<()> {
        self.inner
            .write_mit_kp_kd(actuator, kp, kd)
            .map_err(map_error)
    }

    #[pyo3(signature = (actuator, position, velocity=0.0, torque=0.0))]
    /// Send one MIT control command.
    ///
    /// Args:
    ///     actuator: Logical actuator name.
    ///     position: Target output position in radians.
    ///     velocity: Target output velocity in radians per second.
    ///     torque: Feedforward output torque in newton-meters.
    fn write_mit_control(
        &self,
        actuator: &str,
        position: f64,
        velocity: f64,
        torque: f64,
    ) -> PyResult<()> {
        self.inner
            .write_mit_control(actuator, position, velocity, torque)
            .map_err(map_error)
    }

    /// Trigger an asynchronous state refresh.
    fn request_state(&self, actuator: &str) -> PyResult<()> {
        self.inner.request_state(actuator).map_err(map_error)
    }

    /// Return the last cached actuator state tuple.
    fn read_state(&self, actuator: &str) -> PyResult<Option<(f64, f64, f64, f64, Vec<String>)>> {
        state_to_tuple(self.inner.read_state(actuator))
    }

    /// Return the number of transmitted CAN frames.
    fn read_tx_counter(&self) -> u64 {
        self.inner.tx_counter()
    }

    /// Return the number of received CAN frames.
    fn read_rx_counter(&self) -> u64 {
        self.inner.rx_counter()
    }

    #[pyo3(signature = (actuator=None))]
    /// Read cached fault status.
    ///
    /// Args:
    ///     actuator: Optional logical actuator name. If omitted, returns all cached faults.
    fn read_fault_status(&self, py: Python<'_>, actuator: Option<&str>) -> PyResult<PyObject> {
        let mut faults = self.inner.read_fault_status(actuator).map_err(map_error)?;
        match actuator {
            Some(actuator) => Ok(faults.remove(actuator).unwrap_or_default().into_py(py)),
            None => Ok(faults.into_py(py)),
        }
    }

    /// Clear cached and device-side fault state for one actuator.
    fn clear_fault(&self, actuator: &str) -> PyResult<()> {
        self.inner.clear_fault(actuator).map_err(map_error)
    }

    #[classmethod]
    #[pyo3(signature = (_channel, _device_id, _timeout=0.1))]
    /// Probe one Robstride device ID.
    ///
    /// Args:
    ///     _channel: CAN interface name.
    ///     _device_id: Device ID to probe.
    ///     _timeout: Receive timeout in seconds.
    fn ping_by_id(
        _cls: &Bound<'_, PyType>,
        _channel: &str,
        _device_id: u8,
        _timeout: f64,
    ) -> PyResult<Option<(u16, Vec<u8>)>> {
        let timeout = duration_from_seconds(_timeout)?;
        RobstrideBus::ping_by_id(_channel, _device_id, timeout).map_err(map_error)
    }

    /// Return the configured CAN bitrate.
    fn bitrate(&self) -> u32 {
        self.inner.bitrate()
    }
}

#[pymethods]
impl PySitoBus {
    #[new]
    #[pyo3(signature = (channel, actuators, calibration=None, bitrate=1_000_000, control_frequency=50.0))]
    /// Create a Sito backend instance.
    ///
    /// Args:
    ///     channel: CAN interface name.
    ///     actuators: Actuator metadata keyed by actuator name.
    ///     calibration: Optional per-actuator calibration mapping.
    ///     bitrate: CAN bitrate in bits per second.
    ///     control_frequency: Requested control loop frequency in hertz.
    fn new(
        channel: String,
        actuators: HashMap<String, PyActuatorInput>,
        calibration: Option<HashMap<String, PyCalibrationInput>>,
        bitrate: u32,
        control_frequency: f64,
    ) -> PyResult<Self> {
        let mut inner = SitoBus::new(channel, convert_actuators(actuators)).map_err(map_error)?;
        inner = inner.with_bitrate(bitrate);
        inner = inner
            .with_control_frequency(control_frequency)
            .map_err(map_error)?;
        inner = inner.with_calibrations(convert_calibrations(calibration)?);
        Ok(Self { inner })
    }

    /// Open the CAN backend and start receiving frames.
    fn connect(&mut self) -> PyResult<()> {
        self.inner.connect().map_err(map_error)
    }

    #[pyo3(signature = (disable_torque=true))]
    /// Disconnect the backend.
    ///
    /// Args:
    ///     disable_torque: Whether to disable actuators before closing.
    fn disconnect(&mut self, disable_torque: bool) -> PyResult<()> {
        self.inner.disconnect(disable_torque).map_err(map_error)
    }

    /// Enable one actuator by logical name.
    fn enable(&self, actuator: &str) -> PyResult<()> {
        self.inner.enable(actuator).map_err(map_error)
    }

    /// Disable one actuator by logical name.
    fn disable(&self, actuator: &str) -> PyResult<()> {
        self.inner.disable(actuator).map_err(map_error)
    }

    /// Update MIT-mode proportional and derivative gains.
    fn write_mit_kp_kd(&self, actuator: &str, kp: f64, kd: f64) -> PyResult<()> {
        self.inner
            .write_mit_kp_kd(actuator, kp, kd)
            .map_err(map_error)
    }

    #[pyo3(signature = (actuator, position, velocity=0.0, torque=0.0))]
    /// Send one MIT control command.
    ///
    /// Args:
    ///     actuator: Logical actuator name.
    ///     position: Target output position in radians.
    ///     velocity: Target output velocity in radians per second.
    ///     torque: Feedforward output torque in newton-meters.
    fn write_mit_control(
        &self,
        actuator: &str,
        position: f64,
        velocity: f64,
        torque: f64,
    ) -> PyResult<()> {
        self.inner
            .write_mit_control(actuator, position, velocity, torque)
            .map_err(map_error)
    }

    /// Trigger an asynchronous state refresh.
    fn request_state(&self, actuator: &str) -> PyResult<()> {
        self.inner.request_state(actuator).map_err(map_error)
    }

    /// Return the last cached actuator state tuple.
    fn read_state(&self, actuator: &str) -> PyResult<Option<(f64, f64, f64, f64, Vec<String>)>> {
        state_to_tuple(self.inner.read_state(actuator))
    }

    /// Return the number of transmitted CAN frames.
    fn read_tx_counter(&self) -> u64 {
        self.inner.tx_counter()
    }

    /// Return the number of received CAN frames.
    fn read_rx_counter(&self) -> u64 {
        self.inner.rx_counter()
    }

    /// Return the configured CAN bitrate.
    fn bitrate(&self) -> u32 {
        self.inner.bitrate()
    }
}

pub fn register_python_module(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyErobBus>()?;
    module.add_class::<PyRobstrideBus>()?;
    module.add_class::<PySitoBus>()?;
    Ok(())
}

fn convert_actuators(inputs: HashMap<String, PyActuatorInput>) -> HashMap<String, Actuator> {
    inputs
        .into_iter()
        .map(|(name, actuator)| (name, Actuator::new(actuator.id, actuator.model)))
        .collect()
}

fn convert_calibrations(
    inputs: Option<HashMap<String, PyCalibrationInput>>,
) -> PyResult<HashMap<String, Calibration>> {
    let Some(inputs) = inputs else {
        return Ok(HashMap::new());
    };

    inputs
        .into_iter()
        .map(|(name, calibration)| {
            let calibration = Calibration::new(
                calibration.direction.unwrap_or(1.0),
                calibration.homing_offset.unwrap_or(0.0),
            )
            .map_err(map_error)?;
            Ok((name, calibration))
        })
        .collect()
}

fn state_to_tuple(
    state: Result<Option<ActuatorState>>,
) -> PyResult<Option<(f64, f64, f64, f64, Vec<String>)>> {
    let state = state.map_err(map_error)?;
    Ok(state.map(|state| {
        (
            state.position,
            state.velocity,
            state.torque,
            state.temperature,
            state.faults,
        )
    }))
}

fn robstride_parameter(parameter_id: u16) -> PyResult<robstride::ParameterType> {
    let parameter = match parameter_id {
        0x2005 => robstride::parameter::MECHANICAL_OFFSET,
        0x3016 => robstride::parameter::MEASURED_POSITION,
        0x3017 => robstride::parameter::MEASURED_VELOCITY,
        0x302C => robstride::parameter::MEASURED_TORQUE,
        0x7005 => robstride::parameter::MODE,
        0x7006 => robstride::parameter::IQ_TARGET,
        0x700A => robstride::parameter::VELOCITY_TARGET,
        0x700B => robstride::parameter::TORQUE_LIMIT,
        0x7010 => robstride::parameter::CURRENT_KP,
        0x7011 => robstride::parameter::CURRENT_KI,
        0x7014 => robstride::parameter::CURRENT_FILTER_GAIN,
        0x7016 => robstride::parameter::POSITION_TARGET,
        0x7017 => robstride::parameter::VELOCITY_LIMIT,
        0x7018 => robstride::parameter::CURRENT_LIMIT,
        0x7019 => robstride::parameter::MECHANICAL_POSITION,
        0x701A => robstride::parameter::IQ_FILTERED,
        0x701B => robstride::parameter::MECHANICAL_VELOCITY,
        0x701C => robstride::parameter::VBUS,
        0x701E => robstride::parameter::POSITION_KP,
        0x701F => robstride::parameter::VELOCITY_KP,
        0x7020 => robstride::parameter::VELOCITY_KI,
        0x7021 => robstride::parameter::VELOCITY_FILTER_GAIN,
        0x7022 => robstride::parameter::VEL_ACCELERATION_TARGET,
        0x7024 => robstride::parameter::PP_VELOCITY_MAX,
        0x7025 => robstride::parameter::PP_ACCELERATION_TARGET,
        0x7026 => robstride::parameter::EPSCAN_TIME,
        0x7028 => robstride::parameter::CAN_TIMEOUT,
        0x7029 => robstride::parameter::ZERO_STATE,
        _ => {
            return Err(PyValueError::new_err(format!(
                "unknown Robstride parameter id 0x{parameter_id:04X}"
            )));
        }
    };

    Ok(parameter)
}

fn duration_from_seconds(seconds: f64) -> PyResult<Duration> {
    if seconds < 0.0 {
        return Err(PyValueError::new_err(
            "timeout must be greater than or equal to zero",
        ));
    }

    Ok(Duration::from_secs_f64(seconds))
}

fn map_error(error: ActuatorError) -> PyErr {
    match error {
        ActuatorError::UnknownActuator(message) => PyKeyError::new_err(message),
        error @ ActuatorError::DuplicateActuatorId(_)
        | error @ ActuatorError::InvalidCalibration(_)
        | error @ ActuatorError::UnsupportedModel { .. }
        | error @ ActuatorError::Protocol(_) => {
            PyValueError::new_err(error.to_string())
        }
        ActuatorError::Timeout { .. } => PyTimeoutError::new_err(error.to_string()),
        ActuatorError::AlreadyConnected(_)
        | ActuatorError::NotConnected(_)
        | ActuatorError::Busy(_) => PyRuntimeError::new_err(error.to_string()),
        ActuatorError::Io(_) => PyOSError::new_err(error.to_string()),
    }
}
