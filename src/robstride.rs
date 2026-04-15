use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{SyncSender, sync_channel};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use socketcan::{CanSocket, Socket};

use crate::core::{
    Actuator, ActuatorError, ActuatorState, CachedActuatorState, Calibration, ParameterValue,
    Result, build_frame, is_timeout_error, open_socket_pair, validate_unique_ids,
};
use crate::protocol::robstride::{self, CommunicationType, ParameterType};

const RX_TIMEOUT: Duration = Duration::from_millis(20);
const COMMAND_TIMEOUT: Duration = Duration::from_millis(100);

struct PendingRead {
    parameter: ParameterType,
    actuator_name: String,
    tx: SyncSender<ParameterValue>,
}

struct SharedState {
    running: AtomicBool,
    frames_sent: AtomicU64,
    frames_received: AtomicU64,
    states: RwLock<HashMap<String, CachedActuatorState>>,
    pending_reads: Mutex<Option<PendingRead>>,
    pending_status: Mutex<HashMap<u8, SyncSender<robstride::StatusFrame>>>,
}

pub struct RobstrideBus {
    channel: String,
    bitrate: u32,
    host_id: u8,
    actuators: HashMap<String, Actuator>,
    calibrations: HashMap<String, Calibration>,
    shared: Arc<SharedState>,
    gains: Mutex<HashMap<String, (f64, f64)>>,
    tx_socket: Option<CanSocket>,
    rx_thread: Option<JoinHandle<()>>,
}

impl RobstrideBus {
    pub fn new(channel: impl Into<String>, actuators: HashMap<String, Actuator>) -> Result<Self> {
        validate_unique_ids(&actuators)?;
        let states = actuators
            .keys()
            .cloned()
            .map(|name| (name, CachedActuatorState::default()))
            .collect();

        Ok(Self {
            channel: channel.into(),
            bitrate: 1_000_000,
            host_id: 0xFF,
            actuators,
            calibrations: HashMap::new(),
            shared: Arc::new(SharedState {
                running: AtomicBool::new(false),
                frames_sent: AtomicU64::new(0),
                frames_received: AtomicU64::new(0),
                states: RwLock::new(states),
                pending_reads: Mutex::new(None),
                pending_status: Mutex::new(HashMap::new()),
            }),
            gains: Mutex::new(HashMap::new()),
            tx_socket: None,
            rx_thread: None,
        })
    }

    pub fn with_bitrate(mut self, bitrate: u32) -> Self {
        self.bitrate = bitrate;
        self
    }

    pub fn with_calibrations(mut self, calibrations: HashMap<String, Calibration>) -> Self {
        self.calibrations = calibrations;
        self
    }

    pub fn set_calibration(
        &mut self,
        actuator: impl Into<String>,
        calibration: Calibration,
    ) -> Result<()> {
        let actuator = actuator.into();
        if !self.actuators.contains_key(&actuator) {
            return Err(ActuatorError::UnknownActuator(actuator));
        }
        self.calibrations.insert(actuator, calibration);
        Ok(())
    }

    pub fn connect(&mut self) -> Result<()> {
        if self.tx_socket.is_some() {
            return Err(ActuatorError::AlreadyConnected(self.channel.clone()));
        }

        let (tx_socket, rx_socket) = open_socket_pair(&self.channel, RX_TIMEOUT)?;
        let shared = Arc::clone(&self.shared);
        let calibrations = self.calibrations.clone();
        let actuator_lookup = self
            .actuators
            .iter()
            .map(|(name, actuator)| (actuator.id, (name.clone(), actuator.model.clone())))
            .collect::<HashMap<_, _>>();

        self.shared.running.store(true, Ordering::Relaxed);
        self.rx_thread = Some(thread::spawn(move || {
            receiver_loop(rx_socket, shared, actuator_lookup, calibrations);
        }));
        self.tx_socket = Some(tx_socket);

        Ok(())
    }

    pub fn disconnect(&mut self, disable_torque: bool) -> Result<()> {
        if self.tx_socket.is_none() {
            return Err(ActuatorError::NotConnected(self.channel.clone()));
        }

        let mut first_error = None;
        if disable_torque {
            for name in self.actuators.keys().cloned().collect::<Vec<_>>() {
                if let Err(error) = self.disable(&name) {
                    if first_error.is_none() {
                        first_error = Some(error);
                    }
                }
            }
        }

        self.shared.running.store(false, Ordering::Relaxed);
        self.tx_socket.take();

        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }

        self.shared
            .pending_reads
            .lock()
            .expect("pending poisoned")
            .take();
        self.shared
            .pending_status
            .lock()
            .expect("pending poisoned")
            .clear();

        if let Some(error) = first_error {
            return Err(error);
        }

        Ok(())
    }

    pub fn enable(&self, actuator: &str) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let frame = build_frame(
            robstride::make_id(
                CommunicationType::Enable,
                u16::from(self.host_id),
                actuator_config.id,
            ),
            &[],
        )?;
        self.wait_for_status_after_send(actuator, actuator_config.id, frame)?;
        Ok(())
    }

    pub fn disable(&self, actuator: &str) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let frame = build_frame(
            robstride::make_id(
                CommunicationType::Disable,
                u16::from(self.host_id),
                actuator_config.id,
            ),
            &[0x00; 8],
        )?;
        self.wait_for_status_after_send(actuator, actuator_config.id, frame)?;
        Ok(())
    }

    pub fn read(&self, actuator: &str, parameter: ParameterType) -> Result<ParameterValue> {
        let actuator_config = self.require_actuator(actuator)?;
        let tx_socket = self.require_connected()?;
        let data = [
            (parameter.id & 0x00FF) as u8,
            (parameter.id >> 8) as u8,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ];
        let frame = build_frame(
            robstride::make_id(
                CommunicationType::ReadParameter,
                u16::from(self.host_id),
                actuator_config.id,
            ),
            &data,
        )?;
        let (reply_tx, reply_rx) = sync_channel(1);

        {
            let mut pending = self.shared.pending_reads.lock().expect("pending poisoned");
            if pending.is_some() {
                return Err(ActuatorError::Busy(
                    "another Robstride parameter read is already in flight".to_string(),
                ));
            }

            *pending = Some(PendingRead {
                parameter,
                actuator_name: actuator.to_string(),
                tx: reply_tx,
            });
        }

        if let Err(error) = tx_socket.write_frame(&frame) {
            self.shared
                .pending_reads
                .lock()
                .expect("pending poisoned")
                .take();
            return Err(error.into());
        }
        self.shared.frames_sent.fetch_add(1, Ordering::Relaxed);

        match reply_rx.recv_timeout(COMMAND_TIMEOUT) {
            Ok(value) => Ok(value),
            Err(_) => {
                self.shared
                    .pending_reads
                    .lock()
                    .expect("pending poisoned")
                    .take();
                Err(ActuatorError::Timeout {
                    description: format!("Robstride parameter read for actuator {actuator:?}"),
                    timeout: COMMAND_TIMEOUT,
                })
            }
        }
    }

    pub fn write(
        &self,
        actuator: &str,
        parameter: ParameterType,
        value: ParameterValue,
    ) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let value_bytes = robstride::encode_parameter_value(parameter, value)?;
        let mut data = Vec::with_capacity(8);
        data.extend_from_slice(&parameter.id.to_le_bytes());
        data.extend_from_slice(&0u16.to_le_bytes());
        data.extend_from_slice(&value_bytes);

        let frame = build_frame(
            robstride::make_id(
                CommunicationType::WriteParameter,
                u16::from(self.host_id),
                actuator_config.id,
            ),
            &data,
        )?;
        self.wait_for_status_after_send(actuator, actuator_config.id, frame)?;
        Ok(())
    }

    pub fn request_state(&self, actuator: &str) -> Result<()> {
        let _ = self.read(actuator, robstride::parameter::MEASURED_POSITION)?;
        let _ = self.read(actuator, robstride::parameter::MEASURED_VELOCITY)?;
        let _ = self.read(actuator, robstride::parameter::MEASURED_TORQUE)?;
        Ok(())
    }

    pub fn write_mit_kp_kd(&self, actuator: &str, kp: f64, kd: f64) -> Result<()> {
        self.require_actuator(actuator)?;
        self.gains
            .lock()
            .expect("gains poisoned")
            .insert(actuator.to_string(), (kp, kd));
        Ok(())
    }

    pub fn write_mit_control(
        &self,
        actuator: &str,
        position: f64,
        velocity: f64,
        torque: f64,
    ) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let limits = robstride::mit_limits(&actuator_config.model).ok_or_else(|| {
            ActuatorError::UnsupportedModel {
                backend: "Robstride",
                actuator: actuator.to_string(),
                model: actuator_config.model.clone(),
            }
        })?;
        let calibration = self.calibration_for(actuator)?;
        let (position, velocity, torque) = calibration.apply_command(position, velocity, torque);
        let (kp, kd) = self
            .gains
            .lock()
            .expect("gains poisoned")
            .get(actuator)
            .copied()
            .unwrap_or((0.0, 0.0));

        let position_u16 = scale_signed(position, limits.position);
        let velocity_u16 = scale_signed(velocity, limits.velocity);
        let torque_u16 = scale_signed(torque, limits.torque);
        let kp_u16 = scale_unsigned(kp, limits.kp);
        let kd_u16 = scale_unsigned(kd, limits.kd);

        let mut data = Vec::with_capacity(8);
        data.extend_from_slice(&position_u16.to_be_bytes());
        data.extend_from_slice(&velocity_u16.to_be_bytes());
        data.extend_from_slice(&kp_u16.to_be_bytes());
        data.extend_from_slice(&kd_u16.to_be_bytes());

        let frame = build_frame(
            robstride::make_id(
                CommunicationType::OperationControl,
                torque_u16,
                actuator_config.id,
            ),
            &data,
        )?;
        self.require_connected()?.write_frame(&frame)?;
        self.shared.frames_sent.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    pub fn read_state(&self, actuator: &str) -> Result<Option<ActuatorState>> {
        self.require_actuator(actuator)?;
        Ok(self
            .shared
            .states
            .read()
            .expect("states poisoned")
            .get(actuator)
            .and_then(CachedActuatorState::state))
    }

    pub fn read_fault_status(
        &self,
        actuator: Option<&str>,
    ) -> Result<HashMap<String, Vec<String>>> {
        let states = self.shared.states.read().expect("states poisoned");

        if let Some(actuator) = actuator {
            self.require_actuator(actuator)?;
            return Ok(HashMap::from([(
                actuator.to_string(),
                states
                    .get(actuator)
                    .map(|state| state.faults.clone())
                    .unwrap_or_default(),
            )]));
        }

        Ok(states
            .iter()
            .filter_map(|(name, state)| {
                (!state.faults.is_empty()).then(|| (name.clone(), state.faults.clone()))
            })
            .collect())
    }

    pub fn clear_fault(&self, actuator: &str) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let frame = build_frame(
            robstride::make_id(
                CommunicationType::Disable,
                u16::from(self.host_id),
                actuator_config.id,
            ),
            &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        )?;
        self.wait_for_status_after_send(actuator, actuator_config.id, frame)?;
        self.shared
            .states
            .write()
            .expect("states poisoned")
            .get_mut(actuator)
            .expect("actuator state initialized")
            .clear_faults();
        Ok(())
    }

    pub fn bitrate(&self) -> u32 {
        self.bitrate
    }

    pub fn tx_counter(&self) -> u64 {
        self.shared.frames_sent.load(Ordering::Relaxed)
    }

    pub fn rx_counter(&self) -> u64 {
        self.shared.frames_received.load(Ordering::Relaxed)
    }

    pub fn ping_by_id(
        channel: &str,
        device_id: u8,
        timeout: Duration,
    ) -> Result<Option<(u16, Vec<u8>)>> {
        let host_id: u8 = 0xFF;
        let (tx_socket, rx_socket) = open_socket_pair(channel, timeout)?;
        drain_scan_socket(&rx_socket);

        let frame = build_frame(
            robstride::make_id(
                CommunicationType::GetDeviceId,
                u16::from(host_id),
                device_id,
            ),
            &[],
        )?;
        tx_socket.write_frame(&frame)?;

        wait_for_scan_response(&rx_socket, device_id, timeout)
    }

    fn wait_for_status_after_send(
        &self,
        actuator: &str,
        actuator_id: u8,
        frame: socketcan::CanFrame,
    ) -> Result<robstride::StatusFrame> {
        let tx_socket = self.require_connected()?;
        let (status_tx, status_rx) = sync_channel(1);

        {
            let mut pending = self.shared.pending_status.lock().expect("pending poisoned");
            if pending.contains_key(&actuator_id) {
                return Err(ActuatorError::Busy(format!(
                    "actuator {actuator:?} already has an in-flight Robstride status request"
                )));
            }
            pending.insert(actuator_id, status_tx);
        }

        if let Err(error) = tx_socket.write_frame(&frame) {
            self.shared
                .pending_status
                .lock()
                .expect("pending poisoned")
                .remove(&actuator_id);
            return Err(error.into());
        }
        self.shared.frames_sent.fetch_add(1, Ordering::Relaxed);

        match status_rx.recv_timeout(COMMAND_TIMEOUT) {
            Ok(status) => {
                if status.is_fault {
                    Err(ActuatorError::Protocol(format!(
                        "Robstride actuator {actuator:?} reported fault(s): {}",
                        status.faults.join(", ")
                    )))
                } else {
                    Ok(status)
                }
            }
            Err(_) => {
                self.shared
                    .pending_status
                    .lock()
                    .expect("pending poisoned")
                    .remove(&actuator_id);
                Err(ActuatorError::Timeout {
                    description: format!("Robstride status for actuator {actuator:?}"),
                    timeout: COMMAND_TIMEOUT,
                })
            }
        }
    }

    fn require_connected(&self) -> Result<&CanSocket> {
        self.tx_socket
            .as_ref()
            .ok_or_else(|| ActuatorError::NotConnected(self.channel.clone()))
    }

    fn require_actuator(&self, actuator: &str) -> Result<&Actuator> {
        self.actuators
            .get(actuator)
            .ok_or_else(|| ActuatorError::UnknownActuator(actuator.to_string()))
    }

    fn calibration_for(&self, actuator: &str) -> Result<Calibration> {
        self.require_actuator(actuator)?;
        Ok(self.calibrations.get(actuator).copied().unwrap_or_default())
    }
}

impl Drop for RobstrideBus {
    fn drop(&mut self) {
        let _ = self.disconnect(false);
    }
}

fn receiver_loop(
    rx_socket: CanSocket,
    shared: Arc<SharedState>,
    actuator_lookup: HashMap<u8, (String, String)>,
    calibrations: HashMap<String, Calibration>,
) {
    while shared.running.load(Ordering::Relaxed) {
        match rx_socket.read_frame() {
            Ok(frame) => {
                shared.frames_received.fetch_add(1, Ordering::Relaxed);
                let Some(parsed) = robstride::parse_frame(&frame) else {
                    continue;
                };

                match parsed.communication_type {
                    CommunicationType::ReadParameter => handle_read_parameter(&shared, parsed),
                    CommunicationType::OperationStatus | CommunicationType::FaultReport => {
                        handle_status_frame(&shared, parsed, &actuator_lookup, &calibrations);
                    }
                    _ => {}
                }
            }
            Err(error) if is_timeout_error(&error) => {}
            Err(_) => {}
        }
    }
}

fn drain_scan_socket(rx_socket: &CanSocket) {
    loop {
        match rx_socket.read_frame() {
            Ok(_) => {}
            Err(error) if is_timeout_error(&error) => return,
            Err(_) => return,
        }
    }
}

fn wait_for_scan_response(
    rx_socket: &CanSocket,
    actuator_id: u8,
    timeout: Duration,
) -> Result<Option<(u16, Vec<u8>)>> {
    let deadline = Instant::now() + timeout;

    loop {
        match rx_socket.read_frame() {
            Ok(frame) => {
                let Some(parsed) = robstride::parse_frame(&frame) else {
                    continue;
                };
                if parsed.communication_type != CommunicationType::GetDeviceId {
                    continue;
                }

                let response_actuator_id = parsed.extra_data as u8;
                if response_actuator_id == actuator_id {
                    return Ok(Some((parsed.extra_data, parsed.data)));
                }
            }
            Err(error) if is_timeout_error(&error) => {
                if Instant::now() >= deadline {
                    return Ok(None);
                }
            }
            Err(error) => return Err(error.into()),
        }
    }
}

fn handle_read_parameter(shared: &SharedState, parsed: robstride::ParsedFrame) {
    let pending = shared
        .pending_reads
        .lock()
        .expect("pending poisoned")
        .take();
    let Some(pending) = pending else {
        return;
    };

    if parsed.data.len() < 8 {
        return;
    }

    let Ok(value) = robstride::decode_parameter_value(pending.parameter, &parsed.data[4..8]) else {
        return;
    };

    if let Some(state) = shared
        .states
        .write()
        .expect("states poisoned")
        .get_mut(&pending.actuator_name)
    {
        match pending.parameter {
            robstride::parameter::MEASURED_POSITION => {
                state.update_position(match value {
                    ParameterValue::Float(value) => f64::from(value),
                    ParameterValue::Integer(_) => return,
                });
            }
            robstride::parameter::MEASURED_VELOCITY => {
                state.update_velocity(match value {
                    ParameterValue::Float(value) => f64::from(value),
                    ParameterValue::Integer(_) => return,
                });
            }
            robstride::parameter::MEASURED_TORQUE => {
                state.update_torque(match value {
                    ParameterValue::Float(value) => f64::from(value),
                    ParameterValue::Integer(_) => return,
                });
            }
            _ => {}
        }
    }

    let _ = pending.tx.send(value);
}

fn handle_status_frame(
    shared: &SharedState,
    parsed: robstride::ParsedFrame,
    actuator_lookup: &HashMap<u8, (String, String)>,
    calibrations: &HashMap<String, Calibration>,
) {
    let actuator_id = (parsed.extra_data & 0xFF) as u8;
    let Some((actuator_name, model)) = actuator_lookup.get(&actuator_id) else {
        return;
    };
    let Ok(status) = robstride::decode_status(model, &parsed) else {
        return;
    };

    {
        let calibration = calibrations.get(actuator_name).copied().unwrap_or_default();
        if let Some(state) = shared
            .states
            .write()
            .expect("states poisoned")
            .get_mut(actuator_name)
        {
            if status.is_fault {
                state.set_faults(status.faults.clone());
            } else {
                let (position, velocity, torque) =
                    calibration.apply_state(status.position, status.velocity, status.torque);
                state.set_all(position, velocity, torque, status.temperature);
                state.set_faults(status.faults.clone());
            }
        }
    }

    if let Some(waiter) = shared
        .pending_status
        .lock()
        .expect("pending poisoned")
        .remove(&actuator_id)
    {
        let _ = waiter.send(status);
    }
}

fn scale_signed(value: f64, limit: f64) -> u16 {
    let value = value.clamp(-limit, limit);
    let scaled = ((value / limit) + 1.0) * 32_767.0;
    scaled.clamp(0.0, 65_535.0) as u16
}

fn scale_unsigned(value: f64, limit: f64) -> u16 {
    let value = value.clamp(0.0, limit);
    let scaled = (value / limit) * 65_535.0;
    scaled.clamp(0.0, 65_535.0) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_scaling_stays_in_range() {
        assert_eq!(scale_signed(-10.0, 5.0), 0);
        assert_eq!(scale_signed(10.0, 5.0), 65_534);
    }
}
