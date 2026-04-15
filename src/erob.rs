use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{SyncSender, sync_channel};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use socketcan::{CanSocket, Socket};

use crate::core::{
    Actuator, ActuatorError, ActuatorState, CachedActuatorState, Calibration, ParameterValue,
    Result, build_frame, is_timeout_error, open_socket_pair, validate_unique_ids,
};
use crate::protocol::erob;

const RX_TIMEOUT: Duration = Duration::from_millis(20);
const COMMAND_TIMEOUT: Duration = Duration::from_millis(100);

struct PendingRequest {
    parameter: Option<u16>,
    tx: SyncSender<Vec<u8>>,
}

struct SharedState {
    running: AtomicBool,
    frames_sent: AtomicU64,
    frames_received: AtomicU64,
    pending: Mutex<HashMap<u8, PendingRequest>>,
    states: RwLock<HashMap<String, CachedActuatorState>>,
}

pub struct ErobBus {
    channel: String,
    bitrate: u32,
    actuators: HashMap<String, Actuator>,
    calibrations: HashMap<String, Calibration>,
    ids: HashMap<u8, String>,
    shared: Arc<SharedState>,
    tx_socket: Option<CanSocket>,
    rx_thread: Option<JoinHandle<()>>,
}

impl ErobBus {
    pub fn new(channel: impl Into<String>, actuators: HashMap<String, Actuator>) -> Result<Self> {
        let ids = validate_unique_ids(&actuators)?;
        let states = actuators
            .keys()
            .cloned()
            .map(|name| (name, CachedActuatorState::default()))
            .collect();

        Ok(Self {
            channel: channel.into(),
            bitrate: 1_000_000,
            actuators,
            calibrations: HashMap::new(),
            ids,
            shared: Arc::new(SharedState {
                running: AtomicBool::new(false),
                frames_sent: AtomicU64::new(0),
                frames_received: AtomicU64::new(0),
                pending: Mutex::new(HashMap::new()),
                states: RwLock::new(states),
            }),
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
        let ids = self.ids.clone();

        self.shared.running.store(true, Ordering::Relaxed);
        self.rx_thread = Some(thread::spawn(move || {
            receiver_loop(rx_socket, shared, ids, calibrations);
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
            .pending
            .lock()
            .expect("pending poisoned")
            .clear();

        if let Some(error) = first_error {
            return Err(error);
        }

        Ok(())
    }

    pub fn enable(&self, actuator: &str) -> Result<()> {
        self.require_actuator(actuator)?;

        self.write(
            actuator,
            erob::parameter::CONTROL_MODE,
            i32::from(erob::mode::POSITION),
        )?;
        self.write(actuator, erob::parameter::MOTION_MODE, 1)?;

        let profile_limit = (erob::COUNTS_PER_RAD * 100.0 * (2.0 * std::f64::consts::PI)) as i32;
        self.write(actuator, erob::parameter::ACCELERATION, profile_limit)?;
        self.write(actuator, erob::parameter::DECELERATION, profile_limit)?;
        self.write(actuator, erob::parameter::TARGET_SPEED, profile_limit)?;

        let _ = self.read(actuator, erob::parameter::ERROR_CODE)?;
        thread::sleep(Duration::from_millis(500));

        let payload = [0x01, 0x00, 0x00, 0x00, 0x00, 0x01];
        self.send_and_wait(actuator, None, &payload)?;
        Ok(())
    }

    pub fn disable(&self, actuator: &str) -> Result<()> {
        self.send_command(actuator, erob::command::STOP_MOTION)?;
        let payload = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.send_and_wait(actuator, None, &payload)?;
        Ok(())
    }

    pub fn read(&self, actuator: &str, parameter: u16) -> Result<ParameterValue> {
        let value = self.read_raw(actuator, parameter, &[])?;
        Ok(ParameterValue::Integer(i64::from(value)))
    }

    pub fn write(&self, actuator: &str, parameter: u16, value: i32) -> Result<()> {
        let mut data = Vec::with_capacity(6);
        data.extend_from_slice(&parameter.to_be_bytes());
        data.extend_from_slice(&value.to_be_bytes());
        self.send_and_wait(actuator, None, &data)?;
        Ok(())
    }

    pub fn write_mit_kp_kd(&self, actuator: &str, kp: f64, kd: f64) -> Result<()> {
        if matches!(
            self.read(actuator, erob::parameter::PID_ADJUSTMENT)?,
            ParameterValue::Integer(1)
        ) {
            self.write(actuator, erob::parameter::PID_ADJUSTMENT, 0)?;
        }

        let (kp_raw, kd_raw) = calculate_erob_pd(kp, kd);
        self.write_subindex(actuator, erob::parameter::POSITION_LOOP_GAIN, 0x01, kp_raw)?;
        self.write_subindex(actuator, erob::parameter::SPEED_LOOP_GAIN, 0x01, kd_raw)?;
        self.write_subindex(actuator, erob::parameter::SPEED_LOOP_INTEGRAL, 0x01, 0)?;
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
        let calibration = self.calibration_for(actuator)?;
        let (position, _, _) = calibration.apply_command(position, velocity, torque);
        let target_counts = ((position + std::f64::consts::PI) * erob::COUNTS_PER_RAD) as i32;

        let mut write_payload = Vec::with_capacity(6);
        write_payload.extend_from_slice(&erob::parameter::TARGET_POSITION.to_be_bytes());
        write_payload.extend_from_slice(&target_counts.to_be_bytes());
        self.send_frame(actuator_config.id, &write_payload)?;
        self.send_frame(actuator_config.id, &[0x00, erob::command::START_MOTION])?;
        Ok(())
    }

    pub fn request_state(&self, actuator: &str) -> Result<()> {
        let _ = self.read_raw(actuator, erob::parameter::ACTUAL_POSITION, &[])?;
        let _ = self.read_raw(actuator, erob::parameter::ACTUAL_SPEED, &[0x00, 0x01])?;
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

    pub fn bitrate(&self) -> u32 {
        self.bitrate
    }

    pub fn tx_counter(&self) -> u64 {
        self.shared.frames_sent.load(Ordering::Relaxed)
    }

    pub fn rx_counter(&self) -> u64 {
        self.shared.frames_received.load(Ordering::Relaxed)
    }

    fn send_command(&self, actuator: &str, command: u8) -> Result<()> {
        self.send_and_wait(actuator, None, &[0x00, command])?;
        Ok(())
    }

    fn send_frame(&self, actuator_id: u8, payload: &[u8]) -> Result<()> {
        let tx_socket = self.require_connected()?;
        let frame = build_frame(erob::client_id(actuator_id), payload)?;
        tx_socket.write_frame(&frame)?;
        self.shared.frames_sent.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn read_raw(&self, actuator: &str, parameter: u16, extra_bytes: &[u8]) -> Result<i32> {
        let mut data = Vec::with_capacity(2 + extra_bytes.len());
        data.extend_from_slice(&parameter.to_be_bytes());
        data.extend_from_slice(extra_bytes);

        let payload = self.send_and_wait(actuator, Some(parameter), &data)?;
        decode_signed_be(&payload)
    }

    fn write_subindex(
        &self,
        actuator: &str,
        parameter: u16,
        subindex: u16,
        value: i32,
    ) -> Result<()> {
        let mut data = Vec::with_capacity(8);
        data.extend_from_slice(&parameter.to_be_bytes());
        data.extend_from_slice(&subindex.to_be_bytes());
        data.extend_from_slice(&value.to_be_bytes());
        self.send_and_wait(actuator, None, &data)?;
        Ok(())
    }

    fn send_and_wait(
        &self,
        actuator: &str,
        parameter: Option<u16>,
        payload: &[u8],
    ) -> Result<Vec<u8>> {
        let actuator_config = self.require_actuator(actuator)?;
        let tx_socket = self.require_connected()?;
        let frame = build_frame(erob::client_id(actuator_config.id), payload)?;
        let (reply_tx, reply_rx) = sync_channel(1);

        {
            let mut pending = self.shared.pending.lock().expect("pending poisoned");
            if pending.contains_key(&actuator_config.id) {
                return Err(ActuatorError::Busy(format!(
                    "actuator {actuator:?} already has an in-flight eRob request"
                )));
            }

            pending.insert(
                actuator_config.id,
                PendingRequest {
                    parameter,
                    tx: reply_tx,
                },
            );
        }

        if let Err(error) = tx_socket.write_frame(&frame) {
            self.shared
                .pending
                .lock()
                .expect("pending poisoned")
                .remove(&actuator_config.id);
            return Err(error.into());
        }
        self.shared.frames_sent.fetch_add(1, Ordering::Relaxed);

        match reply_rx.recv_timeout(COMMAND_TIMEOUT) {
            Ok(payload) => Ok(payload),
            Err(_) => {
                self.shared
                    .pending
                    .lock()
                    .expect("pending poisoned")
                    .remove(&actuator_config.id);
                Err(ActuatorError::Timeout {
                    description: format!("eRob response for actuator {actuator:?}"),
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

impl Drop for ErobBus {
    fn drop(&mut self) {
        let _ = self.disconnect(false);
    }
}

fn receiver_loop(
    rx_socket: CanSocket,
    shared: Arc<SharedState>,
    ids: HashMap<u8, String>,
    calibrations: HashMap<String, Calibration>,
) {
    while shared.running.load(Ordering::Relaxed) {
        match rx_socket.read_frame() {
            Ok(frame) => {
                shared.frames_received.fetch_add(1, Ordering::Relaxed);
                let Some(response) = erob::parse_response(&frame) else {
                    continue;
                };

                let pending = shared
                    .pending
                    .lock()
                    .expect("pending poisoned")
                    .remove(&response.actuator_id);

                let Some(pending) = pending else {
                    continue;
                };

                if let Some(parameter) = pending.parameter {
                    if let Some(name) = ids.get(&response.actuator_id) {
                        let calibration = calibrations.get(name).copied().unwrap_or_default();
                        update_erob_state(&shared, name, calibration, parameter, &response.payload);
                    }
                }

                let _ = pending.tx.send(response.payload);
            }
            Err(error) if is_timeout_error(&error) => {}
            Err(_) => {}
        }
    }
}

fn update_erob_state(
    shared: &SharedState,
    actuator: &str,
    calibration: Calibration,
    parameter: u16,
    payload: &[u8],
) {
    let Ok(raw_value) = decode_signed_be(payload) else {
        return;
    };

    let mut states = shared.states.write().expect("states poisoned");
    let Some(state) = states.get_mut(actuator) else {
        return;
    };

    match parameter {
        erob::parameter::ACTUAL_POSITION => {
            let raw_position = f64::from(raw_value) / erob::COUNTS_PER_RAD - std::f64::consts::PI;
            let (position, _, _) = calibration.apply_state(raw_position, 0.0, 0.0);
            state.update_position(position);
        }
        erob::parameter::ACTUAL_SPEED => {
            let raw_velocity = f64::from(raw_value) / erob::COUNTS_PER_RAD;
            let (_, velocity, _) = calibration.apply_state(0.0, raw_velocity, 0.0);
            state.update_velocity(velocity);
        }
        _ => {}
    }
}

fn decode_signed_be(payload: &[u8]) -> Result<i32> {
    match payload.len() {
        2 => Ok(i16::from_be_bytes([payload[0], payload[1]]) as i32),
        4 => Ok(i32::from_be_bytes([
            payload[0], payload[1], payload[2], payload[3],
        ])),
        _ => Err(ActuatorError::Protocol(format!(
            "unsupported eRob payload length {}",
            payload.len()
        ))),
    }
}

fn calculate_erob_pd(desired_kp: f64, desired_kd: f64) -> (i32, i32) {
    if desired_kd == 0.0 {
        return (0, 0);
    }

    let torque_constant = 0.132e-3;
    let gear_ratio = 50.0;
    let position_gain = 1000.0 / 37.5e-4;
    let velocity_gain = 1.0 / 19.3e-5;

    let ma_per_count = desired_kd / (torque_constant * erob::COUNTS_PER_RAD * gear_ratio);
    let kd_erob = ma_per_count * velocity_gain;
    let kp_erob = (desired_kp * position_gain) / (desired_kd * velocity_gain);

    (kp_erob.round() as i32, kd_erob.round() as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_round_trip_matches_python_behavior() {
        let calibration = Calibration::new(-1.0, 0.25).unwrap();
        let (position, velocity, torque) = calibration.apply_command(1.0, 2.0, 3.0);
        assert!((position + 0.75).abs() < 1e-9);
        assert!((velocity + 2.0).abs() < 1e-9);
        assert!((torque + 3.0).abs() < 1e-9);

        let (position, velocity, torque) = calibration.apply_state(position, velocity, torque);
        assert!((position - 1.0).abs() < 1e-9);
        assert!((velocity - 2.0).abs() < 1e-9);
        assert!((torque - 3.0).abs() < 1e-9);
    }
}
