use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use socketcan::{CanSocket, Socket};

use crate::core::{
    Actuator, ActuatorError, ActuatorState, CachedActuatorState, Calibration, Result,
    apply_thread_priority, build_frame, is_timeout_error, open_socket_pair,
    validate_thread_priority, validate_unique_ids,
};
use crate::protocol::sito::{self, SitoModelConfig};

const RX_TIMEOUT: Duration = Duration::from_millis(20);

struct SharedState {
    running: AtomicBool,
    frames_sent: AtomicU64,
    frames_received: AtomicU64,
    states: RwLock<HashMap<String, CachedActuatorState>>,
}

pub struct SitoBus {
    channel: String,
    bitrate: u32,
    feedback_1_interval_ms: u8,
    feedback_2_interval_ms: u8,
    feedback_3_interval_ms: u8,
    actuators: HashMap<String, Actuator>,
    calibrations: HashMap<String, Calibration>,
    ids: HashMap<u8, String>,
    rx_thread_priority: Option<i32>,
    shared: Arc<SharedState>,
    tx_socket: Option<CanSocket>,
    rx_thread: Option<JoinHandle<()>>,
}

impl SitoBus {
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
            feedback_1_interval_ms: 20,
            feedback_2_interval_ms: 20,
            feedback_3_interval_ms: 0,
            actuators,
            calibrations: HashMap::new(),
            ids,
            rx_thread_priority: None,
            shared: Arc::new(SharedState {
                running: AtomicBool::new(false),
                frames_sent: AtomicU64::new(0),
                frames_received: AtomicU64::new(0),
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

    pub fn with_control_frequency(mut self, control_frequency_hz: f64) -> Result<Self> {
        if control_frequency_hz <= 0.0 {
            return Err(ActuatorError::Protocol(
                "control frequency must be greater than zero".to_string(),
            ));
        }

        let interval_ms = (1000.0 / control_frequency_hz).round() as u16;
        if interval_ms == 0 || interval_ms >= u16::from(u8::MAX) {
            return Err(ActuatorError::Protocol(
                "control frequency produces an invalid Sito feedback interval".to_string(),
            ));
        }

        self.feedback_1_interval_ms = interval_ms as u8;
        self.feedback_2_interval_ms = interval_ms as u8;
        Ok(self)
    }

    pub fn with_calibrations(mut self, calibrations: HashMap<String, Calibration>) -> Self {
        self.calibrations = calibrations;
        self
    }

    pub fn with_rx_thread_priority(mut self, priority: i32) -> Result<Self> {
        validate_thread_priority(priority)?;
        self.rx_thread_priority = Some(priority);
        Ok(self)
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
        let models = self
            .actuators
            .iter()
            .map(|(name, actuator)| (name.clone(), actuator.model.clone()))
            .collect::<HashMap<_, _>>();
        let rx_thread_priority = self.rx_thread_priority;

        self.shared.running.store(true, Ordering::Relaxed);
        let handle = thread::spawn(move || {
            receiver_loop(rx_socket, shared, ids, models, calibrations);
        });
        if let Some(priority) = rx_thread_priority {
            if let Err(error) = apply_thread_priority(&handle, priority) {
                self.shared.running.store(false, Ordering::Relaxed);
                let _ = handle.join();
                return Err(error);
            }
        }
        self.rx_thread = Some(handle);
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

        if let Some(error) = first_error {
            return Err(error);
        }

        Ok(())
    }

    pub fn enable(&self, actuator: &str) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let payload = [
            sito::mode::MIT,
            self.feedback_1_interval_ms,
            self.feedback_2_interval_ms,
            self.feedback_3_interval_ms,
            0x00,
            0x00,
            0x00,
            0x00,
        ];
        self.send_frame(
            actuator_config.id,
            sito::communication::SELECT_MODE,
            &payload,
        )
    }

    pub fn disable(&self, actuator: &str) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        self.send_frame(
            actuator_config.id,
            sito::communication::RESET,
            &[0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55],
        )
    }

    pub fn write_mit_kp_kd(&self, actuator: &str, kp: f64, kd: f64) -> Result<()> {
        let actuator_config = self.require_actuator(actuator)?;
        let kp = (kp / 500.0) as f32;
        let kd = (kd / 0.5) as f32;
        let mut payload = Vec::with_capacity(8);
        payload.extend_from_slice(&kp.to_be_bytes());
        payload.extend_from_slice(&kd.to_be_bytes());
        self.send_frame(
            actuator_config.id,
            sito::communication::SET_MIT_KP_KD,
            &payload,
        )
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
        let config = self.model_config(actuator)?;
        let (position, velocity, torque) = calibration.apply_command(position, velocity, torque);

        let current = (torque / config.torque_constant / config.gear_ratio) as i16;
        let velocity_counts = (velocity * config.counts_per_rad) as i16;
        let position_counts = (position * config.counts_per_rad) as i32;

        let mut payload = Vec::with_capacity(8);
        payload.extend_from_slice(&current.to_be_bytes());
        payload.extend_from_slice(&velocity_counts.to_be_bytes());
        payload.extend_from_slice(&position_counts.to_be_bytes());
        self.send_frame(
            actuator_config.id,
            sito::communication::SET_MIT_CURRENT_VELOCITY_POSITION,
            &payload,
        )
    }

    pub fn request_state(&self, actuator: &str) -> Result<()> {
        self.require_actuator(actuator)?;
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

    fn send_frame(&self, actuator_id: u8, message_type: u8, payload: &[u8]) -> Result<()> {
        let tx_socket = self.require_connected()?;
        let frame = build_frame(sito::make_id(actuator_id, message_type), payload)?;
        tx_socket.write_frame(&frame)?;
        self.shared.frames_sent.fetch_add(1, Ordering::Relaxed);
        Ok(())
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

    fn model_config(&self, actuator: &str) -> Result<SitoModelConfig> {
        let actuator_config = self.require_actuator(actuator)?;
        sito::model_config(&actuator_config.model).ok_or_else(|| ActuatorError::UnsupportedModel {
            backend: "Sito",
            actuator: actuator.to_string(),
            model: actuator_config.model.clone(),
        })
    }
}

impl Drop for SitoBus {
    fn drop(&mut self) {
        let _ = self.disconnect(false);
    }
}

fn receiver_loop(
    rx_socket: CanSocket,
    shared: Arc<SharedState>,
    ids: HashMap<u8, String>,
    models: HashMap<String, String>,
    calibrations: HashMap<String, Calibration>,
) {
    while shared.running.load(Ordering::Relaxed) {
        match rx_socket.read_frame() {
            Ok(frame) => {
                shared.frames_received.fetch_add(1, Ordering::Relaxed);
                let Some(feedback) = sito::parse_feedback(&frame) else {
                    continue;
                };
                let Some(actuator_name) = ids.get(&feedback.actuator_id) else {
                    continue;
                };
                let Some(model) = models.get(actuator_name) else {
                    continue;
                };
                let Some(config) = sito::model_config(model) else {
                    continue;
                };
                let calibration = calibrations.get(actuator_name).copied().unwrap_or_default();

                if let Some(state) = shared
                    .states
                    .write()
                    .expect("states poisoned")
                    .get_mut(actuator_name)
                {
                    handle_feedback(state, config, calibration, &feedback);
                }
            }
            Err(error) if is_timeout_error(&error) => {}
            Err(_) => {}
        }
    }
}

fn handle_feedback(
    state: &mut CachedActuatorState,
    config: SitoModelConfig,
    calibration: Calibration,
    feedback: &sito::ParsedFeedback,
) {
    match feedback.message_type {
        sito::communication::FEEDBACK_1 if feedback.data.len() == 8 => {
            let phase_current = i16::from_be_bytes([feedback.data[2], feedback.data[3]]) as f64;
            let velocity_counts = i16::from_be_bytes([feedback.data[6], feedback.data[7]]) as f64;

            let velocity = velocity_counts / config.counts_per_rad;
            let torque = phase_current * config.torque_constant * config.gear_ratio;
            let (_, velocity, torque) = calibration.apply_state(0.0, velocity, torque);
            state.update_velocity(velocity);
            state.update_torque(torque);
        }
        sito::communication::FEEDBACK_2 if feedback.data.len() == 8 => {
            let output_position_counts = i32::from_be_bytes([
                feedback.data[4],
                feedback.data[5],
                feedback.data[6],
                feedback.data[7],
            ]) as f64;
            let position = output_position_counts / config.counts_per_rad;
            let (position, _, _) = calibration.apply_state(position, 0.0, 0.0);
            state.update_position(position);
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_frequency_maps_to_feedback_interval() {
        let actuators = HashMap::from([("joint".to_string(), Actuator::new(1, "TA40-50"))]);
        let bus = SitoBus::new("can0", actuators)
            .unwrap()
            .with_control_frequency(50.0)
            .unwrap();
        assert_eq!(bus.feedback_1_interval_ms, 20);
    }

    #[test]
    fn rx_thread_priority_rejects_invalid_values() {
        let actuators = HashMap::from([("joint".to_string(), Actuator::new(1, "TA40-50"))]);
        let result = SitoBus::new("can0", actuators)
            .unwrap()
            .with_rx_thread_priority(0);

        match result {
            Ok(_) => panic!("priority 0 should be rejected"),
            Err(error) => assert!(
                error
                    .to_string()
                    .contains("rx thread priority must be between")
            ),
        }
    }
}
