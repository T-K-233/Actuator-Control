use std::collections::{HashMap, HashSet};

use crate::core::{ActuatorError, Result};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Actuator {
    pub id: u8,
    pub model: String,
}

impl Actuator {
    pub fn new(id: u8, model: impl Into<String>) -> Self {
        Self {
            id,
            model: model.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    direction: f64,
    homing_offset: f64,
}

impl Calibration {
    pub fn new(direction: f64, homing_offset: f64) -> Result<Self> {
        if (direction - 1.0).abs() > f64::EPSILON && (direction + 1.0).abs() > f64::EPSILON {
            return Err(ActuatorError::InvalidCalibration(direction));
        }

        Ok(Self {
            direction,
            homing_offset,
        })
    }

    pub fn direction(self) -> f64 {
        self.direction
    }

    pub fn homing_offset(self) -> f64 {
        self.homing_offset
    }

    pub fn apply_command(self, position: f64, velocity: f64, torque: f64) -> (f64, f64, f64) {
        (
            position * self.direction + self.homing_offset,
            velocity * self.direction,
            torque * self.direction,
        )
    }

    pub fn apply_state(self, position: f64, velocity: f64, torque: f64) -> (f64, f64, f64) {
        (
            (position - self.homing_offset) * self.direction,
            velocity * self.direction,
            torque * self.direction,
        )
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            direction: 1.0,
            homing_offset: 0.0,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CachedActuatorState {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub temperature: f64,
    pub faults: Vec<String>,
    pub has_feedback: bool,
}

impl CachedActuatorState {
    pub fn update_position(&mut self, position: f64) {
        self.position = position;
        self.has_feedback = true;
    }

    pub fn update_velocity(&mut self, velocity: f64) {
        self.velocity = velocity;
        self.has_feedback = true;
    }

    pub fn update_torque(&mut self, torque: f64) {
        self.torque = torque;
        self.has_feedback = true;
    }

    pub fn update_temperature(&mut self, temperature: f64) {
        self.temperature = temperature;
        self.has_feedback = true;
    }

    pub fn set_all(&mut self, position: f64, velocity: f64, torque: f64, temperature: f64) {
        self.position = position;
        self.velocity = velocity;
        self.torque = torque;
        self.temperature = temperature;
        self.has_feedback = true;
    }

    pub fn set_faults(&mut self, faults: Vec<String>) {
        self.faults = faults;
    }

    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    pub fn state(&self) -> Option<ActuatorState> {
        (self.has_feedback || !self.faults.is_empty()).then_some(ActuatorState {
            position: self.position,
            velocity: self.velocity,
            torque: self.torque,
            temperature: self.temperature,
            faults: self.faults.clone(),
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorState {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
    pub temperature: f64,
    pub faults: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterValue {
    Integer(i64),
    Float(f32),
}

impl ParameterValue {
    pub fn as_i64(self) -> Option<i64> {
        match self {
            Self::Integer(value) => Some(value),
            Self::Float(_) => None,
        }
    }

    pub fn as_f32(self) -> Option<f32> {
        match self {
            Self::Integer(_) => None,
            Self::Float(value) => Some(value),
        }
    }
}

pub fn validate_unique_ids(actuators: &HashMap<String, Actuator>) -> Result<HashMap<u8, String>> {
    let mut seen = HashSet::new();
    let mut ids = HashMap::with_capacity(actuators.len());

    for (name, actuator) in actuators {
        if !seen.insert(actuator.id) {
            return Err(ActuatorError::DuplicateActuatorId(actuator.id));
        }
        ids.insert(actuator.id, name.clone());
    }

    Ok(ids)
}
