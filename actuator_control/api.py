"""Shared Python-side bus helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class Actuator:
    """Actuator metadata used by the Python API."""

    id: int
    model: str


@dataclass(frozen=True, slots=True)
class ActuatorState:
    """Cached actuator state returned by the Rust backend."""

    position: float
    velocity: float
    torque: float
    temperature: float
    faults: tuple[str, ...]


class BusBase:
    """Shared high-level wrapper behavior for Python bus objects."""

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None,
        bitrate: int,
        rx_thread_priority: int | None,
    ) -> None:
        self.channel = channel
        self.actuators = dict(actuators)
        self.calibration = calibration.copy() if calibration else {}
        self.bitrate = bitrate
        self.rx_thread_priority = rx_thread_priority
        self._is_connected = False

    def __len__(self) -> int:
        return len(self.actuators)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        self._core.connect()
        self._is_connected = True

    def disconnect(self, disable_torque: bool = True) -> None:
        self._core.disconnect(disable_torque=disable_torque)
        self._is_connected = False

    def enable(self, actuator: str) -> None:
        self._core.enable(actuator)

    def disable(self, actuator: str) -> None:
        self._core.disable(actuator)

    def write_mit_kp_kd(self, actuator: str, kp: float, kd: float) -> None:
        self._core.write_mit_kp_kd(actuator, kp, kd)

    def write_mit_control(
        self,
        actuator: str,
        position: float,
        velocity: float = 0.0,
        torque: float = 0.0,
    ) -> None:
        self._core.write_mit_control(actuator, position, velocity, torque)

    def request_state(self, actuator: str) -> None:
        self._core.request_state(actuator)

    @property
    def tx_counter(self) -> int:
        return self._core.read_tx_counter()

    @property
    def rx_counter(self) -> int:
        return self._core.read_rx_counter()

    def get_state(self, actuator: str) -> ActuatorState | None:
        state = self._core.read_state(actuator)
        if state is None:
            return None

        position, velocity, torque, temperature, faults = state
        return ActuatorState(
            position=position,
            velocity=velocity,
            torque=torque,
            temperature=temperature,
            faults=tuple(faults),
        )

    def get_fault_status(self, actuator: str) -> tuple[str, ...] | None:
        state = self.get_state(actuator)
        if state is None:
            return None
        return state.faults

    def get_mit_states(self, actuator: str) -> tuple[float, float, float] | None:
        state = self.get_state(actuator)
        if state is None:
            return None
        return state.position, state.velocity, state.torque

    def get_position(self, actuator: str) -> float | None:
        state = self.get_state(actuator)
        return state.position if state is not None else None

    def get_velocity(self, actuator: str) -> float | None:
        state = self.get_state(actuator)
        return state.velocity if state is not None else None

    def get_torque(self, actuator: str) -> float | None:
        state = self.get_state(actuator)
        return state.torque if state is not None else None

    def get_temperature(self, actuator: str) -> float | None:
        state = self.get_state(actuator)
        return state.temperature if state is not None else None

    @classmethod
    def ping_by_id(
        cls,
        channel: str,
        device_id: int,
        timeout: float = 0.1,
    ) -> tuple[int, bytes] | None:
        raise NotImplementedError(f"{cls.__name__}.ping_by_id() is not implemented yet.")

    @classmethod
    def scan_channel(
        cls,
        channel: str,
        start_id: int = 1,
        end_id: int = 255,
        timeout: float = 0.1,
    ) -> dict[int, tuple[int, bytes]]:
        """Probe one channel and return scan responses keyed by device ID."""
        _validate_scan_range(start_id, end_id)

        device_ids: dict[int, tuple[int, bytes]] = {}
        for device_id in range(start_id, end_id + 1):
            response = cls.ping_by_id(channel, device_id, timeout=timeout)
            if response is None:
                continue

            device_ids[device_id] = response

        return device_ids


def _validate_scan_range(start_id: int, end_id: int) -> None:
    if start_id > end_id:
        raise ValueError("start_id must be less than or equal to end_id.")


def _serialize_actuators(actuators: dict[str, Actuator]) -> dict[str, dict[str, Any]]:
    return {
        name: {
            "id": int(actuator.id),
            "model": str(actuator.model),
        }
        for name, actuator in actuators.items()
    }


def _serialize_calibration(
    calibration: dict[str, dict[str, Any]] | None,
) -> dict[str, dict[str, float]] | None:
    if calibration is None:
        return None

    return {
        name: {
            "direction": float(values.get("direction", 1.0)),
            "homing_offset": float(values.get("homing_offset", 0.0)),
        }
        for name, values in calibration.items()
    }
