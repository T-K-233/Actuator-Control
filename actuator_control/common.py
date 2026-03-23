"""Common abstractions shared by all actuator bus backends."""

from dataclasses import dataclass
from functools import cached_property
from typing import Any, TypeAlias

import can


Value: TypeAlias = int | float
Calibration: TypeAlias = dict[str, dict[str, Any]]


@dataclass
class Motor:
    """Metadata needed to address a motor on a CAN bus."""

    id: int
    model: str


class ActuatorBus:
    """Base interface implemented by all actuator control backends."""

    def __init__(
        self,
        channel: str,
        motors: dict[str, Motor],
        calibration: Calibration | None = None,
        bitrate: int = 1000000,
    ) -> None:
        self.channel = channel
        self.motors = motors
        self.calibration = calibration
        self.bitrate = bitrate

        motor_ids = [motor.id for motor in motors.values()]
        if len(motor_ids) != len(set(motor_ids)):
            raise ValueError("Motor IDs must be unique within an actuator bus.")

        if self.calibration:
            print(f"Using calibration: {self.calibration}")
        else:
            print("WARNING: No calibration provided")

        self.channel_handler: can.BusABC | None = None

    def __len__(self) -> int:
        return len(self.motors)

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}("
            f"channel={self.channel!r}, "
            f"motors={self.motors!r}, "
            f"bitrate={self.bitrate!r})"
        )

    def __del__(self) -> None:
        try:
            if self.is_connected:
                self.disconnect()
        except Exception:
            # Destructors run during interpreter shutdown, where module globals may
            # already be gone. Cleanup is best-effort only.
            pass

    @cached_property
    def models(self) -> list[str]:
        """Sorted-by-insertion motor model names for the configured bus."""
        return [m.model for m in self.motors.values()]

    @cached_property
    def ids(self) -> list[int]:
        """Sorted-by-insertion motor IDs for the configured bus."""
        return [m.id for m in self.motors.values()]

    @property
    def is_connected(self) -> bool:
        """bool: `True` if the underlying CAN bus is open."""
        return self.channel_handler is not None

    def connect(self, handshake: bool = True) -> None:
        """Open the CAN interface and initialise communication."""
        if self.is_connected:
            raise RuntimeError(
                f"{self.__class__.__name__}('{self.channel}') is already connected. "
                f"Do not call `{self.__class__.__name__}.connect()` twice."
            )

        self.channel_handler = can.interface.Bus(
            interface="socketcan",
            channel=self.channel,
            bitrate=self.bitrate,
        )
        print(f"{self.__class__.__name__} connected.")

    def disconnect(self, disable_torque: bool = True) -> None:
        """Close the CAN interface, optionally disabling torque first."""
        if not self.is_connected:
            raise RuntimeError(
                f"{self.__class__.__name__}('{self.channel}') is not connected. "
                f"Try running `{self.__class__.__name__}.connect()` first."
            )

        if disable_torque:
            for motor in self.motors:
                self.disable(motor)
            print("Torque disabled for all motors.")

        if self.channel_handler is not None:
            self.channel_handler.shutdown()
            self.channel_handler = None

        print(f"{self.__class__.__name__} disconnected.")

    def _require_connected(self) -> can.BusABC:
        """Return the open CAN bus handle or raise a helpful error."""
        if self.channel_handler is None:
            raise RuntimeError(
                f"{self.__class__.__name__}('{self.channel}') is not connected. "
                f"Try running `{self.__class__.__name__}.connect()` first."
            )
        return self.channel_handler

    def _require_motor(self, motor: str) -> Motor:
        """Return motor metadata or raise a helpful error for unknown names."""
        try:
            return self.motors[motor]
        except KeyError as exc:
            available = ", ".join(self.motors.keys()) or "<none>"
            raise KeyError(f"Unknown motor {motor!r}. Available motors: {available}.") from exc

    def enable(self, motor: str) -> None:
        """Enable the requested motor."""
        raise NotImplementedError("Not implemented")

    def disable(self, motor: str) -> None:
        """Disable the requested motor."""
        raise NotImplementedError("Not implemented")

    def read(self, motor: str, parameter: int) -> Value | None:
        """Read a raw backend-specific parameter."""
        raise NotImplementedError("Not implemented")

    def write(self, motor: str, parameter: int, value: Value) -> None:
        """Write a raw backend-specific parameter."""
        raise NotImplementedError("Not implemented")

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float) -> None:
        """Configure MIT-style proportional and derivative gains for a motor."""
        raise NotImplementedError("Not implemented")

    def _process_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float,
        torque: float,
    ) -> tuple[float, float, float]:
        """Map user-space MIT commands into the calibrated motor frame."""
        if self.calibration:
            # convert to raw motor frame
            calibration = self.calibration[motor]
            position = position * calibration["direction"] + calibration["homing_offset"]
            velocity = velocity * calibration["direction"]
            torque = torque * calibration["direction"]

        return position, velocity, torque

    def _process_mit_state(self, motor: str, position: float, velocity: float) -> tuple[float, float]:
        """Map calibrated motor-state values back into the user frame."""
        if self.calibration:
            calibration = self.calibration[motor]
            position = (position - calibration["homing_offset"]) * calibration["direction"]
            velocity = velocity * calibration["direction"]

        return position, velocity

    def write_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float = 0,
        torque: float = 0,
    ) -> None:
        """Send an MIT-style control command to a motor."""
        raise NotImplementedError("Not implemented")

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        """Read measured position and velocity for a motor."""
        raise NotImplementedError("Not implemented")
