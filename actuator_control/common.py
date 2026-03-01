from dataclasses import dataclass
from functools import cached_property
from typing import TypeAlias

import can


Value: TypeAlias = int | float


@dataclass
class Motor:
    id: int
    model: str


class ActuatorBus:
    def __init__(
        self,
        channel: str,
        motors: dict[str, Motor],
        calibration: dict[str, dict] | None = None,
        bitrate: int = 1000000,
    ):
        self.channel = channel
        self.motors = motors
        self.calibration = calibration
        self.bitrate = bitrate

        if self.calibration:
            print(f"Using calibration: {self.calibration}")
        else:
            print("WARNING: No calibration provided")

        self.channel_handler: can.BusABC | None = None

    def __len__(self):
        return len(self.motors)

    def __repr__(self):
        return (
            f"{self.__class__.__name__}(\n"
            f"    Channel: '{self.channel}',\n"
            f"    Motors: \n{self.motors},\n"
            ")',\n"
        )

    def __del__(self):
        if self.is_connected:
            self.disconnect()

    @cached_property
    def models(self) -> list[str]:
        return [m.model for m in self.motors.values()]

    @cached_property
    def ids(self) -> list[int]:
        return [m.id for m in self.motors.values()]

    @property
    def is_connected(self) -> bool:
        """bool: `True` if the underlying CAN bus is open."""
        return self.channel_handler is not None

    def connect(self, handshake: bool = True) -> None:
        """Open the serial port and initialise communication.
        """
        if self.is_connected:
            raise Exception(
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
        """Close the serial port (optionally disabling torque first).
        """
        if not self.is_connected:
            raise Exception(
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

    def enable(self, motor: str):
        """
        Enable the motor.
        """
        raise NotImplementedError("Not implemented")

    def disable(self, motor: str):
        """
        Disable the motor.
        """
        raise NotImplementedError("Not implemented")

    def read(self, motor: str, parameter: int) -> Value:
        raise NotImplementedError("Not implemented")

    def write(self, motor: str, parameter: int, value: Value):
        raise NotImplementedError("Not implemented")

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float):
        raise NotImplementedError("Not implemented")

    def _process_mit_control(self, motor: str, position: float, velocity: float, torque: float) -> tuple[float, float, float]:
        if self.calibration:
            # convert to raw motor frame
            calibration = self.calibration[motor]
            position = position * calibration["direction"] + calibration["homing_offset"]
            velocity = velocity * calibration["direction"]
            torque = torque * calibration["direction"]

        return position, velocity, torque

    def _process_mit_state(self, motor: str, position: float, velocity: float) -> tuple[float, float]:
        if self.calibration:
            calibration = self.calibration[motor]
            position = (position - calibration["homing_offset"]) * calibration["direction"]
            velocity = velocity * calibration["direction"]

        return position, velocity

    def write_mit_control(self, motor: str, position: float, velocity: float = 0, torque: float = 0):
        """
        Write the control frame to the motor.

        Args:
            motor: Motor name.
            position: Target position in rad.
            velocity: Target velocity in rad/s.
            torque: Target feedforward torque in Nm.
        """
        raise NotImplementedError("Not implemented")

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        """
        Read the state of the motor.

        Args:
            motor: Motor name.

        Returns:
            tuple[float, float]:
                Measured position in rad.
                Measured velocity in rad/s.
        """
        raise NotImplementedError("Not implemented")