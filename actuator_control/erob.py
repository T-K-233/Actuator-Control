"""eRob actuator backend."""

import struct
import time

import can
import numpy as np

from .common import Value, ActuatorBus


class MotorConfig:
    """Shared scaling constants for the eRob position controller."""

    counts_per_rad: float = 524287 / (2 * np.pi)  # counts/rads = 524287 / (2*π)


class CommandCode:
    """Command identifiers used by the eRob CAN protocol."""

    START_MOTION = 0x83
    STOP_MOTION = 0x84
    SAVE_PARAMS = 0xE8


class Parameter:
    """Register identifiers used by the eRob CAN protocol."""

    ACTUAL_POSITION = 0x02
    ACTUAL_SPEED = 0x05
    MOTOR_CURRENT = 0x08
    ERROR_CODE = 0x1F
    RUN_STATUS = 0x20
    POWER_TEMP = 0x26
    CONTROL_MODE = 0x4E
    MAX_POSITION_ERROR = 0x54
    MAX_SPEED = 0x55
    TARGET_POSITION = 0x86
    RELATIVE_POSITION = 0x87
    ACCELERATION = 0x88
    DECELERATION = 0x89
    TARGET_SPEED = 0x8A
    MOTION_MODE = 0x8D

    PID_ADJUSTMENT = 0x0124

    # loop gains（refer to eRob parameter table）
    POSITION_LOOP_GAIN = 0x64
    SPEED_LOOP_GAIN = 0x66
    SPEED_LOOP_INTEGRAL = 0x67


class Mode:
    """Control modes accepted by eRob actuators."""

    TORQUE = 1
    SPEED = 2
    POSITION = 3


class ERobBus(ActuatorBus):
    """CAN backend for eRob actuators using the position-mode protocol."""

    CLIENT_ID_BASE = 0x640
    SERVER_ID_BASE = 0x5C0

    STATUS_SUCCESS = 0x3E

    def receive(self, device_id: int, timeout: float = 0.1) -> bytes | None:
        """Receive and decode a response frame for a specific motor ID."""
        bus = self._require_connected()
        expected_arbitration_id = self.SERVER_ID_BASE | device_id
        deadline = time.monotonic() + timeout

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            frame = bus.recv(timeout=remaining)
            if frame is None:
                break
            if frame.arbitration_id != expected_arbitration_id:
                continue

            if len(frame.data) == 1 and frame.data[0] == self.STATUS_SUCCESS:
                return b""

            if len(frame.data) >= 5 and frame.data[4] == self.STATUS_SUCCESS:
                return bytes(frame.data[0:4])

            # On hardware we have only seen 3-byte replies ending in STATUS_SUCCESS
            # as error/status packets rather than 2-byte payloads.
            if len(frame.data) == 3 and frame.data[2] == self.STATUS_SUCCESS:
                return None

            return None

        if timeout > 0:
            print("No frame received")
        return None

    def transmit(self, device_id: int, data: bytes = b"\x00\x00\x00\x00\x00\x00\x00\x00") -> None:
        """Transmit a raw eRob payload to one device."""
        arbitration_id = self.CLIENT_ID_BASE | device_id
        frame = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            data=data,
        )
        self._require_connected().send(frame)

    def send_command(self, motor: str, command: int) -> None:
        """Send a protocol command opcode to a motor."""
        motor_config = self._require_motor(motor)
        data = struct.pack(">BB", 0x00, command)
        self.transmit(motor_config.id, data)
        self.receive(motor_config.id, timeout=0.1)

    def disable(self, motor: str) -> None:
        """Stop actuator motion and clear the motion-enable flag."""
        motor_config = self._require_motor(motor)
        self.send_command(motor, CommandCode.STOP_MOTION)
        self.transmit(motor_config.id, struct.pack(">BBL", 0x01, 0x00, 0x00000000))
        self.receive(motor_config.id, timeout=0.1)

    def enable(self, motor: str) -> None:
        """Prepare an eRob actuator for repeated position commands."""
        motor_config = self._require_motor(motor)

        # set control mode to position
        self.write(motor, Parameter.CONTROL_MODE, Mode.POSITION)
        self.write(motor, Parameter.MOTION_MODE, 1)

        # set speed profile to a very large value to avoid
        # the trajectory planner to perform any clipping
        accel = int(MotorConfig.counts_per_rad * 100 * (2 * np.pi))
        decel = int(MotorConfig.counts_per_rad * 100 * (2 * np.pi))
        speed = int(MotorConfig.counts_per_rad * 100 * (2 * np.pi))
        self.write(motor, Parameter.ACCELERATION, accel)
        self.write(motor, Parameter.DECELERATION, decel)
        self.write(motor, Parameter.TARGET_SPEED, speed)

        # clear error code
        self.read(motor, Parameter.ERROR_CODE)

        # from user manual: wait for at least 500 ms before sending motion command
        time.sleep(0.5)
        self.transmit(motor_config.id, struct.pack(">BBL", 0x01, 0x00, 0x00000001))
        self.receive(motor_config.id, timeout=0.1)

    def read(self, motor: str, parameter: int, extra_bytes: bytes = b"") -> Value | None:
        """Read a raw eRob register value."""
        motor_config = self._require_motor(motor)
        data = struct.pack(">H", parameter) + extra_bytes
        self.transmit(motor_config.id, data)
        result = self.receive(motor_config.id, timeout=0.1)

        if result is None:
            return None

        if len(result) == 4:
            value, = struct.unpack(">l", result)
            return value

        if len(result) == 2:
            value, = struct.unpack(">h", result)
            return value

        return None

    def write(self, motor: str, param: int, value: int) -> None:
        """Write a raw eRob register value."""
        motor_config = self._require_motor(motor)
        if param == Parameter.CONTROL_MODE or param == Parameter.MOTION_MODE:
            value = value & 0xFF
        data = struct.pack(">Hl", param, value)
        self.transmit(motor_config.id, data)
        self.receive(motor_config.id, timeout=0.1)

    def write_subindex(self, motor: str, param: int, subindex: int, value: int) -> None:
        """Write an indexed sub-parameter used for controller gains."""
        motor_config = self._require_motor(motor)
        data = struct.pack(">HHl", param, subindex, value)
        self.transmit(motor_config.id, data)
        self.receive(motor_config.id, timeout=0.1)

    def _calculate_erob_pd(
        self,
        desired_kp: float = 0.0,
        desired_kd: float = 0.0,
    ) -> tuple[int, int]:
        """Convert user-space MIT gains into the eRob gain registers."""
        if desired_kd == 0:
            return 0, 0

        # For eRob80 series, the torque constant is 0.134e-3 Nm / mA
        # For eRob70 series, the torque constant is 0.132e-3 Nm / mA
        # here we take the average as approximate
        torque_constant = 0.132e-3  # Nm / mA
        gear_ratio = 50

        # from experiment on robot
        position_gain = 1000 / 37.5e-4
        velocity_gain = 1 / 19.3e-5

        ma_per_count = desired_kd / (torque_constant * MotorConfig.counts_per_rad * gear_ratio)
        kd_erob = ma_per_count * velocity_gain
        kp_erob = (desired_kp * position_gain) / (desired_kd * velocity_gain)
        return round(kp_erob), round(kd_erob)

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float) -> None:
        """Update the internal eRob position/velocity controller gains."""
        can_pid_flag = self.read(motor, Parameter.PID_ADJUSTMENT)
        if can_pid_flag == 1:
            # The vendor parameter table documents 0 as "CAN PID disabled".
            self.write(motor, Parameter.PID_ADJUSTMENT, 0x00)

        kp_int, kd_int = self._calculate_erob_pd(kp, kd)
        ki_int = 0
        self.write_subindex(motor, Parameter.POSITION_LOOP_GAIN, 0x01, kp_int)
        self.write_subindex(motor, Parameter.SPEED_LOOP_GAIN, 0x01, kd_int)
        self.write_subindex(motor, Parameter.SPEED_LOOP_INTEGRAL, 0x01, ki_int)

    def write_mit_control(self, motor: str, position: float, velocity: float = 0, torque: float = 0) -> None:
        """Send a target position and trigger eRob motion execution."""
        position, velocity, torque = self._process_mit_control(motor, position, velocity, torque)
        # The device count frame is centered around pi radians.
        target_counts = int((position + np.pi) * MotorConfig.counts_per_rad)
        self.write(motor, Parameter.TARGET_POSITION, target_counts)
        self.send_command(motor, CommandCode.START_MOTION)

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        """Read measured actuator position and velocity."""
        position_counts = self.read(motor, Parameter.ACTUAL_POSITION)
        if position_counts is None:
            print("WARNING: Failed to read position")
            raise ValueError("Failed to read position")

        position = position_counts / MotorConfig.counts_per_rad - np.pi

        velocity_counts = self.read(motor, Parameter.ACTUAL_SPEED, b"\x00\x01")
        if velocity_counts is None:
            print("WARNING: Failed to read velocity")
            raise ValueError("Failed to read velocity")

        velocity = velocity_counts / MotorConfig.counts_per_rad

        # current = self.read(bus, Parameter.MOTOR_CURRENT)

        position, velocity = self._process_mit_state(motor, position, velocity)
        return position, velocity
