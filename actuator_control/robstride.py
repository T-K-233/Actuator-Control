"""Robstride actuator backend."""

from __future__ import annotations

import struct
import time
from typing import TypeAlias

import can
import numpy as np
from tqdm import tqdm

from .common import ActuatorBus, Motor, Value
from .robstride_protocol import CommunicationType
from .robstride_table import (
    MODEL_MIT_KD_TABLE,
    MODEL_MIT_KP_TABLE,
    MODEL_MIT_POSITION_TABLE,
    MODEL_MIT_TORQUE_TABLE,
    MODEL_MIT_VELOCITY_TABLE,
)


RobstrideParameter: TypeAlias = tuple[int, np.dtype, str]


class RobstrideBus(ActuatorBus):
    """CAN backend for Robstride actuators."""

    def __init__(
        self,
        channel: str,
        motors: dict[str, Motor],
        calibration: dict[str, dict] | None = None,
        bitrate: int = 1000000,
    ) -> None:
        super().__init__(channel, motors, calibration, bitrate)
        self.host_id = 0xFF
        self._mit_kp: dict[str, float] = {}
        self._mit_kd: dict[str, float] = {}

    @classmethod
    def scan_channel(
        cls,
        channel: str,
        start_id: int = 1,
        end_id: int = 255,
        bitrate: int = 1000000,
    ) -> dict[int, tuple[int, bytes]]:
        """Probe a CAN channel and return all Robstride motors that respond.

        ``end_id`` is inclusive.
        """
        if start_id > end_id:
            raise ValueError("start_id must be less than or equal to end_id.")

        bus = cls(channel=channel, motors={}, bitrate=bitrate)
        bus.connect(handshake=False)
        try:
            device_ids: dict[int, tuple[int, bytes]] = {}
            for device_id in tqdm(range(start_id, end_id + 1), desc="Scanning channel"):
                response = bus.ping_by_id(device_id, timeout=0.1)
                if response is not None:
                    tqdm.write(f"Motor found for device_id={device_id}: {response}")
                    device_ids[device_id] = response
            return device_ids
        finally:
            bus.disconnect(disable_torque=False)

    def _require_model(self, motor: str) -> str:
        model = self._require_motor(motor).model
        if model not in MODEL_MIT_POSITION_TABLE:
            supported = ", ".join(sorted(MODEL_MIT_POSITION_TABLE))
            raise KeyError(
                f"Unsupported Robstride model {model!r} for motor {motor!r}. "
                f"Supported models: {supported}."
            )
        return model

    def connect(self, handshake: bool = True) -> None:
        """Open the CAN interface and discard any queued startup frames."""
        super().connect(handshake=handshake)
        self._drain_rx_buffer(timeout=0.05)

    def _drain_rx_buffer(self, *, timeout: float = 0.0, max_frames: int = 256) -> int:
        """Drain pending frames that may be left over from actuator startup."""
        if not self.is_connected:
            return 0

        bus = self._require_connected()
        drained = 0
        deadline = time.monotonic() + max(timeout, 0.0)

        while drained < max_frames:
            remaining = max(0.0, deadline - time.monotonic())
            frame = bus.recv(timeout=remaining)
            if frame is None:
                break
            drained += 1

        if drained:
            print(f"Drained {drained} pending CAN frame(s) from {self.channel}.")
        return drained

    def _receive_matching_frame(
        self,
        predicate,
        *,
        timeout: float | None,
        description: str,
    ) -> tuple[int, int, int, bytes] | None:
        """Receive frames until one matches the expected response."""
        bus = self._require_connected()
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            if deadline is not None and remaining == 0.0:
                return None

            frame = bus.recv(timeout=remaining)
            if frame is None:
                print("WARNING: Received no response from the motor")
                return None
            if not frame.is_extended_id:
                continue

            communication_type = (frame.arbitration_id >> 24) & 0x1F
            extra_data = (frame.arbitration_id >> 8) & 0xFFFF
            device_id = frame.arbitration_id & 0xFF

            if predicate(communication_type, extra_data, device_id, frame.data):
                return communication_type, extra_data, device_id, frame.data

            print(
                f"Ignoring unrelated Robstride frame on {self.channel} while waiting for {description}: "
                f"comm={communication_type} extra=0x{extra_data:04X} device={device_id} data={frame.data.hex()}"
            )

    def transmit(
        self,
        communication_type: int,
        extra_data: int,
        device_id: int,
        data: bytes = b"\x00\x00\x00\x00\x00\x00\x00\x00",
    ) -> None:
        """Transmit one Robstride protocol frame."""
        if not (0 <= communication_type <= 0x1F):
            raise ValueError("communication_type out of range.")
        if not (0 <= extra_data <= 0xFFFF):
            raise ValueError("extra_data out of range.")
        if not (0 < device_id <= 0xFF):
            raise ValueError("device_id out of range.")
        if len(data) > 8:
            raise ValueError("data length exceeds the CAN payload size of 8 bytes.")

        ext_id = (communication_type << 24) | (extra_data << 8) | device_id
        frame = can.Message(
            arbitration_id=ext_id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        self._require_connected().send(frame)

    def receive(self, timeout: float | None = None) -> tuple[int, int, int, bytes] | None:
        """Receive and unpack one Robstride protocol frame."""
        bus = self._require_connected()
        deadline = None if timeout is None else time.time() + timeout

        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.time())
            if deadline is not None and remaining == 0.0:
                return None

            frame = bus.recv(timeout=remaining)
            if frame is None:
                print("WARNING: Received no response from the motor")
                return None
            if not frame.is_extended_id:
                continue

            communication_type = (frame.arbitration_id >> 24) & 0x1F
            extra_data = (frame.arbitration_id >> 8) & 0xFFFF
            device_id = frame.arbitration_id & 0xFF
            return communication_type, extra_data, device_id, frame.data

    def receive_status_frame(self, motor: str) -> tuple[float, float, float, float]:
        """Receive, validate, and decode a Robstride status frame."""
        motor_id = self._require_motor(motor).id
        received_frame = self._receive_matching_frame(
            lambda communication_type, extra_data, _device_id, _data: (
                communication_type in (
                    CommunicationType.OPERATION_STATUS,
                    CommunicationType.FAULT_REPORT,
                )
                and (extra_data & 0xFF) == motor_id
            ),
            timeout=0.1,
            description=f"status frame for {motor!r}",
        )
        if received_frame is None:
            raise RuntimeError(f"No response from the motor {motor!r}.")

        communication_type, extra_data, _host_id, data = received_frame
        model = self._require_model(motor)

        status_uncalibrated = (extra_data >> 13) & 0x01
        status_stall_overload = (extra_data >> 12) & 0x01
        status_magnetic_encoder_fault = (extra_data >> 11) & 0x01
        status_overtemperature = (extra_data >> 10) & 0x01
        status_gate_driver_fault = (extra_data >> 9) & 0x01
        status_undervoltage = (extra_data >> 8) & 0x01
        device_id = extra_data & 0xFF

        if status_uncalibrated:
            print(f"WARNING: {motor} uncalibrated")
        if status_stall_overload:
            print(f"WARNING: {motor} stall overload fault")
        if status_magnetic_encoder_fault:
            print(f"WARNING: {motor} magnetic encoder fault")
        if status_overtemperature:
            print(f"WARNING: {motor} overtemperature")
        if status_gate_driver_fault:
            print(f"WARNING: {motor} gate driver fault")
        if status_undervoltage:
            print(f"WARNING: {motor} undervoltage")
        if device_id != motor_id:
            print(f"WARNING: Invalid device ID, got {device_id}, expected {motor_id}")

        if communication_type not in (
            CommunicationType.OPERATION_STATUS,
            CommunicationType.FAULT_REPORT,
        ):
            raise RuntimeError(f"Invalid communication type: {communication_type}.")

        if communication_type == CommunicationType.FAULT_REPORT:
            fault_value, warning_value = struct.unpack("<LL", data)
            warning_motor_overtemperature = warning_value & 0x01
            fault_i2t_overload = (fault_value >> 14) & 0x01
            fault_uncalibrated = (fault_value >> 7) & 0x01
            fault_overvoltage = (fault_value >> 3) & 0x01
            fault_undervoltage = (fault_value >> 2) & 0x01
            fault_gate_driver_fault = (fault_value >> 1) & 0x01
            fault_overtemperature = fault_value & 0x01

            if fault_overtemperature:
                print(f"FAULT: {motor} overtemperature")
            if fault_gate_driver_fault:
                print(f"FAULT: {motor} gate driver fault")
            if fault_undervoltage:
                print(f"FAULT: {motor} undervoltage")
            if fault_overvoltage:
                print(f"FAULT: {motor} overvoltage")
            if fault_uncalibrated:
                print(f"FAULT: {motor} encoder uncalibrated")
            if fault_i2t_overload:
                print(f"FAULT: {motor} i2t overload (locked-rotor protection)")
            if warning_motor_overtemperature:
                print(f"WARNING: {motor} overtemperature")
            raise RuntimeError(f"Received fault frame from {motor}: {data!r}")

        if len(data) != 8:
            raise RuntimeError(f"Invalid Robstride status payload length: {len(data)}.")
        position_u16, velocity_u16, torque_i16, temperature_u16 = struct.unpack(">HHHH", data)
        position = (float(position_u16) / 0x7FFF - 1.0) * MODEL_MIT_POSITION_TABLE[model]
        velocity = (float(velocity_u16) / 0x7FFF - 1.0) * MODEL_MIT_VELOCITY_TABLE[model]
        torque = (float(torque_i16) / 0x7FFF - 1.0) * MODEL_MIT_TORQUE_TABLE[model]
        temperature = float(temperature_u16) * 0.1
        return position, velocity, torque, temperature

    def receive_read_frame(self) -> bytes:
        """Receive a parameter-read response payload."""
        received_frame = self.receive(timeout=0.1)
        if received_frame is None:
            raise RuntimeError("No response received for parameter read.")

        communication_type, _extra_data, _host_id, data = received_frame
        if communication_type != CommunicationType.READ_PARAMETER:
            raise RuntimeError(f"Invalid communication type: {communication_type}.")
        if len(data) < 4:
            raise RuntimeError(f"Invalid Robstride read payload length: {len(data)}.")
        return data[4:]

    def ping_by_id(self, device_id: int, timeout: float | None = None) -> tuple[int, bytes] | None:
        """Probe one Robstride device ID and return its identity payload."""
        self.transmit(CommunicationType.GET_DEVICE_ID, self.host_id, device_id)
        response = self.receive(timeout)
        if response is None:
            return None

        communication_type, extra_data, _host_id, data = response
        if communication_type != CommunicationType.GET_DEVICE_ID:
            raise RuntimeError(f"Invalid communication type: {communication_type}.")
        return extra_data, bytes(data)

    def read_id(self, motor: str, timeout: float | None = None) -> tuple[int, bytes] | None:
        """Read the configured device ID and identity payload for one motor."""
        return self.ping_by_id(self._require_motor(motor).id, timeout)

    def write_id(self, motor: str, new_id: int) -> tuple[int, bytes] | None:
        """Assign a new device ID to a Robstride actuator."""
        current_id = self._require_motor(motor).id
        self.transmit(CommunicationType.SET_DEVICE_ID, new_id, current_id)
        response = self.receive(timeout=0.1)
        if response is None:
            return None

        communication_type, extra_data, _host_id, data = response
        if communication_type != CommunicationType.GET_DEVICE_ID:
            raise RuntimeError(f"Invalid communication type: {communication_type}.")

        self.motors[motor].id = new_id
        return extra_data, bytes(data)

    def enable(self, motor: str) -> None:
        """Enable a Robstride actuator."""
        device_id = self._require_motor(motor).id
        self.transmit(CommunicationType.ENABLE, self.host_id, device_id)
        self.receive_status_frame(motor)

    def disable(self, motor: str) -> None:
        """Disable a Robstride actuator."""
        device_id = self._require_motor(motor).id
        self.transmit(CommunicationType.DISABLE, self.host_id, device_id)
        self.receive_status_frame(motor)

    def read(self, motor: str, parameter_type: RobstrideParameter) -> Value:
        """Read a typed Robstride parameter descriptor."""
        device_id = self._require_motor(motor).id
        param_id, param_dtype, param_name = parameter_type

        data = struct.pack("<HHL", param_id, 0x00, 0x00)
        self.transmit(CommunicationType.READ_PARAMETER, self.host_id, device_id, data)
        response = self.receive_read_frame()

        match param_dtype:
            case np.uint8:
                value, _, _ = struct.unpack("<BBH", response)
            case np.int8:
                value, _, _ = struct.unpack("<bBH", response)
            case np.uint16:
                value, _ = struct.unpack("<HH", response)
            case np.int16:
                value, _ = struct.unpack("<hH", response)
            case np.uint32:
                value, = struct.unpack("<L", response)
            case np.int32:
                value, = struct.unpack("<l", response)
            case np.float32:
                value, = struct.unpack("<f", response)
            case _:
                raise ValueError(
                    f"Unsupported parameter type for {param_name}: {param_dtype}."
                )

        return value

    def write(self, motor: str, parameter_type: RobstrideParameter, value: Value) -> None:
        """Write a typed Robstride parameter descriptor."""
        device_id = self._require_motor(motor).id
        param_id, param_dtype, param_name = parameter_type

        match param_dtype:
            case np.uint8:
                value_buffer = struct.pack("<BBH", int(value), 0, 0)
            case np.int8:
                value_buffer = struct.pack("<bBH", int(value), 0, 0)
            case np.uint16:
                value_buffer = struct.pack("<HH", int(value), 0)
            case np.int16:
                value_buffer = struct.pack("<hH", int(value), 0)
            case np.uint32:
                value_buffer = struct.pack("<L", int(value))
            case np.int32:
                value_buffer = struct.pack("<l", int(value))
            case np.float32:
                value_buffer = struct.pack("<f", float(value))
            case _:
                raise ValueError(
                    f"Unsupported parameter type for {param_name}: {param_dtype}."
                )

        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.transmit(CommunicationType.WRITE_PARAMETER, self.host_id, device_id, data)
        self.receive_status_frame(motor)

    def write_operation_frame(
        self,
        motor: str,
        position: float,
        kp: float,
        kd: float,
        velocity: float = 0,
        torque: float = 0,
    ) -> None:
        """Send one Robstride MIT control frame."""
        device_id = self._require_motor(motor).id
        model = self._require_model(motor)

        position, velocity, torque = self._process_mit_control(motor, position, velocity, torque)

        position = np.clip(position, -MODEL_MIT_POSITION_TABLE[model], MODEL_MIT_POSITION_TABLE[model])
        position_u16 = int(((position / MODEL_MIT_POSITION_TABLE[model]) + 1.0) * 0x7FFF)
        position_u16 = int(np.clip(position_u16, 0x0, 0xFFFF))

        velocity = np.clip(velocity, -MODEL_MIT_VELOCITY_TABLE[model], MODEL_MIT_VELOCITY_TABLE[model])
        velocity_u16 = int(((velocity / MODEL_MIT_VELOCITY_TABLE[model]) + 1.0) * 0x7FFF)
        velocity_u16 = int(np.clip(velocity_u16, 0x0, 0xFFFF))

        kp = float(np.clip(kp, 0.0, MODEL_MIT_KP_TABLE[model]))
        kp_u16 = int((kp / MODEL_MIT_KP_TABLE[model]) * 0xFFFF)

        kd = float(np.clip(kd, 0.0, MODEL_MIT_KD_TABLE[model]))
        kd_u16 = int((kd / MODEL_MIT_KD_TABLE[model]) * 0xFFFF)

        torque_u16 = int((torque / MODEL_MIT_TORQUE_TABLE[model] + 1.0) * 0x7FFF)
        torque_u16 = int(np.clip(torque_u16, 0x0, 0xFFFF))

        data = struct.pack(">HHHH", position_u16, velocity_u16, kp_u16, kd_u16)
        self.transmit(CommunicationType.OPERATION_CONTROL, torque_u16, device_id, data)

    def read_operation_frame(self, motor: str) -> tuple[float, float, float, float]:
        """Receive and decode one Robstride MIT status frame."""
        position, velocity, torque, temperature = self.receive_status_frame(motor)
        position, velocity = self._process_mit_state(motor, position, velocity)
        if self.calibration:
            torque *= self.calibration[motor]["direction"]
        return position, velocity, torque, temperature

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float) -> None:
        """Cache MIT gains for the next Robstride control frames."""
        self._require_model(motor)
        self._mit_kp[motor] = kp
        self._mit_kd[motor] = kd

    def write_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float = 0,
        torque: float = 0,
    ) -> None:
        """Send an MIT command using cached gains."""
        self.write_operation_frame(
            motor=motor,
            position=position,
            kp=self._mit_kp.get(motor, 0.0),
            kd=self._mit_kd.get(motor, 0.0),
            velocity=velocity,
            torque=torque,
        )

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        """Read position and velocity from the latest Robstride status frame."""
        position, velocity, _torque, _temperature = self.read_operation_frame(motor)
        return position, velocity
