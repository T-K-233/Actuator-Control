"""Sito actuator backend."""

import struct
import threading
import time

import can
import numpy as np

from .common import Motor, ActuatorBus


class CommunicationType:
    """Sito command and feedback identifiers."""

    RESET = 0x00
    SELECT_MODE = 0x01
    SET_MIT_CURRENT_VELOCITY_POSITION = 0x09
    SET_MIT_KP_KD = 0x45
    FEEDBACK_0 = 0xB0
    FEEDBACK_1 = 0xB1
    FEEDBACK_2 = 0xB2
    FEEDBACK_3 = 0xB3


class Mode:
    """Operating modes supported by the Sito protocol."""

    POSITION = 0x08
    MIT = 0x09


class SitoBus(ActuatorBus):
    """CAN backend for Sito TA40-series actuators."""

    motor_configs = {
        "TA40-50": {
            "actuator_counts_per_rad": 65536 / (2 * np.pi),
            "gear_ratio": 51,
            "torque_constant": 0.00009,  # Nm/Arms
        },
        "TA40-100": {
            "actuator_counts_per_rad": 65536 / (2 * np.pi),
            "gear_ratio": 101,
            "torque_constant": 0.00009,  # Nm/Arms
        },
    }

    def __init__(
        self,
        channel: str,
        motors: dict[str, Motor],
        calibration: dict[str, dict] | None = None,
        bitrate: int = 1000000,
        control_frequency: float = 50.0,
    ) -> None:
        super().__init__(channel, motors, calibration, bitrate)

        if control_frequency <= 0:
            raise ValueError("control_frequency must be greater than zero.")
        message_interval_ms = int(1000 / control_frequency)
        if message_interval_ms <= 0:
            raise ValueError("Message interval must be greater than 0 ms.")
        if message_interval_ms >= 0xFF:
            raise ValueError("Message interval must be less than 255 ms.")

        self.feedback_1_interval = message_interval_ms
        self.feedback_2_interval = message_interval_ms
        self.feedback_3_interval = 0

        self.motor_status: dict[str, dict[str, float]] = {}

        for motor in self.motors:
            self.motor_status[motor] = {
                "position": 0,
                "velocity": 0,
                "torque": 0,
                "temperature": 0,
            }

        self._killed = threading.Event()
        self._message_receiver_thread: threading.Thread | None = None

    def connect(self, handshake: bool = True) -> None:
        """Open the CAN channel and start the feedback receiver thread."""
        super().connect(handshake=handshake)
        self._start_receiver_thread()

    def disconnect(self, disable_torque: bool = True) -> None:
        """Stop the feedback thread and close the CAN channel."""
        try:
            super().disconnect(disable_torque=disable_torque)
        finally:
            self._stop_receiver_thread()

    def _start_receiver_thread(self) -> None:
        if self._message_receiver_thread is not None and self._message_receiver_thread.is_alive():
            return
        self._killed.clear()
        self._message_receiver_thread = threading.Thread(
            target=self._run_message_receiver_thread,
            daemon=True,
            name=f"{self.__class__.__name__}-receiver",
        )
        self._message_receiver_thread.start()

    def _stop_receiver_thread(self) -> None:
        self._killed.set()
        if self._message_receiver_thread is not None:
            self._message_receiver_thread.join(timeout=1.0)
            self._message_receiver_thread = None

    def _make_arbitration_id(self, motor: str, communication_type: int) -> int:
        motor_config = self._require_motor(motor)
        return 0x05060000 | (motor_config.id << 8) | communication_type

    def _motor_name_from_id(self, motor_id: int) -> str | None:
        for name, config in self.motors.items():
            if config.id == motor_id:
                return name
        return None

    def _get_motor_config(self, motor: str) -> dict[str, float]:
        motor_config = self._require_motor(motor)
        try:
            return self.motor_configs[motor_config.model]
        except KeyError as exc:
            supported = ", ".join(sorted(self.motor_configs))
            raise KeyError(
                f"Unsupported Sito motor model {motor_config.model!r} for motor {motor!r}. "
                f"Supported models: {supported}."
            ) from exc

    def _run_message_receiver_thread(self) -> None:
        while not self._killed.is_set():
            if not self.is_connected:
                time.sleep(0.01)
                continue

            bus = self.channel_handler
            if bus is None:
                continue
            try:
                frame = bus.recv(timeout=0.001)
            except can.CanError as e:
                if not self._killed.is_set():
                    print(f"Error receiving message: {e}")
                time.sleep(0.01)
                continue
            if frame is None:
                continue

            # process feedback message
            if frame.arbitration_id & 0xFFFF0000 == 0x05060000:
                motor_id = frame.arbitration_id >> 8 & 0xFF
                motor_name = self._motor_name_from_id(motor_id)
                if motor_name is None:
                    continue
                message_type = frame.arbitration_id & 0xFF
                config = self._get_motor_config(motor_name)
                match message_type:
                    case CommunicationType.FEEDBACK_1:
                        voltage, pwm, phase_current, velocity_counts = struct.unpack(">hhhh", frame.data)

                        velocity = velocity_counts / config["actuator_counts_per_rad"]
                        torque = phase_current * config["torque_constant"] * config["gear_ratio"]
                        self.motor_status[motor_name]["velocity"] = velocity
                        self.motor_status[motor_name]["torque"] = torque
                    case CommunicationType.FEEDBACK_2:
                        motor_position_counts, output_position_counts = struct.unpack(">ll", frame.data)

                        position = output_position_counts / config["actuator_counts_per_rad"]
                        self.motor_status[motor_name]["position"] = position
        print("Message receiver exited.")

    def enable(self, motor: str) -> None:
        """Put the actuator into MIT mode and configure feedback cadence."""
        bus = self._require_connected()
        mode = Mode.MIT
        arbitration_id = self._make_arbitration_id(motor, CommunicationType.SELECT_MODE)
        data = struct.pack(
            ">BBBBL",
            mode,
            self.feedback_1_interval,
            self.feedback_2_interval,
            self.feedback_3_interval,
            0,
        )
        frame = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        bus.send(frame)

    def disable(self, motor: str) -> None:
        """Reset the actuator and disable MIT control."""
        bus = self._require_connected()
        arbitration_id = self._make_arbitration_id(motor, CommunicationType.RESET)
        data = struct.pack(">LL", 0x55555555, 0x55555555)
        frame = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        bus.send(frame)

    def write_mit_kp_kd(
        self,
        motor: str,
        kp: float,
        kd: float,
    ) -> None:
        """Set MIT gains for a Sito actuator."""
        bus = self._require_connected()
        # convert from SI units to sito units
        # kp_sito = kp / 688.58  # these numbers are from single actuator experiment
        # kd_sito = kd / 1.125
        kp_sito = kp / 500  # manually tuned numbers to fit better, not sure why
        kd_sito = kd / 0.5
        arbitration_id = self._make_arbitration_id(motor, CommunicationType.SET_MIT_KP_KD)
        data = struct.pack(">ff", kp_sito, kd_sito)
        frame = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        bus.send(frame)

    def write_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float = 0,
        torque: float = 0,
    ) -> None:
        """Send a MIT position/velocity/current command to a Sito actuator."""
        bus = self._require_connected()
        position, velocity, torque = self._process_mit_control(motor, position, velocity, torque)

        arbitration_id = self._make_arbitration_id(motor, CommunicationType.SET_MIT_CURRENT_VELOCITY_POSITION)
        config = self._get_motor_config(motor)
        current = torque / config["torque_constant"] / config["gear_ratio"]
        position_counts = config["actuator_counts_per_rad"] * position
        velocity_counts = config["actuator_counts_per_rad"] * velocity

        data = struct.pack(">hhl", int(current), int(velocity_counts), int(position_counts))
        frame = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        bus.send(frame)

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        """Return the latest cached MIT feedback received from the actuator."""
        self._require_motor(motor)
        position = self.motor_status[motor]["position"]
        velocity = self.motor_status[motor]["velocity"]
        # torque = self.motor_status[motor]["torque"]
        # temperature = self.motor_status[motor]["temperature"]

        position, velocity = self._process_mit_state(motor, position, velocity)
        return position, velocity
