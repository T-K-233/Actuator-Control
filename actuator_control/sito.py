import queue
import threading
import struct
import time

import numpy as np
import can

from .common import Motor, ActuatorBus


class CommunicationType:
    RESET = 0x00
    SELECT_MODE = 0x01
    SET_MIT_CURRENT_VELOCITY_POSITION = 0x09
    SET_MIT_KP_KD = 0x45
    FEEDBACK_0 = 0xB0
    FEEDBACK_1 = 0xB1
    FEEDBACK_2 = 0xB2
    FEEDBACK_3 = 0xB3


class Mode:
    POSITION = 0x08
    MIT = 0x09


class SitoBus(ActuatorBus):
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
    ):
        super().__init__(channel, motors, calibration, bitrate)

        message_interval_ms = int(1000 / control_frequency)
        assert message_interval_ms > 0, "Message interval must be greater than 0 ms"
        assert message_interval_ms < 0xFF, "Message interval must be less than 255 ms"

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

        self.message_queue: queue.Queue[can.Message] = queue.Queue(maxsize=1024)

        self._killed = threading.Event()
        self._message_receiver_thread = threading.Thread(target=self._run_message_receiver_thread, daemon=True)
        self._message_receiver_thread.start()

    def _run_message_receiver_thread(self):
        while not self._killed.is_set():
            if not self.is_connected:
                time.sleep(0.01)
                continue
            try:
                # print("receiving message")
                frame = self.channel_handler.recv(timeout=0.001)
            except can.CanError as e:
                print(f"Error receiving message: {e} aaaaaa")
                break
            if frame is None:
                continue

            # process feedback message
            if frame.arbitration_id & 0xFFFF0000 == 0x05060000:
                motor_id = frame.arbitration_id >> 8 & 0xFF
                motor_name = next(key for key, value in self.motors.items() if value.id == motor_id)
                message_type = frame.arbitration_id & 0xFF
                config = self.motor_configs[self.motors[motor_name].model]
                match message_type:
                    case CommunicationType.FEEDBACK_1:
                        voltage, pwm, phase_current, velocity_counts = struct.unpack(">hhhh", frame.data)

                        velocity = velocity_counts / config["actuator_counts_per_rad"]
                        torque = phase_current * config["torque_constant"] * config["gear_ratio"]
                        self.motor_status[motor_name]["velocity"] = velocity
                        self.motor_status[motor_name]["torque"] = torque
                    case CommunicationType.FEEDBACK_2:
                        motor_position_couunts, output_position_counts = struct.unpack(">ll", frame.data)

                        position = output_position_counts / config["actuator_counts_per_rad"]
                        self.motor_status[motor_name]["position"] = position
            else:
                print(f"Unknown frame: {frame}")
        print("Message receiver exited.")

    def enable(self, motor: str):
        mode = Mode.MIT
        id = 0x05060000 | (self.motors[motor].id << 8) | CommunicationType.SELECT_MODE
        data = struct.pack(
            ">BBBBL",
            mode,
            self.feedback_1_interval,
            self.feedback_2_interval,
            self.feedback_3_interval,
            0,
        )
        frame = can.Message(
            arbitration_id=id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        self.channel_handler.send(frame)
        print(frame)

    def disable(self, motor: str):
        id = 0x05060000 | (self.motors[motor].id << 8) | CommunicationType.RESET
        data = struct.pack(">LL", 0x55555555, 0x55555555)
        frame = can.Message(
            arbitration_id=id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        self.channel_handler.send(frame)

    def disconnect(self, disable_torque: bool = True) -> None:
        self._killed.set()
        self._message_receiver_thread.join()

        super().disconnect(disable_torque)

    def write_mit_kp_kd(
        self,
        motor: str,
        kp: float,
        kd: float,
    ):
        print(f"SITO update PD on f{motor}: {kp}, {kd}")
        # convert from SI units to sito units
        # kp_sito = kp / 688.58  # these numbers are from single actuator experiment
        # kd_sito = kd / 1.125
        kp_sito = kp / 500  # manually tuned numbers to fit better, not sure why
        kd_sito = kd / 0.5
        id = 0x05060000 | (self.motors[motor].id << 8) | CommunicationType.SET_MIT_KP_KD
        data = struct.pack(">ff", kp_sito, kd_sito)
        frame = can.Message(
            arbitration_id=id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        self.channel_handler.send(frame)

    def write_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float = 0,
        torque: float = 0,
    ):
        position, velocity, torque = self._process_mit_control(motor, position, velocity, torque)

        id = 0x05060000 | (self.motors[motor].id << 8) | CommunicationType.SET_MIT_CURRENT_VELOCITY_POSITION
        config = self.motor_configs[self.motors[motor].model]
        current = torque / config["torque_constant"] / config["gear_ratio"]
        position_counts = config["actuator_counts_per_rad"] * position
        velocity_counts = config["actuator_counts_per_rad"] * velocity

        data = struct.pack(">hhl", int(current), int(velocity_counts), int(position_counts))
        frame = can.Message(
            arbitration_id=id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        self.channel_handler.send(frame)

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        position = self.motor_status[motor]["position"]
        velocity = self.motor_status[motor]["velocity"]
        # torque = self.motor_status[motor]["torque"]
        # temperature = self.motor_status[motor]["temperature"]

        position, velocity = self._process_mit_state(motor, position, velocity)
        return position, velocity
