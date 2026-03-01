import time
import struct

import numpy as np
import can

from .common import Value, ActuatorBus


class MotorConfig:
    counts_per_rad: float = 524287 / (2 * np.pi)  # counts/弧度 = 524287 / (2*π)


class CommandCode:
    """命令索引"""
    START_MOTION = 0x83
    STOP_MOTION = 0x84
    SAVE_PARAMS = 0xE8


class Parameter:
    """参数索引"""
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

    # 增益（参考 eRob 参数表）
    POSITION_LOOP_GAIN = 0x64
    SPEED_LOOP_GAIN = 0x66
    SPEED_LOOP_INTEGRAL = 0x67


class Mode:
    """控制模式"""
    TORQUE = 1
    SPEED = 2
    POSITION = 3


class ERobBus(ActuatorBus):
    CLIENT_ID_BASE = 0x640
    SERVER_ID_BASE = 0x5C0

    STATUS_SUCCESS = 0x3E      # 成功状态码

    def receive(self, timeout: float = 0.1) -> bytes | None:
        assert self.channel_handler is not None, "Channel handler not initialized"
        frame = self.channel_handler.recv(timeout=timeout)
        if not frame:
            print("No frame received")
            return None

        # 解析响应
        if len(frame.data) == 1 and frame.data[0] == self.STATUS_SUCCESS:
            # 写入成功确认
            return b''

        if len(frame.data) >= 5 and frame.data[4] == self.STATUS_SUCCESS:
            # 读取数据响应 (4B): [D0, D1, D2, D3, 0x3E]
            return bytes(frame.data[0:4])

        # 说明：实机上出现 3 字节且末尾为 0x3E 的响应，通常表示错误码：
        # [E0, E1, 0x3E]，不是“2 字节数据”。这里直接视为失败。
        if len(frame.data) == 3 and frame.data[2] == self.STATUS_SUCCESS:
            return None

        return None

    def transmit(self, device_id: int, data: bytes = b"\x00\x00\x00\x00\x00\x00\x00\x00") -> None:
        arbitration_id = self.CLIENT_ID_BASE | device_id
        frame = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            data=data,
        )
        assert self.channel_handler is not None, "Channel handler not initialized"
        self.channel_handler.send(frame)

    def send_command(self, motor: str, command: int):
        data = struct.pack(">BB", 0x00, command)
        self.transmit(self.motors[motor].id, data)
        self.receive(timeout=0.1)

    def disable(self, motor: str) -> None:
        self.send_command(motor, CommandCode.STOP_MOTION)
        self.transmit(self.motors[motor].id, struct.pack(">BBL", 0x01, 0x00, 0x00000000))
        self.receive(timeout=0.1)

    def enable(self, motor: str) -> None:
        # self.disable(motor)

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
        time.sleep(0.3)
        self.transmit(self.motors[motor].id, struct.pack(">BBL", 0x01, 0x00, 0x00000001))
        self.receive(timeout=0.1)

    def read(self, motor: str, parameter: int, extra_bytes: bytes = b'') -> Value | None:
        data = struct.pack(">H", parameter) + extra_bytes
        self.transmit(self.motors[motor].id, data)
        result = self.receive(timeout=0.1)

        if result is None:
            return None

        # 大端序解析（兼容 4 字节与 2 字节寄存器）
        if len(result) == 4:
            value, = struct.unpack(">l", result)
            return value

        if len(result) == 2:
            value, = struct.unpack(">h", result)
            return value

        return None

    def write(self, motor: str, param: int, value: int) -> None:
        # 特殊格式参数
        if param == Parameter.CONTROL_MODE or param == Parameter.MOTION_MODE:
            value = value & 0xFF
        data = struct.pack(">Hl", param, value)
        self.transmit(self.motors[motor].id, data)
        self.receive(timeout=0.1)

    def write_subindex(self, motor: str, param: int, subindex: int, value: int) -> None:
        data = struct.pack(">HHl", param, subindex, value)
        self.transmit(self.motors[motor].id, data)
        self.receive(timeout=0.1)

    def _calculate_erob_pd(self, desired_kp: float = 0.0, desired_kd: float = 0.0):
        """
        Calculate the eRob kP and kD values based on the desired kP and kD values.

        Args:
            desired_kp: The desired kP value in Nm/rad.
            desired_kd: The desired kD value in Nm-s/rad.

        Returns:
            kp_erob: The eRob kP value in int.
            kd_erob: The eRob kD value in int.
        """
        # print(f"Desired kP: {desired_kp}, Desired kD: {desired_kd}")

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

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float):
        can_pid_flag = self.read(motor, 0x0124)
        if can_pid_flag == 1:
            # write value = 0 to disable
            self.write(motor, Parameter.PID_ADJUSTMENT, 0x01)

        kp_int, kd_int = self._calculate_erob_pd(kp, kd)
        ki_int = 0
        self.write_subindex(motor, Parameter.POSITION_LOOP_GAIN, 0x01, kp_int)
        self.write_subindex(motor, Parameter.SPEED_LOOP_GAIN, 0x01, kd_int)
        self.write_subindex(motor, Parameter.SPEED_LOOP_INTEGRAL, 0x01, ki_int)

    def write_mit_control(self, motor: str, position: float, velocity: float = 0, torque: float = 0) -> None:
        position, velocity, torque = self._process_mit_control(motor, position, velocity, torque)
        # the count is centered aroud np.pi.
        target_counts = int((position + np.pi) * MotorConfig.counts_per_rad)
        self.write(motor, Parameter.TARGET_POSITION, target_counts)
        self.send_command(motor, CommandCode.START_MOTION)

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        # 读取当前位置/速度/电流（即使在 position 控制下也刷新这些状态，便于监控/保护排查）
        position_counts = self.read(motor, Parameter.ACTUAL_POSITION)
        if position_counts is None:
            print("WARNING: Failed to read position")
            raise ValueError("Failed to read position")

        position = position_counts / MotorConfig.counts_per_rad - np.pi

        velocity_counts = self.read(motor, Parameter.ACTUAL_SPEED, b'\x00\x01')
        if velocity_counts is None:
            print("WARNING: Failed to read velocity")
            raise ValueError("Failed to read velocity")

        velocity = velocity_counts / MotorConfig.counts_per_rad

        # current = self.read(bus, Parameter.MOTOR_CURRENT)

        position, velocity = self._process_mit_state(motor, position, velocity)
        return position, velocity
