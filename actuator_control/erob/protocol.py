"""eRob protocol constants and metadata."""

from __future__ import annotations

from ..protocol import ActuatorProtocol


class ErobCommandType:
    START_MOTION = 0x83
    STOP_MOTION = 0x84
    SAVE_PARAMS = 0xE8


class ErobParameterType:
    ACTUAL_POSITION = 0x0002
    ACTUAL_SPEED = 0x0005
    MOTOR_CURRENT = 0x0008
    ERROR_CODE = 0x001F
    RUN_STATUS = 0x0020
    POWER_TEMP = 0x0026
    CONTROL_MODE = 0x004E
    MAX_POSITION_ERROR = 0x0054
    MAX_SPEED = 0x0055
    POSITION_LOOP_GAIN = 0x0064
    SPEED_LOOP_GAIN = 0x0066
    SPEED_LOOP_INTEGRAL = 0x0067
    TARGET_POSITION = 0x0086
    RELATIVE_POSITION = 0x0087
    ACCELERATION = 0x0088
    DECELERATION = 0x0089
    TARGET_SPEED = 0x008A
    MOTION_MODE = 0x008D
    PID_ADJUSTMENT = 0x0124


class ErobMode:
    TORQUE = 1
    SPEED = 2
    POSITION = 3


EROB_PARAMETER_DATA_TYPES = {
    ErobParameterType.ACTUAL_POSITION: "int",
    ErobParameterType.ACTUAL_SPEED: "int",
    ErobParameterType.MOTOR_CURRENT: "int",
    ErobParameterType.ERROR_CODE: "int",
    ErobParameterType.RUN_STATUS: "int",
    ErobParameterType.POWER_TEMP: "int",
    ErobParameterType.CONTROL_MODE: "int",
    ErobParameterType.MAX_POSITION_ERROR: "int",
    ErobParameterType.MAX_SPEED: "int",
    ErobParameterType.POSITION_LOOP_GAIN: "int",
    ErobParameterType.SPEED_LOOP_GAIN: "int",
    ErobParameterType.SPEED_LOOP_INTEGRAL: "int",
    ErobParameterType.TARGET_POSITION: "int",
    ErobParameterType.RELATIVE_POSITION: "int",
    ErobParameterType.ACCELERATION: "int",
    ErobParameterType.DECELERATION: "int",
    ErobParameterType.TARGET_SPEED: "int",
    ErobParameterType.MOTION_MODE: "int",
    ErobParameterType.PID_ADJUSTMENT: "int",
}

EROB_PROTOCOL = ActuatorProtocol(
    name="erob",
    parameter_data_types=EROB_PARAMETER_DATA_TYPES,
)

__all__ = [
    "EROB_PARAMETER_DATA_TYPES",
    "EROB_PROTOCOL",
    "ErobCommandType",
    "ErobMode",
    "ErobParameterType",
]
