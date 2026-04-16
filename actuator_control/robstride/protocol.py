"""Robstride protocol constants and metadata."""

from __future__ import annotations

from ..protocol import ActuatorProtocol


class RobstrideCommunicationType:
    """Robstride extended CAN communication type constants."""

    GET_DEVICE_ID = 0
    OPERATION_CONTROL = 1
    OPERATION_STATUS = 2
    ENABLE = 3
    DISABLE = 4
    SET_ZERO_POSITION = 6
    SET_DEVICE_ID = 7
    READ_PARAMETER = 17
    WRITE_PARAMETER = 18
    FAULT_REPORT = 21
    SAVE_PARAMETERS = 22
    SET_BAUDRATE = 23
    ACTIVE_REPORT = 24
    SET_PROTOCOL = 25


class RobstrideParameterType:
    """Robstride parameter ID constants used by `RobstrideBus.read` and `write`."""

    MECHANICAL_OFFSET = 0x2005
    MEASURED_POSITION = 0x3016
    MEASURED_VELOCITY = 0x3017
    MEASURED_TORQUE = 0x302C
    MODE = 0x7005
    IQ_TARGET = 0x7006
    VELOCITY_TARGET = 0x700A
    TORQUE_LIMIT = 0x700B
    CURRENT_KP = 0x7010
    CURRENT_KI = 0x7011
    CURRENT_FILTER_GAIN = 0x7014
    POSITION_TARGET = 0x7016
    VELOCITY_LIMIT = 0x7017
    CURRENT_LIMIT = 0x7018
    MECHANICAL_POSITION = 0x7019
    IQ_FILTERED = 0x701A
    MECHANICAL_VELOCITY = 0x701B
    VBUS = 0x701C
    POSITION_KP = 0x701E
    VELOCITY_KP = 0x701F
    VELOCITY_KI = 0x7020
    VELOCITY_FILTER_GAIN = 0x7021
    VEL_ACCELERATION_TARGET = 0x7022
    PP_VELOCITY_MAX = 0x7024
    PP_ACCELERATION_TARGET = 0x7025
    EPSCAN_TIME = 0x7026
    CAN_TIMEOUT = 0x7028
    ZERO_STATE = 0x7029


ROBSTRIDE_PARAMETER_DATA_TYPES = {
    RobstrideParameterType.MECHANICAL_OFFSET: "float",
    RobstrideParameterType.MEASURED_POSITION: "float",
    RobstrideParameterType.MEASURED_VELOCITY: "float",
    RobstrideParameterType.MEASURED_TORQUE: "float",
    RobstrideParameterType.MODE: "int",
    RobstrideParameterType.IQ_TARGET: "float",
    RobstrideParameterType.VELOCITY_TARGET: "float",
    RobstrideParameterType.TORQUE_LIMIT: "float",
    RobstrideParameterType.CURRENT_KP: "float",
    RobstrideParameterType.CURRENT_KI: "float",
    RobstrideParameterType.CURRENT_FILTER_GAIN: "float",
    RobstrideParameterType.POSITION_TARGET: "float",
    RobstrideParameterType.VELOCITY_LIMIT: "float",
    RobstrideParameterType.CURRENT_LIMIT: "float",
    RobstrideParameterType.MECHANICAL_POSITION: "float",
    RobstrideParameterType.IQ_FILTERED: "float",
    RobstrideParameterType.MECHANICAL_VELOCITY: "float",
    RobstrideParameterType.VBUS: "float",
    RobstrideParameterType.POSITION_KP: "float",
    RobstrideParameterType.VELOCITY_KP: "float",
    RobstrideParameterType.VELOCITY_KI: "float",
    RobstrideParameterType.VELOCITY_FILTER_GAIN: "float",
    RobstrideParameterType.VEL_ACCELERATION_TARGET: "float",
    RobstrideParameterType.PP_VELOCITY_MAX: "float",
    RobstrideParameterType.PP_ACCELERATION_TARGET: "float",
    RobstrideParameterType.EPSCAN_TIME: "int",
    RobstrideParameterType.CAN_TIMEOUT: "int",
    RobstrideParameterType.ZERO_STATE: "int",
}

ROBSTRIDE_PROTOCOL = ActuatorProtocol(
    name="robstride",
    parameter_data_types=ROBSTRIDE_PARAMETER_DATA_TYPES,
)

__all__ = [
    "ROBSTRIDE_PARAMETER_DATA_TYPES",
    "ROBSTRIDE_PROTOCOL",
    "RobstrideCommunicationType",
    "RobstrideParameterType",
]
