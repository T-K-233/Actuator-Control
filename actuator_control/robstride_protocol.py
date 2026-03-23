"""Protocol constants for Robstride actuators.

These values are adapted from the public Robstride Dynamics Python SDK
(`ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK`, MIT licensed).
"""

import numpy as np


class CommunicationType:
    """Private communication types used by the Robstride CAN protocol."""

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


class ParameterType:
    """Typed Robstride parameter descriptors.

    Each value is ``(parameter_id, numpy_dtype, parameter_name)``.
    """

    MECHANICAL_OFFSET = (0x2005, np.float32, "mechOffset")
    MEASURED_POSITION = (0x3016, np.float32, "mechPos")
    MEASURED_VELOCITY = (0x3017, np.float32, "mechVel")
    MEASURED_TORQUE = (0x302C, np.float32, "torque_fdb")
    MODE = (0x7005, np.int8, "run_mode")
    IQ_TARGET = (0x7006, np.float32, "iq_ref")
    VELOCITY_TARGET = (0x700A, np.float32, "spd_ref")
    TORQUE_LIMIT = (0x700B, np.float32, "limit_torque")
    CURRENT_KP = (0x7010, np.float32, "cur_kp")
    CURRENT_KI = (0x7011, np.float32, "cur_ki")
    CURRENT_FILTER_GAIN = (0x7014, np.float32, "cur_filter_gain")
    POSITION_TARGET = (0x7016, np.float32, "loc_ref")
    VELOCITY_LIMIT = (0x7017, np.float32, "limit_spd")
    CURRENT_LIMIT = (0x7018, np.float32, "limit_cur")
    MECHANICAL_POSITION = (0x7019, np.float32, "mechPos")
    IQ_FILTERED = (0x701A, np.float32, "iqf")
    MECHANICAL_VELOCITY = (0x701B, np.float32, "mechVel")
    VBUS = (0x701C, np.float32, "VBUS")
    POSITION_KP = (0x701E, np.float32, "loc_kp")
    VELOCITY_KP = (0x701F, np.float32, "spd_kp")
    VELOCITY_KI = (0x7020, np.float32, "spd_ki")
    VELOCITY_FILTER_GAIN = (0x7021, np.float32, "spd_filter_gain")
    VEL_ACCELERATION_TARGET = (0x7022, np.float32, "acc_rad")
    PP_VELOCITY_MAX = (0x7024, np.float32, "vel_max")
    PP_ACCELERATION_TARGET = (0x7025, np.float32, "acc_set")
    EPSCAN_TIME = (0x7026, np.uint16, "EPScan_time")
    CAN_TIMEOUT = (0x7028, np.uint32, "canTimeout")
    ZERO_STATE = (0x7029, np.uint8, "zero_sta")
