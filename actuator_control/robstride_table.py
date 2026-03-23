"""MIT scaling tables for Robstride actuators.

These values are adapted from the public Robstride Dynamics Python SDK
(`ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK`, MIT licensed).
"""

import numpy as np


MODEL_MIT_POSITION_TABLE = {
    "rs-00": 4 * np.pi,
    "rs-01": 4 * np.pi,
    "rs-02": 4 * np.pi,
    "rs-03": 4 * np.pi,
    "rs-04": 4 * np.pi,
    "rs-05": 4 * np.pi,
    "rs-06": 4 * np.pi,
}
"""Position scaling range for MIT frames, in rad."""


MODEL_MIT_VELOCITY_TABLE = {
    "rs-00": 50,
    "rs-01": 44,
    "rs-02": 44,
    "rs-03": 50,
    "rs-04": 15,
    "rs-05": 33,
    "rs-06": 20,
}
"""Velocity scaling range for MIT frames, in rad/s."""


MODEL_MIT_TORQUE_TABLE = {
    "rs-00": 17,
    "rs-01": 17,
    "rs-02": 17,
    "rs-03": 60,
    "rs-04": 120,
    "rs-05": 17,
    "rs-06": 60,
}
"""Torque scaling range for MIT frames, in Nm."""


MODEL_MIT_KP_TABLE = {
    "rs-00": 500.0,
    "rs-01": 500.0,
    "rs-02": 500.0,
    "rs-03": 5000.0,
    "rs-04": 5000.0,
    "rs-05": 500.0,
    "rs-06": 5000.0,
}
"""Kp scaling range for MIT frames, in Nm/rad."""


MODEL_MIT_KD_TABLE = {
    "rs-00": 5.0,
    "rs-01": 5.0,
    "rs-02": 5.0,
    "rs-03": 100.0,
    "rs-04": 100.0,
    "rs-05": 5.0,
    "rs-06": 100.0,
}
"""Kd scaling range for MIT frames, in Nm-s/rad."""
