"""Sito protocol constants and metadata."""

from __future__ import annotations

from ..protocol import ActuatorProtocol


class SitoCommunicationType:
    RESET = 0x00
    SELECT_MODE = 0x01
    SET_MIT_CURRENT_VELOCITY_POSITION = 0x09
    SET_MIT_KP_KD = 0x45
    FEEDBACK_0 = 0xB0
    FEEDBACK_1 = 0xB1
    FEEDBACK_2 = 0xB2
    FEEDBACK_3 = 0xB3


class SitoMode:
    POSITION = 0x08
    MIT = 0x09


SITO_PARAMETER_DATA_TYPES: dict[int, str] = {}

SITO_PROTOCOL = ActuatorProtocol(
    name="sito",
    parameter_data_types=SITO_PARAMETER_DATA_TYPES,
)

__all__ = [
    "SITO_PARAMETER_DATA_TYPES",
    "SITO_PROTOCOL",
    "SitoCommunicationType",
    "SitoMode",
]
