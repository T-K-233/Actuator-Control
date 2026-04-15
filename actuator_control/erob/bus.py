"""Python-facing eRob bus wrapper."""

from __future__ import annotations

from typing import Any

from .._rust import _ErobBus
from ..api import Actuator, BusBase, _serialize_actuators, _serialize_calibration
from .protocol import EROB_PROTOCOL


class ERobBus(BusBase):
    protocol = EROB_PROTOCOL

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None = None,
        bitrate: int = 1_000_000,
    ) -> None:
        super().__init__(channel, actuators, calibration, bitrate)
        self._core = _ErobBus(
            channel=channel,
            actuators=_serialize_actuators(actuators),
            calibration=_serialize_calibration(calibration),
            bitrate=bitrate,
        )

    def read(self, actuator: str, parameter: int) -> int:
        return self._core.read(actuator, int(parameter))

    def write(self, actuator: str, parameter: int, value: int) -> None:
        self._core.write(actuator, int(parameter), int(value))
