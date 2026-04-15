"""Python-facing Sito bus wrapper."""

from __future__ import annotations

from typing import Any

from .._rust import _SitoBus
from ..api import Actuator, BusBase, _serialize_actuators, _serialize_calibration
from .protocol import SITO_PROTOCOL


class SitoBus(BusBase):
    protocol = SITO_PROTOCOL

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None = None,
        bitrate: int = 1_000_000,
        control_frequency: float = 50.0,
    ) -> None:
        super().__init__(channel, actuators, calibration, bitrate)
        self._core = _SitoBus(
            channel=channel,
            actuators=_serialize_actuators(actuators),
            calibration=_serialize_calibration(calibration),
            bitrate=bitrate,
            control_frequency=control_frequency,
        )
