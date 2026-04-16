"""Python-facing Sito bus wrapper."""

from __future__ import annotations

from typing import Any

from .._rust import _SitoBus
from ..api import Actuator, BusBase, _serialize_actuators, _serialize_calibration
from .protocol import SITO_PROTOCOL


class SitoBus(BusBase):
    """High-level Python wrapper for the Sito CAN backend."""

    protocol = SITO_PROTOCOL

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None = None,
        bitrate: int = 1_000_000,
        control_frequency: float = 50.0,
    ) -> None:
        """Create a Sito bus wrapper.

        Args:
            channel: CAN interface name, e.g. `can0`.
            actuators: Actuators keyed by logical actuator name.
            calibration: Optional per-actuator calibration overrides.
            bitrate: CAN bitrate in bits per second.
            control_frequency: Requested control loop frequency in hertz.
        """
        super().__init__(channel, actuators, calibration, bitrate)
        self._core = _SitoBus(
            channel=channel,
            actuators=_serialize_actuators(actuators),
            calibration=_serialize_calibration(calibration),
            bitrate=bitrate,
            control_frequency=control_frequency,
        )
