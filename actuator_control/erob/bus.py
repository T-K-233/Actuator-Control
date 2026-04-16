"""Python-facing eRob bus wrapper."""

from __future__ import annotations

from typing import Any

from .._rust import _ErobBus
from ..api import Actuator, BusBase, _serialize_actuators, _serialize_calibration
from .protocol import EROB_PROTOCOL


class ERobBus(BusBase):
    """High-level Python wrapper for the eRob CAN backend."""

    protocol = EROB_PROTOCOL

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None = None,
        bitrate: int = 1_000_000,
    ) -> None:
        """Create an eRob bus wrapper.

        Args:
            channel: CAN interface name, e.g. `can0`.
            actuators: Actuators keyed by actuator name.
            calibration: Optional per-actuator calibration overrides.
            bitrate: CAN bitrate in bits per second.
        """
        super().__init__(channel, actuators, calibration, bitrate)
        self._core = _ErobBus(
            channel=channel,
            actuators=_serialize_actuators(actuators),
            calibration=_serialize_calibration(calibration),
            bitrate=bitrate,
        )

    def read(self, actuator: str, parameter: int) -> int:
        """Read one eRob parameter as an integer value."""
        return self._core.read(actuator, int(parameter))

    def write(self, actuator: str, parameter: int, value: int) -> None:
        """Write one integer eRob parameter."""
        self._core.write(actuator, int(parameter), int(value))
