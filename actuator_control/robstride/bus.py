"""Python-facing Robstride bus wrapper."""

from __future__ import annotations

from typing import Any

from .._rust import _RobstrideBus
from ..api import Actuator, BusBase, _serialize_actuators, _serialize_calibration
from .protocol import ROBSTRIDE_PROTOCOL


class RobstrideBus(BusBase):
    protocol = ROBSTRIDE_PROTOCOL

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None = None,
        bitrate: int = 1_000_000,
    ) -> None:
        super().__init__(channel, actuators, calibration, bitrate)
        self._core = _RobstrideBus(
            channel=channel,
            actuators=_serialize_actuators(actuators),
            calibration=_serialize_calibration(calibration),
            bitrate=bitrate,
        )

    def read(self, actuator: str, parameter: int) -> int | float:
        return self._core.read(actuator, int(parameter))

    def write(self, actuator: str, parameter: int, value: int | float) -> None:
        parameter_id = int(parameter)
        data_type = self.protocol.parameter_data_types.get(parameter_id)
        if data_type is None:
            raise ValueError(f"Unknown {self.protocol.name} parameter id 0x{parameter_id:04X}")

        if data_type == "float":
            self._core.write_float(actuator, parameter_id, float(value))
        else:
            self._core.write_integer(actuator, parameter_id, int(value))

    def read_fault_status(
        self,
        actuator: str | None = None,
    ) -> dict[str, list[str]] | list[str]:
        return self._core.read_fault_status(actuator)

    def clear_fault(self, actuator: str) -> None:
        self._core.clear_fault(actuator)

    @classmethod
    def ping_by_id(
        cls,
        channel: str,
        device_id: int,
        timeout: float = 0.1,
    ) -> tuple[int, bytes] | None:
        response = _RobstrideBus.ping_by_id(channel, int(device_id), float(timeout))
        if response is None:
            return None

        extra_data, payload = response
        return int(extra_data), bytes(payload)
