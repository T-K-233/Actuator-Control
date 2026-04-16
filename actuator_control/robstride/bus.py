"""Python-facing Robstride bus wrapper."""

from __future__ import annotations

from typing import Any

from .._rust import _RobstrideBus
from ..api import Actuator, BusBase, _serialize_actuators, _serialize_calibration
from .protocol import ROBSTRIDE_PROTOCOL


class RobstrideBus(BusBase):
    """High-level Python wrapper for the Robstride CAN backend."""

    protocol = ROBSTRIDE_PROTOCOL

    def __init__(
        self,
        channel: str,
        actuators: dict[str, Actuator],
        calibration: dict[str, dict[str, Any]] | None = None,
        bitrate: int = 1_000_000,
    ) -> None:
        """Create a Robstride bus wrapper.

        Args:
            channel: CAN interface name, e.g. `can0`.
            actuators: Actuators keyed by logical actuator name.
            calibration: Optional per-actuator calibration overrides.
            bitrate: CAN bitrate in bits per second.
        """
        super().__init__(channel, actuators, calibration, bitrate)
        self._core = _RobstrideBus(
            channel=channel,
            actuators=_serialize_actuators(actuators),
            calibration=_serialize_calibration(calibration),
            bitrate=bitrate,
        )

    def read(self, actuator: str, parameter: int) -> int | float:
        """Read one Robstride parameter.

        Args:
            actuator: Actuator name.
            parameter: Robstride parameter ID.

        Returns:
            The decoded integer or floating-point parameter value.
        """
        return self._core.read(actuator, int(parameter))

    def write(self, actuator: str, parameter: int, value: int | float) -> None:
        """Write one Robstride parameter using its declared Python-side type.

        Args:
            actuator: Actuator name.
            parameter: Robstride parameter ID.
            value: Integer or floating-point value matching the parameter type.
        """
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
        """Read cached fault status.

        Args:
            actuator: Optional actuator name. If omitted, returns all cached faults.

        Returns:
            Fault labels for one actuator, or a mapping for every actuator with cached faults.
        """
        return self._core.read_fault_status(actuator)

    def clear_fault(self, actuator: str) -> None:
        """Clear cached and device-side fault state for the specified actuator."""
        self._core.clear_fault(actuator)

    @classmethod
    def ping_by_id(
        cls,
        channel: str,
        device_id: int,
        timeout: float = 0.1,
    ) -> tuple[int, bytes] | None:
        """Probe one Robstride device ID.

        Args:
            channel: CAN interface name.
            device_id: Device ID to probe.
            timeout: Receive timeout in seconds.

        Returns:
            `(extra_data, payload)` when a device replies, otherwise `None`.
        """
        response = _RobstrideBus.ping_by_id(channel, int(device_id), float(timeout))
        if response is None:
            return None

        extra_data, payload = response
        return int(extra_data), bytes(payload)
