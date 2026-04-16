"""Shared protocol metadata helpers."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ActuatorProtocol:
    """Static protocol metadata for one actuator family.

    Attributes:
        name: Stable protocol name used in the Python API.
        parameter_data_types: Mapping from parameter ID to ``"int"`` or ``"float"``.
    """

    name: str
    parameter_data_types: dict[int, str]


class _ProtocolRegistry(dict[str, ActuatorProtocol]):
    """Lazy registry that avoids import cycles during package initialization."""

    def __init__(self) -> None:
        super().__init__()
        self._loaded = False

    def _ensure_loaded(self) -> None:
        if self._loaded:
            return

        from .erob.protocol import EROB_PROTOCOL
        from .robstride.protocol import ROBSTRIDE_PROTOCOL
        from .sito.protocol import SITO_PROTOCOL

        self.update(
            {
                EROB_PROTOCOL.name: EROB_PROTOCOL,
                ROBSTRIDE_PROTOCOL.name: ROBSTRIDE_PROTOCOL,
                SITO_PROTOCOL.name: SITO_PROTOCOL,
            }
        )
        self._loaded = True

    def __iter__(self) -> Iterator[str]:
        self._ensure_loaded()
        return super().__iter__()

    def __len__(self) -> int:
        self._ensure_loaded()
        return super().__len__()

    def __getitem__(self, key: str) -> ActuatorProtocol:
        self._ensure_loaded()
        return super().__getitem__(key)

    def get(self, key: str, default: ActuatorProtocol | None = None) -> ActuatorProtocol | None:
        self._ensure_loaded()
        return super().get(key, default)


ACTUATOR_PROTOCOLS = _ProtocolRegistry()


def get_parameter_data_type(actuator_type: str, parameter: int) -> str | None:
    """Return the declared Python-side type for a protocol parameter.

    Args:
        actuator_type: Protocol name such as ``"robstride"``.
        parameter: Backend parameter ID.

    Returns:
        ``"int"`` or ``"float"`` when the parameter is known, otherwise ``None``.
    """

    protocol = ACTUATOR_PROTOCOLS.get(actuator_type)
    if protocol is None:
        raise ValueError(f"Unknown actuator protocol '{actuator_type}'")
    return protocol.parameter_data_types.get(int(parameter))


__all__ = [
    "ActuatorProtocol",
    "ACTUATOR_PROTOCOLS",
    "get_parameter_data_type",
]
