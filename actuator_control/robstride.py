"""
Robstride actuator bus.

Wraps the Robstride Dynamics Python SDK to implement the ActuatorBus interface,
compatible with erob and sito backends. Uses the MIT control protocol where kp/kd
are sent with each control frame.
"""

from robstride_dynamics import RobstrideBus as SDKRobstrideBus

from .common import ActuatorBus, Motor


class RobstrideBus(ActuatorBus):
    """
    Robstride actuator bus implementing the ActuatorBus interface.

    Uses the Robstride Dynamics SDK for CAN communication. MIT gains (kp, kd)
    are stored when write_mit_kp_kd is called and sent with each write_mit_control frame.
    """

    def __init__(
        self,
        channel: str,
        motors: dict[str, Motor],
        calibration: dict[str, dict] | None = None,
        bitrate: int = 1000000,
    ):
        super().__init__(channel, motors, calibration, bitrate)
        self._sdk_bus = SDKRobstrideBus(
            channel=channel,
            motors=motors,
            calibration=calibration,
            bitrate=bitrate,
        )
        # Store kp/kd per motor; Robstride sends them with every MIT frame
        self._mit_kp: dict[str, float] = {}
        self._mit_kd: dict[str, float] = {}

    def connect(self, handshake: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError(
                f"{self.__class__.__name__}('{self.channel}') is already connected. "
                f"Do not call connect() twice."
            )
        self._sdk_bus.connect(handshake=handshake)
        self.channel_handler = self._sdk_bus.channel_handler

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self.is_connected:
            raise RuntimeError(
                f"{self.__class__.__name__}('{self.channel}') is not connected. "
                "Try running connect() first."
            )
        self._sdk_bus.disconnect(disable_torque=disable_torque)
        self.channel_handler = None

    def enable(self, motor: str) -> None:
        self._sdk_bus.enable(motor)

    def disable(self, motor: str) -> None:
        self._sdk_bus.disable(motor)

    def write_mit_kp_kd(self, motor: str, kp: float, kd: float) -> None:
        """Store kp/kd for this motor; they are sent with each MIT control frame."""
        self._mit_kp[motor] = kp
        self._mit_kd[motor] = kd

    def write_mit_control(
        self,
        motor: str,
        position: float,
        velocity: float = 0,
        torque: float = 0,
    ) -> None:
        kp = self._mit_kp.get(motor, 0.0)
        kd = self._mit_kd.get(motor, 0.0)
        self._sdk_bus.write_operation_frame(
            motor=motor,
            position=position,
            kp=kp,
            kd=kd,
            velocity=velocity,
            torque=torque,
        )

    def read_mit_state(self, motor: str) -> tuple[float, float]:
        position, velocity, _torque, _temperature = self._sdk_bus.read_operation_frame(
            motor
        )
        return position, velocity
