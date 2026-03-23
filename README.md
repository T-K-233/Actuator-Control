# Actuator Control

Python SDK package for controlling robotic actuators over the CAN bus.

## Installation

```bash
uv sync
```

For the plotting utilities in `examples/actuator_characterization`:

```bash
uv sync --extra examples
```

## Quick start

```python
from actuator_control import ERobBus, Motor

motors = {
    "joint": Motor(id=15, model="eRob70"),
}

bus = ERobBus(channel="can0", motors=motors, bitrate=1_000_000)
bus.connect()

try:
    bus.enable("joint")
    bus.write_mit_kp_kd("joint", kp=10.0, kd=1.0)
    bus.write_mit_control("joint", position=0.25)
    position, velocity = bus.read_mit_state("joint")
    print(position, velocity)
finally:
    bus.disconnect()
```

## Motor configuration

Motors are declared as a dictionary keyed by your logical motor names:

```python
from actuator_control import Motor

motors = {
    "left_wrist_roll": Motor(id=0x16, model="TA40-50"),
    "left_wrist_pitch": Motor(id=0x17, model="TA40-50"),
}
```

Motor IDs must be unique within one bus instance.

## Calibration

Optional calibration is applied in the shared MIT helpers before commands are sent and after state is read:

```python
calibration = {
    "left_wrist_roll": {
        "direction": -1,
        "homing_offset": 0.15,
    },
}
```

- `direction` flips the sign of position, velocity, and torque when set to `-1`.
- `homing_offset` shifts the raw actuator zero point in radians.

The calibration dictionary should contain entries for every motor you want transformed.

## Backend notes

### eRob

- eRob currently exposes MIT-like semantics on top of the actuator's position-mode protocol.
- `velocity` and `torque` arguments in `write_mit_control()` are accepted for API compatibility, but only position is used by the backend.
- Controller gains are converted into the vendor-specific position/velocity loop registers.

### Robstride

- Robstride communication is implemented directly in this package using the same private CAN protocol and MIT scaling tables as the public vendor SDK.
- `write_mit_kp_kd()` caches gains locally because the protocol expects them on every MIT command frame.
- Additional Robstride-specific protocol constants are exported as `RobstrideCommunicationType` and `RobstrideParameterType`.

### Sito

- Sito starts a background receiver thread after `connect()` and keeps the latest feedback in memory.
- `read_mit_state()` returns the most recently received state, so it depends on feedback messages arriving from the actuator.
- `control_frequency` controls both command cadence in examples and requested feedback intervals on the bus.
