# Actuator Control

Actuator Control is a communication library to interface with eRob, Robstride, and Sito actuators over
SocketCAN. The implementation is in Rust, with Python bindings.

The main control model exposed by the library is MIT control. At the API level,
an MIT command is one control frame with five fields:

- `position`: target output position in radians
- `velocity`: target output velocity in radians per second
- `kp`: proportional gain, or stiffness
- `kd`: derivative gain, or damping
- `torque`: feedforward output torque in newton-meters

Those values are sent with `write_mit_control()`. The library keeps that control
surface consistent across backends even when the underlying device protocol does
not use a literal MIT packet on the wire.

## Installation

The package uses `maturin` to provide Python-Rust binding under the hood, but `uv` remains the user-facing
workflow for syncing, building, and running the project.

```bash
uv sync
```

For the plotting utilities in `examples/actuator_characterization`:

```bash
uv sync --extra examples
```

## Runtime model

## MIT control model

`write_mit_control(actuator, position, velocity, kp, kd, torque)` is the main
command interface for closed-loop actuator control.

Conceptually, the commanded output torque is:

```text
tau = kp * (position_target - position) +
      kd * (velocity_target - velocity) +
      torque_feedforward
```

All three backends now use the same receive architecture:

1. Python API calls send command frames on the calling thread
2. a dedicated receive thread continuously reads the bus
3. received frames are dispatched by protocol/message type
4. the receiver updates the local actuator state buffer
5. `get_state()` returns the latest cached state reported by the backend

For eRob, `write_mit_control()` refreshes the cached position and velocity
before returning.

## Examples

```python
from actuator_control import Actuator, ERobBus

actuators = {
    "joint": Actuator(id=15, model="eRob70"),
}

bus = ERobBus(channel="can0", actuators=actuators, bitrate=1_000_000)
bus.connect()

try:
    bus.enable("joint")
    bus.write_mit_control("joint", position=0.25, velocity=0.0, kp=10.0, kd=1.0, torque=0.0)
    state = bus.get_state("joint")
    if state is not None:
        print(state.position, state.velocity)
finally:
    bus.disconnect()
```

More examples can be found in `./examples/` folder.

## Calibration

Optional calibration is passed as a Python dictionary:

```python
calibration = {
    "joint": {
        "direction": -1,
        "homing_offset": 0.15,
    },
}
```

## Backend notes

### eRob

- MIT control is still mapped onto the position-mode protocol.
- `write_mit_control()` performs the explicit register reads needed to refresh
  the local state cache.
- `write_mit_control()` caches the last applied `kp` and `kd` per actuator and
  only rewrites the loop-gain registers when they change.

### Robstride

- MIT gains are packed directly into each operation-control frame.
- Operation-control writes are followed by a device status frame that updates
  the local state cache.

### Sito

- The actuator streams feedback continuously after `enable()`.
- `write_mit_control()` caches the last applied `kp` and `kd` per actuator and
  only sends `SET_MIT_KP_KD` when they change.
- `with_control_frequency()` sets the requested feedback intervals for the
  feedback-1 and feedback-2 streams.
