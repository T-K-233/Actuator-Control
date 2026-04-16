# Actuator Control

Actuator Control is a communication library to interface with eRob, Robstride, and Sito actuators over
SocketCAN. The implementation is in Rust, with Python bindings.

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

All three backends now use the same receive architecture:

1. Python API calls send command frames on the calling thread
2. a dedicated receive thread continuously reads the bus
3. received frames are dispatched by protocol/message type
4. the receiver updates the local actuator state buffer
5. `get_state()` returns the latest cached state

For backends without continuous actuator feedback, call `request_state()`
before `get_state()` when you need a fresh sample.

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
    bus.write_mit_kp_kd("joint", kp=10.0, kd=1.0)
    bus.write_mit_control("joint", position=0.25)
    bus.request_state("joint")
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
- `request_state()` performs the explicit register reads needed to refresh
  the local state cache.
- `write_mit_kp_kd()` converts MIT gains into the actuator position/velocity
  loop registers.

### Robstride

- MIT gains are cached locally and packed into each operation-control frame.

### Sito

- the actuator streams feedback continuously after `enable()`.
- `with_control_frequency()` sets the requested feedback intervals for the
  feedback-1 and feedback-2 streams.
