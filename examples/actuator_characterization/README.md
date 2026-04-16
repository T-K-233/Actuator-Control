# Actuator Characterization Test

This example shows the procedure to profile an actuator with a torque test stand.

In the characterization test, the actuator under test is commanded with a test signal to move to target positions. The actual position and velocity are measured. Commands are transmitted at **sampling frequency**, while the actual command values are generated at **policy frequency** to simulate an RL policy output.

**Plotting** (`generate_test_signal.py --save-plot` / `--show`, and `plot_data.py`) requires the optional `examples` extra: `pip install actuator-control[examples]`.

## Usage

1. Install dependencies

    ```bash
    uv sync --extra examples
    ```

2. Generate test signal:

   ```bash
   python generate_test_signal.py --config ERobCfg -o data/test_signal.npz [--save-plot] [--show]
   ```

3. Run characterization on hardware (CAN channel, actuator ID, etc. are set in the script):

   ```bash
   python run_erob_characterization.py --signal data/test_signal.npz -o data/characterization_data.npz
   ```

4. Plot results:

   ```bash
   python plot_data.py data/characterization_data.npz [-o data/characterization_plot.png] [--config 0]
   ```

## Experiment parameters

Signals are generated across different amplitudes and frequencies. Supported signal types:

- **Sinusoidal wave** — Identifies frequency response, phase lag, and smooth nonlinearities.
- **Square wave** — Excites friction, deadzones, backlash/hysteresis, and torque-tracking dynamics.
- **Chirp** — Linear frequency sweep (e.g. 1–25 Hz) for broadband frequency response and resonance.
- **Gaussian** — Step-wise constant targets from clipped Gaussian samples; captures nonlinearity under stochastic inputs.

The same signal is run for each **hardware config**: different combinations of joint stiffness (`joint_kp`), joint damping (`joint_kd`), and external torque load (`brake_torque`).


## Data formats

### Signal data (from `generate_test_signal.py`)

Saved as a single NPZ (e.g. `data/test_signal.npz`):

| Key | Description |
|-----|-------------|
| `times` | `(N,)` float32 — Time in seconds at sampling rate. |
| `signal` | `(N,)` float32 — Position command (rad) at sampling rate. |
| `commands` | `(M,)` float32 — Position commands at policy (command) rate; `M = N // (sampling_frequency // policy_frequency)`. |
| `steps` | `(M,)` int64 — Sampling-step indices; `signal[steps] == commands`. |
| `policy_frequency` | int — Command update rate (Hz). |
| `sampling_frequency` | int — Actuator communication rate (Hz). |
| `rest_duration` | float — Rest time (s) between segments. |
| `hardware_configs` | list of dict — e.g. `joint_kp`, `joint_kd`, `brake_torque` per run. |
| `signal_configs` | list of dict — Signal-type configs used to build the trajectory. |

### Experiment data (from `run_erob_characterization.py`)

| Key | Description |
|-----|-------------|
| `policy_frequency` | int — From signal NPZ. |
| `sampling_frequency` | int — From signal NPZ. |
| `hardware_configs` | list of dict — Same as in signal NPZ. |
| `signal_configs` | list of dict — Same as in signal NPZ. |
| `results` | array of object — One dict per hardware config. Each dict contains: `hardware_config`, `kp`, `kd`, `brake_torque`, `times`, `target_positions`, `measured_positions`, `measured_velocities`. |

Plotting scripts expect `times` / `target_positions` / `measured_positions` / `measured_velocities` per result.
