"""
Generate test trajectory signals for actuator characterization.
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


class CharacterizationCfg:
    """Base configuration for characterization tests."""
    seed: int
    policy_frequency: int
    sampling_frequency: int
    rest_duration: float
    hardware_configs: list[dict]
    signal_configs: list[dict]


class ERobCfg(CharacterizationCfg):
    """Configuration for ERob actuator."""

    seed: int = 42
    """ Seed for gaussian signal generation. """

    policy_frequency: int = 50  # Hz
    """ The frequency that position commands are generated. """

    sampling_frequency: int = 50  # Hz
    """ The frequency of the actuator communication. """

    rest_duration: float = 1.0  # s
    """ Rest between test segments and padding before/after each excitation (converted to steps via sampling_frequency). """

    hardware_configs: list[dict] = [
        {
            "brake_torque": 0.0,  # Nm
            "joint_kp": 10.0,  # Nm/rad
            "joint_kd": 1.0,  # Nm-s/rad
        },
        {
            "brake_torque": 10.0,  # Nm
            "joint_kp": 20.0,  # Nm/rad
            "joint_kd": 2.0,  # Nm-s/rad
        },
    ]

    signal_configs: list[dict] = [
        {
            "signal": "sine",
            "duration": 8.0,  # s
            "frequencies": [0.25, 0.5, 1.0, 2.0, 5.0],  # Hz
            "amplitudes": [0.3142, 0.7854, 1.5708, 3.1416],  # [10%, 25%, 50%, 100%] of ±3.1416 rad
        },
        {
            "signal": "square",
            "duration": 8.0,  # s
            "frequencies": [0.25, 0.5, 1.0, 2.0, 5.0],  # Hz
            "amplitudes": [0.3142, 0.7854, 1.5708, 3.1416],  # [10%, 25%, 50%, 100%] of ±3.1416 rad
        },
        {
            "signal": "chirp",
            "duration": 16.0,  # s
            "start_frequency": 1.0,  # Hz
            "end_frequency": 25.0,  # Hz
            "amplitudes": [0.7854, 3.1416],  # [25%, 100%] of ±3.1416 rad
        },
        {
            "signal": "gaussian",
            "duration": 300.0,  # s
            "amplitude": 3.1416,  # rad
            "min_step_size": 1,
            "max_step_size": 20,
        },
    ]


class SanityCheckCfg(ERobCfg):
    """Configuration for sanity check tests."""

    signal_configs: list[dict] = [
        {
            "signal": "sine",
            "duration": 8.0,  # s
            "frequencies": [0.25, 2.0, 5.0],  # Hz
            "amplitudes": [0.3142],  # [10%, 25%, 50%, 100%] of ±3.1416 rad
        },
        {
            "signal": "square",
            "duration": 8.0,  # s
            "frequencies": [0.25, 2.0, 5.0],  # Hz
            "amplitudes": [0.3142],  # [10%, 25%, 50%, 100%] of ±3.1416 rad
        },
        {
            "signal": "chirp",
            "duration": 16.0,  # s
            "start_frequency": 1.0,  # Hz
            "end_frequency": 25.0,  # Hz
            "amplitudes": [0.3142],  # [25%, 100%] of ±3.1416 rad
        },
        {
            "signal": "gaussian",
            "duration": 10.0,  # s
            "amplitude": 0.3142,  # rad
            "min_step_size": 1,
            "max_step_size": 20,
        },
    ]


class CharacterizationTest:
    """
    Generated test trajectory from a config.

    Attributes:
        times: (N,) per-sampling time in seconds; N = number of sampling steps.
        signal: (N,) per-sampling position command in rad; same length as times.
        commands: (M,) subsampled position commands at policy (command) frequency, for sending to actuators; M = N // (sampling_frequency // policy_frequency).
        steps: (M,) sampling-step indices; steps[i] is the index into times/signal where commands[i] applies.
    """

    def __init__(
        self,
        cfg: CharacterizationCfg,
        signal_offset: float = 0.0,
    ):
        self.cfg = cfg
        self.signal_offset = signal_offset

        assert cfg.sampling_frequency % cfg.policy_frequency == 0, "sampling_frequency must be a multiple of policy_frequency"

        np.random.seed(cfg.seed)

        # Populated by generate_signal(): times/signal at sampling rate; commands/steps at command rate.
        self.times = np.array([], dtype=np.float32)
        self.signal = np.array([], dtype=np.float32)
        self.steps = np.array([], dtype=np.int64)
        self.commands = np.array([], dtype=np.float32)

        self.generate_signal()

    @property
    def duration(self) -> float:
        """Duration of the test trajectory in seconds."""
        return len(self.times) / self.cfg.sampling_frequency

    @staticmethod
    def generate_sine_signal(times: np.ndarray, frequency: float, amplitude: float, offset: float) -> np.ndarray:
        return amplitude * np.sin(2 * np.pi * frequency * times) + offset

    @staticmethod
    def generate_square_signal(times: np.ndarray, frequency: float, amplitude: float, offset: float) -> np.ndarray:
        return amplitude * np.sign(np.sin(2 * np.pi * frequency * times)) + offset

    @staticmethod
    def generate_chirp_signal(
        times: np.ndarray,
        start_frequency: float,
        end_frequency: float,
        amplitude: float,
        offset: float,
    ) -> np.ndarray:
        duration = times[-1] - times[0]
        k = (end_frequency - start_frequency) / duration
        phase = 2 * np.pi * (start_frequency * times + 0.5 * k * times**2)
        return amplitude * np.sin(phase) + offset

    @staticmethod
    def generate_gaussian_signal(
        times: np.ndarray,
        amplitude: float,
        offset: float,
        step_min: int,
        step_max: int,
    ) -> np.ndarray:
        """Step-wise constant targets from clipped Gaussian samples."""
        trajectory = np.zeros_like(times)
        time_index = 0
        while time_index < times.shape[0]:
            random_step_size = np.random.randint(step_min, step_max)
            random_target = amplitude * np.clip(np.random.normal(0, amplitude / 2), -1, 1) + offset
            trajectory[time_index:time_index + random_step_size] = random_target
            time_index += random_step_size
        return trajectory

    def generate_signal(self) -> None:
        """
        Build test trajectory at sampling rate, then derive command-rate arrays.

        Sets:
            times: (N,) per-sampling time in seconds.
            signal: (N,) per-sampling position command in rad.
            commands: (M,) subsampled at policy_frequency for actuators; M < N.
            steps: (M,) sampling-step indices s.t. signal[steps] == commands.
        """
        cfg = self.cfg
        offset = self.signal_offset
        policy_frequency = cfg.policy_frequency
        sampling_frequency = cfg.sampling_frequency
        rest_duration_s = cfg.rest_duration

        dt = 1.0 / sampling_frequency
        repeat = sampling_frequency // policy_frequency
        rest_samples = int(rest_duration_s * sampling_frequency)
        blocks: list[np.ndarray] = []

        for test in cfg.signal_configs:
            match test["signal"]:
                case "sine":
                    for freq in test["frequencies"]:
                        for amp in test["amplitudes"]:
                            policy_times = np.linspace(0, test["duration"], int(policy_frequency * test["duration"]), endpoint=False)
                            sig = self.generate_sine_signal(policy_times, freq, amp, offset)
                            sig_sampled = np.repeat(sig, repeat)
                            block = np.concatenate([
                                sig_sampled,
                                np.full(rest_samples, offset),
                            ])
                            blocks.append(block)
                case "square":
                    for freq in test["frequencies"]:
                        for amp in test["amplitudes"]:
                            policy_times = np.linspace(0, test["duration"], int(policy_frequency * test["duration"]), endpoint=False)
                            sig = self.generate_square_signal(policy_times, freq, amp, offset)
                            sig_sampled = np.repeat(sig, repeat)
                            block = np.concatenate([
                                sig_sampled,
                                np.full(rest_samples, offset),
                            ])
                            blocks.append(block)
                case "chirp":
                    policy_times = np.linspace(0, test["duration"], int(policy_frequency * test["duration"]), endpoint=False)
                    amps = test.get("amplitudes", [test["amplitude"]]) if "amplitude" in test else test["amplitudes"]
                    for amp in amps:
                        sig = self.generate_chirp_signal(
                            policy_times,
                            test["start_frequency"],
                            test["end_frequency"],
                            amp,
                            offset,
                        )
                        sig_sampled = np.repeat(sig, repeat)
                        block = np.concatenate([
                            sig_sampled,
                            np.full(rest_samples, offset),
                        ])
                        blocks.append(block)
                case "gaussian":
                    policy_times = np.linspace(0, test["duration"], int(policy_frequency * test["duration"]), endpoint=False)
                    amp = test["amplitude"]
                    step_min = test.get("min_step_size", test.get("step_min", 1))
                    step_max = test.get("max_step_size", test.get("step_max", 20))
                    sig = self.generate_gaussian_signal(policy_times, amp, offset, step_min, step_max)
                    sig_sampled = np.repeat(sig, repeat)
                    block = np.concatenate([
                        sig_sampled,
                        np.full(rest_samples, offset),
                    ])
                    blocks.append(block)
                case _:
                    raise ValueError(f"Unknown signal type: {test['signal']}")

        signal = np.concatenate(blocks)

        times = np.arange(signal.shape[0], dtype=np.float32) * dt
        self.times = times.astype(np.float32)
        self.signal = signal.astype(np.float32)

        # Command-rate: subsample every `repeat` sampling steps; steps[i] = sampling index, commands[i] = signal at that index.
        step_indices = np.arange(0, signal.shape[0], repeat, dtype=np.int64)
        self.steps = step_indices
        self.commands = signal[step_indices].astype(np.float32)

    def _command_at_sampling_rate(self) -> np.ndarray:
        """Expand commands to same length as signal (same X as times/signal). Each command spans from steps[k] to steps[k+1]."""
        N = len(self.signal)
        M = len(self.commands)
        expanded = np.empty(N, dtype=np.float32)
        for k in range(M):
            start = self.steps[k]
            end = self.steps[k + 1] if k + 1 < M else N
            expanded[start:end] = self.commands[k]
        return expanded

    def save(self, path: Path | str) -> Path:
        """Save times, signal, commands, and steps to NPZ. Creates parent dirs. Returns path."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,

            # configurations
            policy_frequency=self.cfg.policy_frequency,
            sampling_frequency=self.cfg.sampling_frequency,
            rest_duration=self.cfg.rest_duration,
            hardware_configs=self.cfg.hardware_configs,
            signal_configs=self.cfg.signal_configs,

            # data
            times=self.times,
            signal=self.signal,
            steps=self.steps,
            commands=self.commands,
        )
        return path

    def plot(self) -> None:
        """Show matplotlib figure: per-sampling signal and command (command extended to same X as signal)."""
        command_at_sampling = self._command_at_sampling_rate()
        fig, ax = plt.subplots(1, 1, figsize=(10, 4))
        ax.plot(self.times, self.signal, label="signal", color="tab:blue")
        ax.plot(
            self.times,
            command_at_sampling,
            label="command",
            color="k",
            linestyle="--",
            drawstyle="steps-pre",
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title("Test signal")
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        plt.show()

    def save_plot(self, path: Path | str, dpi: int = 150) -> Path:
        """Save plot of signal and command (command extended to same X) to file. Creates parent dirs. Returns path."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        command_at_sampling = self._command_at_sampling_rate()
        fig, ax = plt.subplots(1, 1, figsize=(10, 4))
        ax.plot(self.times, self.signal, label="signal", color="tab:blue")
        ax.plot(
            self.times,
            command_at_sampling,
            label="command",
            color="k",
            linestyle="--",
            drawstyle="steps-pre",
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.set_title("Test signal")
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        return path


def _all_config_classes(base: type) -> list[type]:
    """Return base and all descendant classes (recursive)."""
    out = [base]
    for sub in base.__subclasses__():
        out.extend(_all_config_classes(sub))
    return out


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    config_classes = _all_config_classes(CharacterizationCfg)
    config_choices = [c.__name__ for c in config_classes]
    parser.add_argument(
        "--config", type=str, default="ERobCfg",
        choices=config_choices,
        help="config class to use",
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default="./data/test_signal.npz",
        help="path to store test signal",
    )
    parser.add_argument("--show", action="store_true", help="show matplotlib plot")
    parser.add_argument("--save-plot", action="store_true", help="save plot to file")
    args = parser.parse_args()

    config_cls = globals()[args.config]
    cfg = config_cls()

    test = CharacterizationTest(cfg)

    test.save(args.output)

    print(f"Saved signal to {args.output} (length={len(test.signal)}, duration={test.duration:.2f} s)")
    print("Hardware configs:")
    for hw in cfg.hardware_configs:
        print(f"  - kp={hw['joint_kp']} Nm/rad, kd={hw['joint_kd']} Nm-s/rad, brake_torque={hw['brake_torque']} Nm")

    print("Signal configs:")
    for sig in cfg.signal_configs:
        parts = [f"signal={sig['signal']}", f"duration={sig['duration']} s"]
        if "frequencies" in sig:
            parts.append(f"frequencies={sig['frequencies']} Hz")
        if "amplitudes" in sig:
            parts.append(f"amplitudes={sig['amplitudes']} rad")
        if "start_frequency" in sig:
            parts.append(f"start_freq={sig['start_frequency']} Hz")
        if "end_frequency" in sig:
            parts.append(f"end_freq={sig['end_frequency']} Hz")
        if "amplitude" in sig:
            parts.append(f"amplitude={sig['amplitude']} rad")
        if "min_step_size" in sig:
            parts.append(f"min_step={sig['min_step_size']}, max_step={sig['max_step_size']}")
        print(f"  - {', '.join(parts)}")

    if args.save_plot:
        test.save_plot(args.output.with_suffix(".png"))
        print(f"Saved plot to {args.output.with_suffix('.png')}")
    if args.show:
        test.plot()
