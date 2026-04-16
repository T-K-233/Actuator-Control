"""
Run characterization test signal on ERob actuator and log measurements.
"""

import argparse
import time
from pathlib import Path

import numpy as np
from loop_rate_limiters import RateLimiter
import tqdm

from actuator_control import Actuator, ERobBus

channel = "can0"
bitrate = 1_000_000

actuators = {
    "actuator": Actuator(id=15, model="eRob70"),
}


def set_brake_torque(torque_nm: float) -> None:
    """Set brake torque (Nm). No-op here: brake is assumed to be external hardware; torque is stored in logs for metadata."""
    # TODO: implement torque control of the electric brake on the torque test stand
    pass


def run_test(
    bus: ERobBus,
    signal_data: np.ndarray,
    hardware_config: dict,
    rate: RateLimiter,
) -> dict:
    """Run the full signal once with the given hardware config. Returns a result dict."""
    kp = hardware_config["joint_kp"]
    kd = hardware_config["joint_kd"]
    brake_torque = hardware_config["brake_torque"]

    for name in actuators:
        bus.write_mit_kp_kd(name, kp, kd)

    set_brake_torque(brake_torque)

    num_samples = len(signal_data)
    times = np.zeros(num_samples, dtype=np.float32)
    target_positions = np.zeros(num_samples, dtype=np.float32)
    measured_positions = np.zeros(num_samples, dtype=np.float32)
    measured_velocities = np.zeros(num_samples, dtype=np.float32)

    start_time = time.perf_counter()
    for i in tqdm.trange(num_samples):
        target_position = float(signal_data[i])
        for name in actuators:
            bus.write_mit_control(actuator=name, position=target_position)
            bus.request_state(actuator=name)
            state = bus.get_state(actuator=name)
            if state is None:
                raise RuntimeError(f"No cached state available for actuator {name!r}")
            measured_position = state.position
            measured_velocity = state.velocity

        times[i] = time.perf_counter() - start_time
        target_positions[i] = target_position
        measured_positions[i] = measured_position
        measured_velocities[i] = measured_velocity
        rate.sleep()

    # return actuator to rest position
    for name in actuators:
        bus.write_mit_control(actuator=name, position=0)
        bus.request_state(actuator=name)
        state = bus.get_state(actuator=name)
        if state is None:
            raise RuntimeError(f"No cached state available for actuator {name!r}")
        measured_position = state.position
        measured_velocity = state.velocity

    return {
        "hardware_config": hardware_config,
        "kp": kp,
        "kd": kd,
        "brake_torque": brake_torque,
        "times": times,
        "target_positions": target_positions,
        "measured_positions": measured_positions,
        "measured_velocities": measured_velocities,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--signal",
        type=Path,
        default="./data/test_signal.npz",
        help="Path to test signal NPZ (from generate_test_signal.py)",
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default="./data/characterization_data.npz",
        help="Path to save logged data (NPZ)",
    )
    args = parser.parse_args()

    test_signal = np.load(args.signal, allow_pickle=True)
    signal_data = np.asarray(test_signal["signal"]).flatten()
    policy_frequency = int(test_signal["policy_frequency"])
    sampling_frequency = int(test_signal["sampling_frequency"])
    hardware_configs = list(test_signal["hardware_configs"]) if "hardware_configs" in test_signal else []
    signal_configs = list(test_signal["signal_configs"]) if "signal_configs" in test_signal else []
    test_signal.close()

    bus = ERobBus(channel=channel, actuators=actuators, bitrate=bitrate)
    bus.connect()

    try:
        for name in actuators:
            bus.enable(name)

        # return actuator to rest position
        for name in actuators:
            bus.write_mit_kp_kd(name, kp=0.0, kd=1.0)
            bus.write_mit_control(actuator=name, position=0)
            bus.request_state(actuator=name)
            state = bus.get_state(actuator=name)
            if state is None:
                raise RuntimeError(f"No cached state available for actuator {name!r}")

        results: list[dict] = []
        num_samples = len(signal_data)

        print(f"Running characterization for {len(hardware_configs)} hardware config(s)")
        print(f"Signal: {num_samples} steps at {sampling_frequency} Hz (~{num_samples / sampling_frequency:.1f} s)")
        print(f"Policy (command) frequency: {policy_frequency} Hz")
        print("Interrupt with Ctrl+C to stop early.")

        rate = RateLimiter(frequency=sampling_frequency)

        try:
            for index, hardware_config in enumerate(hardware_configs):
                print(f"\n--- Test bundle ({index + 1}/{len(hardware_configs)}): kp={hardware_config['joint_kp']}, kd={hardware_config['joint_kd']}, brake_torque={hardware_config['brake_torque']} Nm ---")

                result = run_test(bus, signal_data, hardware_config, rate)
                results.append(result)
        except KeyboardInterrupt:
            print("Interrupted by user. Stopping early.")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            args.output,
            policy_frequency=policy_frequency,
            sampling_frequency=sampling_frequency,
            hardware_configs=hardware_configs,
            signal_configs=signal_configs,
            results=np.array(results, dtype=object),
        )
        print(f"Saved characterization data to {args.output} ({len(hardware_configs)} x {len(results)} tests)")
    finally:
        print("\nDisabling actuators...")
        for name in actuators:
            bus.disable(name)
        bus.disconnect()

    print("Program terminated.")
