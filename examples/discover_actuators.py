"""
discover_actuators.py

Scan for actuators present on a CAN bus.
"""

import argparse

from tqdm import tqdm
from actuator_control import RobstrideBus

parser = argparse.ArgumentParser()
parser.add_argument("--channel", "-c", type=str, default="can0", help="CAN channel")
args = parser.parse_args()

device_ids: dict[int, tuple[int, bytes]] = {}

for device_id in tqdm(range(1, 51), desc="Scanning channel"):
    response = RobstrideBus.ping_by_id(args.channel, device_id, timeout=0.1)
    if response is None:
        continue

    print(f"Actuator found for device_id={device_id}")
    device_ids[device_id] = response

print("--------------------------------")
print("Actuators:")
for device_id, response in device_ids.items():
    print(f"{device_id}: {response}")
print("--------------------------------")

print("Program terminated.")
