"""
discover_actuators.py

Scan for motors present on a CAN bus.
"""

import argparse

from actuator_control import RobstrideBus

parser = argparse.ArgumentParser()
parser.add_argument("--channel", "-c", type=str, default="can0", help="CAN channel")
args = parser.parse_args()

ids = RobstrideBus.scan_channel(args.channel, start_id=1, end_id=50)

print(ids)

print("Program terminated.")
