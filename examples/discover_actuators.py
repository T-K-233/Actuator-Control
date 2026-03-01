"""
discover_actuators.py

检测 CAN 总线上存在的电机
"""

import argparse

from robstride_dynamics import RobstrideBus

parser = argparse.ArgumentParser()
parser.add_argument("--channel", "-c", type=str, default="can0", help="CAN 通道")
args = parser.parse_args()

bus = RobstrideBus(channel=args.channel, motors={})
bus.connect()

ids = bus.scan_channel(args.channel, start_id=1, end_id=50)

print(ids)

bus.disconnect()

print("Program terminated.")
