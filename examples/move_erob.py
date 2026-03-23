import time

import numpy as np
from loop_rate_limiters import RateLimiter

from actuator_control import ERobBus, Motor


channel = "can0"
bitrate = 1000000


motors = {
    "left_wrist_yaw": Motor(id=15, model="eRob70"),
}


# eRob uses position commands with START_MOTION per cycle; keep loop rate moderate
control_frequency = 50.0


rate = RateLimiter(frequency=control_frequency)
bus = ERobBus(channel=channel, motors=motors, bitrate=bitrate)
bus.connect()

for name, motor in motors.items():
    bus.enable(name)


bus.write_mit_kp_kd("left_wrist_yaw", kp=10.0, kd=1.0)

try:
    while True:
        # target_position_1 = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        target_position_1 = 0
        bus.write_mit_control(motor="left_wrist_yaw", position=target_position_1)

        position_0, velocity_0 = bus.read_mit_state(motor="left_wrist_yaw")
        print(f"position: {position_0:.3f}, velocity: {velocity_0:.3f}")
        rate.sleep()

except KeyboardInterrupt:
    pass

time.sleep(1)
for name in motors.keys():
    bus.disable(name)
bus.disconnect()

print("Program terminated.")
