import time

import numpy as np
from loop_rate_limiters import RateLimiter

from actuator_control import Actuator, ERobBus


channel = "can0"
bitrate = 1000000


actuators = {
    "left_wrist_yaw": Actuator(id=15, model="eRob70"),
}


# eRob uses position commands with START_MOTION per cycle; keep loop rate moderate
control_frequency = 50.0


rate = RateLimiter(frequency=control_frequency)
bus = ERobBus(channel=channel, actuators=actuators, bitrate=bitrate)
bus.connect()

for name in actuators:
    bus.enable(name)

try:
    while True:
        target_position = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        bus.write_mit_control(
            actuator="left_wrist_yaw",
            position=target_position,
            velocity=0.0,
            kp=10.0,
            kd=1.0,
            torque=0.0,
        )

        state = bus.get_state(actuator="left_wrist_yaw")
        if state is None:
            rate.sleep()
            continue

        position_0, velocity_0 = state.position, state.velocity
        print(f"position: {position_0:.3f}, velocity: {velocity_0:.3f}")
        rate.sleep()

except KeyboardInterrupt:
    pass

time.sleep(1)
for name in actuators:
    bus.disable(name)
bus.disconnect()

print("Program terminated.")
