import time

import numpy as np
from loop_rate_limiters import RateLimiter

from actuator_control import Actuator, SitoBus


channel = "can2"
bitrate = 1000000


actuators = {
    "left_wrist_roll": Actuator(id=0x16, model="TA40-50"),
    "left_wrist_pitch": Actuator(id=0x17, model="TA40-50"),
}


# can go up to 1 kHz with 2 actuators
control_frequency = 50.0


rate = RateLimiter(frequency=control_frequency)
bus = SitoBus(channel=channel, actuators=actuators, control_frequency=control_frequency)
bus.connect()

for name in actuators:
    bus.enable(name)


try:
    while True:
        target_position_1 = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        target_position_2 = np.cos(2 * np.pi * 1 * time.time()) * 0.2
        bus.write_mit_control(
            actuator="left_wrist_roll",
            position=target_position_1,
            velocity=0.0,
            kp=20.0,
            kd=1.0,
            torque=0.0,
        )
        bus.write_mit_control(
            actuator="left_wrist_pitch",
            position=target_position_2,
            velocity=0.0,
            kp=20.0,
            kd=1.0,
            torque=0.0,
        )

        state_0 = bus.get_state(actuator="left_wrist_roll")
        state_1 = bus.get_state(actuator="left_wrist_pitch")
        if state_0 is None or state_1 is None:
            rate.sleep()
            continue

        position_0, velocity_0 = state_0.position, state_0.velocity
        position_1, velocity_1 = state_1.position, state_1.velocity
        print(f"position: {position_0:.3f}, {position_1:.3f}, velocity: {velocity_0:.3f}, {velocity_1:.3f}")
        rate.sleep()

except KeyboardInterrupt:
    pass

for name in actuators:
    bus.disable(name)
bus.disconnect()

print("Program terminated.")
