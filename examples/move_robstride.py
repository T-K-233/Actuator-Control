import time

import numpy as np
from loop_rate_limiters import RateLimiter

from actuator_control import Actuator, RobstrideBus


channel = "can1"
bitrate = 1000000


actuators = {
    "actuator_1": Actuator(id=11, model="rs-02"),
    "actuator_2": Actuator(id=12, model="rs-00"),
}


control_frequency = 50.0


rate = RateLimiter(frequency=control_frequency)
bus = RobstrideBus(channel=channel, actuators=actuators, bitrate=bitrate)
bus.connect()

bus.write_mit_kp_kd("actuator_1", 0.1, 1.0)
bus.write_mit_kp_kd("actuator_2", 0.1, 1.0)

for name in actuators:
    bus.enable(name)


try:
    while True:
        target_position_1 = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        target_position_2 = np.cos(2 * np.pi * 1 * time.time()) * 0.2
        bus.write_mit_control(actuator="actuator_1", position=target_position_1, velocity=0, torque=0)
        bus.write_mit_control(actuator="actuator_2", position=target_position_2, velocity=0, torque=0)

        state_0 = bus.get_state(actuator="actuator_1")
        state_1 = bus.get_state(actuator="actuator_2")
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
