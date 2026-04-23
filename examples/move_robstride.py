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

for name in actuators:
    bus.enable(name)


try:
    while True:
        target_position_1 = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        target_position_2 = np.cos(2 * np.pi * 1 * time.time()) * 0.2
        bus.write_mit_control(
            actuator="actuator_1",
            position=target_position_1,
            velocity=0.0,
            kp=0.1,
            kd=1.0,
            torque=0.0,
        )
        bus.write_mit_control(
            actuator="actuator_2",
            position=target_position_2,
            velocity=0.0,
            kp=0.1,
            kd=1.0,
            torque=0.0,
        )

        state_0 = bus.get_mit_state(actuator="actuator_1")
        state_1 = bus.get_mit_state(actuator="actuator_2")
        if state_0 is None or state_1 is None:
            rate.sleep()
            continue

        position_0, velocity_0, torque_0 = state_0
        position_1, velocity_1, torque_1 = state_1
        print(f"position: {position_0:.3f}, {position_1:.3f}, velocity: {velocity_0:.3f}, {velocity_1:.3f}, torque: {torque_0:.3f}, {torque_1:.3f}")
        rate.sleep()

except KeyboardInterrupt:
    pass

for name in actuators:
    bus.disable(name)
bus.disconnect()

print("Program terminated.")
