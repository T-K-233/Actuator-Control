import time

import numpy as np
from loop_rate_limiters import RateLimiter

from actuator_control import Motor, RobstrideBus


channel = "can0"
bitrate = 1000000


motors = {
    "motor_1": Motor(id=11, model="rs-02"),
    "motor_2": Motor(id=12, model="rs-00"),
}


control_frequency = 50.0


rate = RateLimiter(frequency=control_frequency)
bus = RobstrideBus(channel=channel, motors=motors, bitrate=bitrate)
bus.connect()

bus.write_mit_kp_kd("motor_1", 0.1, 1.0)
bus.write_mit_kp_kd("motor_2", 0.1, 1.0)

for name, motor in motors.items():
    bus.enable(name)


try:
    while True:
        target_position_1 = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        target_position_2 = np.cos(2 * np.pi * 1 * time.time()) * 0.2
        bus.write_mit_control(motor="motor_1", position=target_position_1, velocity=0, torque=0)
        bus.write_mit_control(motor="motor_2", position=target_position_2, velocity=0, torque=0)

        position_0, velocity_0 = bus.read_mit_state(motor="motor_1")
        position_1, velocity_1 = bus.read_mit_state(motor="motor_2")
        print(f"position: {position_0:.3f}, {position_1:.3f}, velocity: {velocity_0:.3f}, {velocity_1:.3f}")
        rate.sleep()

except KeyboardInterrupt:
    pass

for name in motors.keys():
    bus.disable(name)
bus.disconnect()

print("Program terminated.")
