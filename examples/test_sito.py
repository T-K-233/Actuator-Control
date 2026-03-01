import time

import numpy as np
from loop_rate_limiters import RateLimiter

from actuator_control.sito import SitoBus, Motor


channel = "can2"
bitrate = 1000000


motors = {
    "left_wrist_roll": Motor(id=0x16, model="TA40"),
    "left_wrist_pitch": Motor(id=0x17, model="TA40"),
}


# can go up to 1 kHz with 2 actuators
control_frequency = 50.0


rate = RateLimiter(frequency=control_frequency)
bus = SitoBus(channel=channel, motors=motors, control_frequency=control_frequency)
bus.connect()

bus.write_mit_kp_kd("left_wrist_roll", 20.0, 1.0)
bus.write_mit_kp_kd("left_wrist_pitch", 20.0, 1.0)

for name, motor in motors.items():
    bus.enable(name)


try:
    while True:
        target_position_1 = np.sin(2 * np.pi * 1 * time.time()) * 0.2
        target_position_2 = np.cos(2 * np.pi * 1 * time.time()) * 0.2
        bus.write_mit_control(motor="left_wrist_roll", position=target_position_1, velocity=0, torque=0)
        bus.write_mit_control(motor="left_wrist_pitch", position=target_position_2, velocity=0, torque=0)

        position_0, velocity_0 = bus.read_mit_state(motor="left_wrist_roll")
        position_1, velocity_1 = bus.read_mit_state(motor="left_wrist_pitch")
        print(f"position: {position_0:.3f}, {position_1:.3f}, velocity: {velocity_0:.3f}, {velocity_1:.3f}")
        rate.sleep()

except KeyboardInterrupt:
    pass

for name in motors.keys():
    bus.disable(name)
bus.disconnect()

print("Program terminated.")
