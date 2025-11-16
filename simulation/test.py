from joyconrobotics import JoyconRobotics
import time

joyconrobotics_right = JoyconRobotics("left", horizontal_stick_mode="yaw_diff")

for i in range(10000):
    pose, gripper, control_button = joyconrobotics_right.get_control()
    print(f'{pose=}, {gripper=}, {control_button=}')
    time.sleep(0.02)

joyconrobotics_right.disconnect()