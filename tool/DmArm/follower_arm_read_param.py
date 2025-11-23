import time
from ArmDriver.ArmDriver import RobotController
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive Serial Shell")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port for the Arm (e.g. COM3 or /dev/ttyACM0)",
    )
    args = parser.parse_args()
    RobotCtrl = RobotController(port=args.port, type="follower")
    if not RobotCtrl.connect():
        print("Arm connect failed")

    if not RobotCtrl.enable():
        print("Arm enable failed")

    if not RobotCtrl.set_zero():
        print("Set Zero failed")

    if not RobotCtrl.set_joint_pos_vel_param():
        print("Arm joint param set failed")

    for joint in RobotCtrl.joints:
        print('----------------------------------------------')
        RobotCtrl.get_motor_params(joint)

    print('----------------------------------------------')
    RobotCtrl.get_motor_params(RobotCtrl.gripper)

    if not RobotCtrl.disable():
        print("Arm disable failed")
    
    if not RobotCtrl.disconnect():
        print("Arm disconnect failed")