import time
from ArmDriver.ArmDriver import RobotController
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive Serial Shell")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port for the Master Arm (e.g. COM3 or /dev/ttyACM0)",
    )
    args = parser.parse_args()
    RobotCtrl = RobotController(port=args.port)
    RobotCtrl.connect()

    # if not RobotCtrl.set_zero():
    #     print("Set Zero failed")

    # if not RobotCtrl.set_mode():
    #     print("Arm mode set failed")

    # if not RobotCtrl.set_mit_mode():
    #     print("Arm MIT mode set failed")

    # if not RobotCtrl.set_joint_param():
    #     print("Arm joint param set failed")

    for joint in RobotCtrl.joints:
        print('----------------------------------------------')
        RobotCtrl.get_motor_params(joint)

    print('----------------------------------------------')
    RobotCtrl.get_motor_params(RobotCtrl.gripper)

    if not RobotCtrl.enable():
        print("Arm enable failed")
    
    time.sleep(1)

    # while True:
    #     current_angles = RobotCtrl.get_current_joint_angles()
    #     print("q:", [angle for angle in current_angles])

    #     current_gripper = RobotCtrl.get_current_gripper_angles()
    #     print("gripper: ", current_gripper)

    #     time.sleep(0.1)

    if not RobotCtrl.disable():
        print("Arm disable failed")
    
    if not RobotCtrl.disconnect():
        print("Arm disconnect failed")