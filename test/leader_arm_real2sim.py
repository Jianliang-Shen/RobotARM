import time
from ArmDriver.DM_CAN import *
from ArmDriver.ArmDriver import RobotController
import argparse
from ArmDriver.RobotKinematics import MujocoRobot

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive Serial Shell")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port for the Master Arm (e.g. COM3 or /dev/ttyACM0)",
    )
    args = parser.parse_args()

    controller = RobotController(port=args.port, type='leader')
    controller.enable()

    sim = MujocoRobot()
    while sim.viewer.is_running():
        controller.gravity_compensation()
        q = controller.get_current_joint_angles()
        gripper = controller.get_current_gripper_angles()

        print(q, gripper)
        sim.update_param(q, gripper=-gripper*0.05/1.4)
        time.sleep(0.01)

