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
        help="Serial port for the Arm (e.g. COM3 or /dev/ttyACM0)",
    )
    args = parser.parse_args()

    RobotCtrl = RobotController(port=args.port, type='leader')
    RobotCtrl.connect()
    RobotCtrl.enable()

    sim = MujocoRobot()
    while sim.viewer.is_running():
        RobotCtrl.gravity_compensation()
        q = RobotCtrl.get_current_joint_angles()
        gripper = RobotCtrl.get_current_gripper_angles()

        print(q, gripper)
        sim.update_param(q, gripper=-gripper*0.05/1.4)
        time.sleep(0.01)
    
    RobotCtrl.disable()
    RobotCtrl.disconnect()
