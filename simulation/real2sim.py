import numpy as np
import time
from pynput import keyboard
from DmArm.ArmDriver import ArmDriver
import argparse
import time
from DmArm.RobotKinematics import MujocoRobot

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive Serial Shell")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port for the Arm (e.g. COM3 or /dev/ttyACM0)",
    )
    args = parser.parse_args()

    # Initialize the arm
    arm = ArmDriver(port=args.port)
    arm.connect()
    arm.enable()

    # Sync with simulation
    sim = MujocoRobot()
    while sim.viewer.is_running():   
        q = arm.get_q()
        sim.update_param(q[:-1], gripper=0)

        time.sleep(0.01)
