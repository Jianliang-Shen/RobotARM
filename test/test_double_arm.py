import time
from ArmDriver.DM_CAN import *
from ArmDriver.ArmDriver import RobotController

if __name__ == "__main__":
    master = RobotController(port='/dev/ttyACM0', type='leader')
    master.enable()
    time.sleep(0.5)

    slave = RobotController(port='/dev/ttyACM1', type='follower')
    slave.enable()
    time.sleep(0.5)

    while True:
        master.gravity_compensation()
        gripper = master.get_current_gripper_angles()

        slave.set_joint_angles(master.get_current_joint_angles(), 4)
        slave.set_gripper_angles(gripper * 2, 2, 0.2)

        time.sleep(0.02)
