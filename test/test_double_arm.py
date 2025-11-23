import time
from ArmDriver.DM_CAN import *
from ArmDriver.ArmDriver import RobotController

if __name__ == "__main__":
    leader = RobotController(port='/dev/ttyACM0', type='leader')
    leader.connect()
    leader.enable()
    time.sleep(0.5)

    follower = RobotController(port='/dev/ttyACM1', type='follower')
    follower.connect()
    follower.enable()
    time.sleep(0.5)

    while True:
        leader.gravity_compensation()
        gripper = leader.get_current_gripper_angles()

        follower.set_joint_angles(leader.get_current_joint_angles(), 4)
        follower.set_gripper_angles(gripper * 2, 2, 0.2)

        time.sleep(0.02)
