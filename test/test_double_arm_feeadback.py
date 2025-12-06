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
    # cnt = 0
    del_tau = [0, 0, 0, 0, 0, 0]

    try:
        while True:
            leader.gravity_compensation(del_tau)
            gripper = leader.get_current_gripper_angles()

            follower.set_joint_angles(leader.get_current_joint_angles(), 4)
            follower.set_gripper_angles(gripper * 2, 2, 0.2)
            follower_tau = follower.get_current_joint_tau()
            follower_gravity = follower.gravity()
            # leader_tau = leader.get_current_joint_tau()
            # del_tau = []
            # cnt += 1

            for i in range(6):
                del_tau[i] = (follower_tau[i] - follower_gravity[i])
            # if cnt % 25 == 0:
            #     print(f"leader_tau = {leader_tau}")
            #     print(f"follower_tau = {follower_tau}")
            #     print(f"follower_gravity = {follower_gravity}")
            #     print(f"del_tau = {del_tau}")
            # time.sleep(0.01)
    except KeyboardInterrupt:
        leader.disable()
        leader.disconnect()

        follower.disable()
        follower.disconnect()
