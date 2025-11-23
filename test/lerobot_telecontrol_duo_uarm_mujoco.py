from ArmDriver.DuoUArmLeader import DuoUArmLeader, DuoUArmLeaderConfig
from ArmDriver.RobotKinematics import MujocoDuoRobot
import time

leader_config = DuoUArmLeaderConfig(
    left_port="/dev/ttyUSB0",
    right_port="/dev/ttyUSB1",
    fps=30
)

leader = DuoUArmLeader(leader_config)
leader.connect()

try:
    sim = MujocoDuoRobot(x = 1, y = 0)
    # sim_right = MujocoRobot(x = -1, y = 0)

    while sim.viewer.is_running():
        start = time.perf_counter()
        action = leader.get_action()

        left_q = [
            action["left.joint_1.pos"],
            action["left.joint_2.pos"],
            action["left.joint_3.pos"],
            action["left.joint_4.pos"],
            action["left.joint_5.pos"],
            action["left.joint_6.pos"],
        ]

        right_q = [
            action["right.joint_1.pos"],
            action["right.joint_2.pos"],
            action["right.joint_3.pos"],
            action["right.joint_4.pos"],
            action["right.joint_5.pos"],
            action["right.joint_6.pos"],
        ]

        left_gripper = action["left.gripper"]
        right_gripper = action["right.gripper"]
        sim.update_left(left_q, gripper=-left_gripper*0.05/1.35)
        sim.update_right(right_q, gripper=-right_gripper*0.05/1.35)
        sim.render()

        elapsed = time.perf_counter() - start
        if elapsed < 0.032:
            time.sleep(0.032 - elapsed)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
