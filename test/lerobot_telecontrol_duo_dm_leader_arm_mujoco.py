from ArmDriver.DuoDmArmLeader import DuoDmArmLeaderConfig, DuoDmArmLeader
from ArmDriver.RobotKinematics import MujocoDuoRobot
import time

leader_config = DuoDmArmLeaderConfig(
    left_port="/dev/ttyACM0",
    right_port="/dev/ttyACM1",
)

leader = DuoDmArmLeader(leader_config)
leader.connect()

class ViewerClosedException(Exception):
    pass

try:
    sim = MujocoDuoRobot()

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
    raise ViewerClosedException("Viewer window closed")
except (KeyboardInterrupt, ViewerClosedException) as e:
    print(f"Detected exit: {e}")
    print("\nStopping teleop...")
    leader.disconnect()
