from ArmDriver.UArmLeader import UArmLeader, UArmLeaderConfig
from ArmDriver.RobotKinematics import MujocoRobot
import time

leader_config = UArmLeaderConfig(
    port="/dev/ttyUSB0",
    fps=50
)

leader = UArmLeader(leader_config)
leader.connect()

try:
    sim = MujocoRobot()
    while sim.viewer.is_running():
        start = time.perf_counter()
        action = leader.get_action()

        q = [
            action["joint_1.pos"],
            action["joint_2.pos"],
            action["joint_3.pos"],
            action["joint_4.pos"],
            action["joint_5.pos"],
            action["joint_6.pos"],
        ]

        gripper = action["gripper"]
        print(gripper)
        sim.update_param(q, gripper=-gripper*0.05/1.35)

        elapsed = time.perf_counter() - start
        if elapsed < 0.032:
            time.sleep(0.032 - elapsed)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
