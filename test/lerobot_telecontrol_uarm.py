from ArmDriver.DmArmFollower import DmArmFollower, DmArmFollowerConfig
from ArmDriver.UArmLeader import UArmLeader, UArmLeaderConfig
import time


follower_config = DmArmFollowerConfig(
    port="/dev/ttyACM1",
)

leader_config = UArmLeaderConfig(
    port="/dev/ttyUSB0",
    fps=30
)

leader = UArmLeader(leader_config)
leader.connect()

follower = DmArmFollower(follower_config)
follower.connect()

try:
    while True:
        start = time.perf_counter()
        action = leader.get_action()
        follower.send_action(action)

        elapsed = time.perf_counter() - start
        if elapsed < 0.032:
            time.sleep(0.032 - elapsed)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
