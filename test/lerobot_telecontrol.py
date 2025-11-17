from ArmDriver.follower import DmArmFollower, DmArmFollowerConfig
from ArmDriver.leader import DmArmLeader, DmArmLeaderConfig
import time


follower_config = DmArmFollowerConfig(
    port="/dev/ttyACM1",
)

leader_config = DmArmLeaderConfig(
    port="/dev/ttyACM0",
)

leader = DmArmLeader(leader_config)
leader.connect()

follower = DmArmFollower(follower_config)
follower.connect()

try:
    while True:
        start = time.perf_counter()
        action = leader.get_action()
        follower.send_action(action)

        elapsed = time.perf_counter() - start
        if elapsed < 0.02:
            time.sleep(0.02 - elapsed)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
