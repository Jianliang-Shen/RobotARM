from ArmDriver.DuoDmArmFollower import DuoDmArmFollower, DuoDmArmFollowerConfig
from ArmDriver.DuoUArmLeader import DuoUArmLeader, DuoUArmLeaderConfig

import time


follower_config = DuoDmArmFollowerConfig(
    left_port="/dev/ttyACM0",
    right_port="/dev/ttyACM1"
)

leader_config = DuoUArmLeaderConfig(
    left_port="/dev/ttyUSB0",
    right_port="/dev/ttyUSB1",
    fps=30
)

leader = DuoUArmLeader(leader_config)
leader.connect()

follower = DuoDmArmFollower(follower_config)
follower.connect()

try:
    while True:
        start = time.perf_counter()
        action = leader.get_action()
        follower.send_action(action)

        elapsed = time.perf_counter() - start
        print(elapsed)
        if elapsed < 0.032:
            time.sleep(0.032 - elapsed)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
