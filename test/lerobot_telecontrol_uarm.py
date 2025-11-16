from ArmDriver.follower import DmArmFollower, DmArmFollowerConfig
from ArmDriver.servo_leader import ServoArmLeader, ServoArmLeaderConfig
import time


follower_config = DmArmFollowerConfig(
    port="/dev/ttyACM0",
)

leader_config = ServoArmLeaderConfig(
    port="/dev/ttyUSB0",
    fps=50
)

leader = ServoArmLeader(leader_config)
leader.connect()

follower = DmArmFollower(follower_config)
follower.connect()

try:
    while True:
        start = time.perf_counter()
        action = leader.get_action()
        follower.send_action(action)

        elapsed = time.perf_counter() - start
        # print(elapsed)
        if elapsed < 0.02:
            time.sleep(0.02 - elapsed)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
