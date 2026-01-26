from dataclasses import dataclass
import logging

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ArmDriver.DmArmLeader import DmArmLeader, DmArmLeaderConfig

logger = logging.getLogger(__name__)

@TeleoperatorConfig.register_subclass("duo_dm_arm_leader")
@dataclass
class DuoDmArmLeaderConfig(TeleoperatorConfig):
    left_port: str
    right_port: str

class DuoDmArmLeader(Teleoperator):
    config_class = DuoDmArmLeaderConfig
    name = "duo_dm_arm_leader"

    def __init__(self, config: DuoDmArmLeaderConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False


    @property
    def action_features(self) -> dict[str, type]:
        return {}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = False) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_config = DmArmLeaderConfig(port=self.config.left_port)
        self.right_config = DmArmLeaderConfig(port=self.config.right_port)

        self.left = DmArmLeader(self.left_config)
        self.right = DmArmLeader(self.right_config)

        self.left.connect()
        self.right.connect()

        if self.left.is_connected and self.right.is_connected:
            self._is_connected = True
        else:
            print("Follower Duo Arm connected fail")

        self.configure()
        logger.info(f"{self} connected.")

    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        print("configure leader arm down")

    def setup_motors(self) -> None:
        print("set up motor, do nothing here")

    def get_action(self) -> dict[str, float]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        self.left.arm.gravity_compensation()
        self.right.arm.gravity_compensation()
        self.left_action = self.left.get_action()
        self.right_action = self.right.get_action()

        action = {}
        
        action["left.joint_1.pos"] = self.left_action["joint_1.pos"]
        action["left.joint_2.pos"] = self.left_action["joint_2.pos"]
        action["left.joint_3.pos"] = self.left_action["joint_3.pos"]
        action["left.joint_4.pos"] = self.left_action["joint_4.pos"]
        action["left.joint_5.pos"] = self.left_action["joint_5.pos"]
        action["left.joint_6.pos"] = self.left_action["joint_6.pos"]
        action["left.gripper"] = self.left_action["gripper"]

        action["right.joint_1.pos"] = self.right_action["joint_1.pos"]
        action["right.joint_2.pos"] = self.right_action["joint_2.pos"]
        action["right.joint_3.pos"] = self.right_action["joint_3.pos"]
        action["right.joint_4.pos"] = self.right_action["joint_4.pos"]
        action["right.joint_5.pos"] = self.right_action["joint_5.pos"]
        action["right.joint_6.pos"] = self.right_action["joint_6.pos"]
        action["right.gripper"] = self.right_action["gripper"]
        
        # print(action)
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.left.disconnect()
        self.right.disconnect()

        self._is_connected = False

        logger.info(f"{self} disconnected.")
