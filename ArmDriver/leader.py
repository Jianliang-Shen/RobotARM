from dataclasses import dataclass
import logging
import time
import numpy as np
import threading

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ArmDriver.ArmDriver import RobotController

logger = logging.getLogger(__name__)

@TeleoperatorConfig.register_subclass("dm_arm_leader")
@dataclass
class DmArmLeaderConfig(TeleoperatorConfig):
    port: str

class DmArmLeader(Teleoperator):
    config_class = DmArmLeaderConfig
    name = "dm_arm_leader"

    def __init__(self, config: DmArmLeaderConfig):
        super().__init__(config)
        self.config = config
        self.results = [0, 0, 0, 0, 0, 0]
        self.prev_frame = self.results
        self.lock = threading.Lock()
        self.stop_flag = False
        self.t1 = None
        self.t2 = None
        self._port = self.config.port
        self._baudrate = 921600
        self._ser = None
        self._is_connected = False
        self._calibration_data = []

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

        self.arm = RobotController(self.config.port, type='leader')
        if self.arm.RobotCtrl.serial_.is_open:
            self._is_connected = True
        else:
            print("Follower Arm connected fail")

        self.configure()
        logger.info(f"{self} connected.")

    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def send_master(self, cmd):
        self._ser.write(cmd.encode('ascii'))
        time.sleep(0.007)

    def configure(self) -> None:
        self.arm.enable()
        time.sleep(0.5)
        print("configure leader arm down")

    def setup_motors(self) -> None:
        print("set up motor, do nothing here")

    def get_action(self) -> dict[str, float]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        self.arm.gravity_compensation()
        self.results = self.arm.get_current_joint_angles()

        action = {}
        
        action["joint_1.pos"] = self.results[0]
        action["joint_2.pos"] = self.results[1]
        action["joint_3.pos"] = self.results[2]
        action["joint_4.pos"] = self.results[3]
        action["joint_5.pos"] = self.results[4]
        action["joint_6.pos"] = self.results[5]
        action["gripper"] = self.arm.get_current_gripper_angles()
        
        # print(action)
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.arm.disable()

        logger.info(f"{self} disconnected.")
