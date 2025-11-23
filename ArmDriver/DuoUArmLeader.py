from dataclasses import dataclass
import logging
import time
import numpy as np
import threading
import math
import serial
import re
import os

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ArmDriver.UArmLeader import UArmLeader, UArmLeaderConfig

logger = logging.getLogger(__name__)

@TeleoperatorConfig.register_subclass("duo_uarm_leader")
@dataclass
class DuoUArmLeaderConfig(TeleoperatorConfig):
    left_port: str
    right_port: str
    fps: int

class DuoUArmLeader(Teleoperator):
    config_class = DuoUArmLeaderConfig
    name = "duo_uarm_leader"

    def __init__(self, config: DuoUArmLeaderConfig):
        super().__init__(config)
        self.config = config
        self.stop_flag = False
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
        self.left_config = UArmLeaderConfig(port=self.config.left_port, fps=30, cali_file="left.cali")
        self.right_config = UArmLeaderConfig(port=self.config.right_port, fps=30, cali_file="right.cali")
        self.left = UArmLeader(self.left_config)
        self.right = UArmLeader(self.right_config)
        self.configure()
        
        logger.info(f"{self} connected.")

    def is_calibrated(self) -> bool:
        return self.left.is_calibrated() and self.right.is_calibrated()


    def calibrate(self) -> None:
        self.left.connect()
        self.right.connect()
        self._is_connected = True
        self.left.calibrate()
        self.right.calibrate()

    def configure(self) -> None:
        if self.is_calibrated():
            self.left.connect()
            self.right.connect()
            self._is_connected = True
        else:
            print("Duo leader arm is not calibrated!")
        
    def setup_motors(self) -> None:
        print("set up motor, do nothing here")

    def get_action(self) -> dict[str, float]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

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
        logger.info(f"{self} disconnected.")
