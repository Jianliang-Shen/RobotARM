from dataclasses import dataclass, field
from functools import cached_property
import serial
import time
import logging
from typing import Any
import re

from lerobot.cameras import CameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot, RobotConfig
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ArmDriver.ArmDriver import RobotController
from ArmDriver.DmArmFollower import DmArmFollower, DmArmFollowerConfig

logger = logging.getLogger(__name__)


@RobotConfig.register_subclass("duo_dm_arm_follower")
@dataclass
class DuoDmArmFollowerConfig(RobotConfig):
    left_port: str
    right_port: str
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


class DuoDmArmFollower(Robot):
    """
    DM Arm Follower Arm designed by The Robot Learning Company.
    """

    config_class = DuoDmArmFollowerConfig
    name = "duo_dm_arm_follower"

    def __init__(self, config: DuoDmArmFollowerConfig):
        super().__init__(config)

        self.config = config
        # self._ser = None
        self._is_connected = False
        self.obs_dict = {}
        self.arm =None
        self.first_action_received = False


        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        pos_dict = {
            "left.joint_1.pos": 0,
            "left.joint_2.pos": 0,
            "left.joint_3.pos": 0,
            "left.joint_4.pos": 0,
            "left.joint_5.pos": 0,
            "left.joint_6.pos": 0,
            "left.gripper": 0,
            "right.joint_1.pos": 0,
            "right.joint_2.pos": 0,
            "right.joint_3.pos": 0,
            "right.joint_4.pos": 0,
            "right.joint_5.pos": 0,
            "right.joint_6.pos": 0,
            "right.gripper": 0,
        }

        return pos_dict

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_config = DmArmFollowerConfig(port=self.config.left_port)
        self.right_config = DmArmFollowerConfig(port=self.config.right_port)
        self.left = DmArmFollower(self.left_config)
        self.right = DmArmFollower(self.right_config)
        self.left.connect()
        self.right.connect()

        if self.left.is_connected and self.right.is_connected:
            self._is_connected = True
        else:
            print("Follower Duo Arm connected fail")

        self.configure()

        for cam in self.cameras.values():
            cam.connect()
        
        print("Camera connect down")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        print("configure follower arm down")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()

        left_pos = self.left.get_current_joint_angles()

        self.obs_dict["left.joint_1.pos"] = left_pos[0]
        self.obs_dict["left.joint_2.pos"] = left_pos[1]
        self.obs_dict["left.joint_3.pos"] = left_pos[2]
        self.obs_dict["left.joint_4.pos"] = left_pos[3]
        self.obs_dict["left.joint_5.pos"] = left_pos[4]
        self.obs_dict["left.joint_6.pos"] = left_pos[5]
        self.obs_dict["left.gripper"] = self.left.get_current_gripper_angles()

        right_pos = self.right.get_current_joint_angles()

        self.obs_dict["right.joint_1.pos"] = right_pos[0]
        self.obs_dict["right.joint_2.pos"] = right_pos[1]
        self.obs_dict["right.joint_3.pos"] = right_pos[2]
        self.obs_dict["right.joint_4.pos"] = right_pos[3]
        self.obs_dict["right.joint_5.pos"] = right_pos[4]
        self.obs_dict["right.joint_6.pos"] = right_pos[5]
        self.obs_dict["right.gripper"] = self.right.get_current_gripper_angles()

        dt_ms = (time.perf_counter() - start) * 1e3
        print(f"read state: {dt_ms:.1f}ms")
        # print(self.obs_dict)

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            self.obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            # print(f"{self} read {cam_key}: {dt_ms:.1f} ms")

        return self.obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        right_action = {
            "joint_1.pos": action["right.joint_1.pos"],
            "joint_2.pos": action["right.joint_2.pos"],
            "joint_3.pos": action["right.joint_3.pos"],
            "joint_4.pos": action["right.joint_4.pos"],
            "joint_5.pos": action["right.joint_5.pos"],
            "joint_6.pos": action["right.joint_6.pos"],
            "gripper": action["right.gripper"],
        }
        self.right.send_action(right_action)

        left_action = {
            "joint_1.pos": action["left.joint_1.pos"],
            "joint_2.pos": action["left.joint_2.pos"],
            "joint_3.pos": action["left.joint_3.pos"],
            "joint_4.pos": action["left.joint_4.pos"],
            "joint_5.pos": action["left.joint_5.pos"],
            "joint_6.pos": action["left.joint_6.pos"],
            "gripper": action["left.gripper"],
        }
        self.left.send_action(left_action)


        return action

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError("DuoDmArmFollower is not connected.")

        self.left.disconnect()
        self.right.disconnect()
        self._is_connected = False

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info("DuoDmArmFollower disconnected.")
