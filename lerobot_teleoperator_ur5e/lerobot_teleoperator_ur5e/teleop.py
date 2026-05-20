#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from pathlib import Path
from .dynamixel.dynamixel_robot import DynamixelRobot
from typing import Any, Dict
import yaml
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.teleoperators.teleoperator import Teleoperator
from .config_teleop import UR5eTeleopConfig
logger = logging.getLogger(__file__)
logger.setLevel(logging.INFO)
class UR5eTeleop(Teleoperator):
    """
    Isomorphic Teleop class for controlling a single robot arm.
    """

    config_class = UR5eTeleopConfig
    name = "IsoTeleop"

    def __init__(self, config: UR5eTeleopConfig):
        super().__init__(config)
        self.cfg = config
        self._is_connected = False
        self.robot = None
        self.urdf_path = Path(__file__).parents[2] / self.cfg.robot_urdf_path

    @property
    def action_features(self) -> dict:
        return {}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        pass

    def set_robot(self, robot) -> None:
        self.robot = robot

    def connect(self) -> None:
        if self.cfg.control_space not in ("joint", "joint_to_tcp_force", "tcp_force", "tcp_position"):
            raise ValueError(
                f"Unsupported control_space: {self.cfg.control_space}. "
                "Expected 'joint', 'joint_to_tcp_force', 'tcp_force', or 'tcp_position'."
            )
        if self.cfg.control_space == "tcp_force" and self.cfg.tcp_force_reference_frame not in ("base", "tcp"):
            raise ValueError(
                f"Unsupported tcp_force.reference_frame: {self.cfg.tcp_force_reference_frame}. "
                "Expected 'base' or 'tcp'."
            )
        if self.cfg.control_space == "tcp_position" and self.cfg.tcp_position_reference_frame not in ("base", "tcp"):
            raise ValueError(
                f"Unsupported tcp_position.reference_frame: {self.cfg.tcp_position_reference_frame}. "
                "Expected 'base' or 'tcp'."
            )

        self._check_dynamixel_connection()
        if self.cfg.control_space in ("tcp_force", "tcp_position"):
            self._init_pinocchio(self.urdf_path, base_frame="base", ee_frame="tool0")
        self._is_connected = True
        logger.info(f"[INFO] {self.name} env initialization completed successfully.\n")

    def _check_dynamixel_connection(self) -> None:
        logger.info("\n===== [TELEOP] Connecting to dynamixel Robot =====")
        self.dynamixel_robot = DynamixelRobot(
                hardware_offsets=self.cfg.hardware_offsets,
                joint_ids=self.cfg.joint_ids,
                joint_offsets=self.cfg.joint_offsets,
                joint_signs=self.cfg.joint_signs,
                port=self.cfg.port,
                use_gripper=self.cfg.use_gripper,
                gripper_config=self.cfg.gripper_config,
                real=True
                )
        joint_positions = self.dynamixel_robot.get_joint_state()
        logger.info(f"[TELEOP] Current joint positions: {joint_positions.tolist()}")
        logger.info("===== [TELEOP] Dynamixel robot connected successfully. =====\n")
    
    def calibrate(self) -> None:
        pass

    def configure(self):
        pass

    def get_action(self) -> dict[str, Any]:
        if self.cfg.control_space in ("tcp_force", "tcp_position"):
            return self._get_delta_action()

        return self.dynamixel_robot.get_observations()

    def _init_pinocchio(self, urdf_path: str, base_frame: str = "base", ee_frame: str = "tool0"):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.base_id = self.model.getFrameId(base_frame)
        self.ee_frame_id = self.model.getFrameId(ee_frame)
        self.data = self.model.createData()

    def _fk(self, joint_positions):
        q = np.array(joint_positions)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        m_tool = self.data.oMf[self.ee_frame_id]
        m_base = self.data.oMf[self.base_id]
        m_rel = m_base.inverse() * m_tool
        position = m_rel.translation
        rotvec = pin.log3(m_rel.rotation)
        return np.concatenate([position, rotvec])

    def _get_delta_action(self) -> dict[str, Any]:
        if self.robot is None:
            raise ValueError(f"{self.cfg.control_space} requires a robot object on teleop.")

        observations = self.dynamixel_robot.get_observations()
        joint_positions = np.array([observations[f"joint_{i+1}.pos"] for i in range(6)], dtype=float)
        target_ee_pose = self._fk(joint_positions)
        current_ee_pose = np.array(self.robot.get_ee_pose(), dtype=float)

        target_position = np.array(target_ee_pose[:3], dtype=float)
        current_position = np.array(current_ee_pose[:3], dtype=float)
        target_rotation = R.from_rotvec(target_ee_pose[3:]).as_matrix()
        current_rotation = R.from_rotvec(current_ee_pose[3:]).as_matrix()

        reference_frame = (
            self.cfg.tcp_force_reference_frame
            if self.cfg.control_space == "tcp_force"
            else self.cfg.tcp_position_reference_frame
        )

        if reference_frame == "base":
            delta_position = target_position - current_position
            delta_rotation = target_rotation @ current_rotation.T
        elif reference_frame == "tcp":
            delta_position = current_rotation.T @ (target_position - current_position)
            delta_rotation = current_rotation.T @ target_rotation
        else:
            raise ValueError(f"Unsupported {self.cfg.control_space}.reference_frame: {reference_frame}")

        delta_euler = R.from_matrix(delta_rotation).as_euler("xyz")

        action = {
            "delta_x": float(delta_position[0]),
            "delta_y": float(delta_position[1]),
            "delta_z": float(delta_position[2]),
            "delta_rx": float(delta_euler[0]),
            "delta_ry": float(delta_euler[1]),
            "delta_rz": float(delta_euler[2]),
        }
        if "gripper_position" in observations:
            action["gripper_position"] = observations["gripper_position"]
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        
        self.dynamixel_robot._driver.close()
        logger.info(f"[INFO] ===== All {self.name} connections have been closed =====")

if __name__ == "__main__":
    import numpy as np
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    logger = logging.getLogger(__name__)

    class RecordConfig:
        def __init__(self, cfg: Dict[str, Any]):
            teleop = cfg["teleop"]
            dxl_cfg = teleop["dynamixel_config"]

            # teleop config
            self.port = dxl_cfg["port"]
            self.use_gripper = dxl_cfg["use_gripper"]  
            self.joint_ids = dxl_cfg["joint_ids"]
            self.hardware_offsets = dxl_cfg["hardware_offsets"]
            self.joint_offsets = dxl_cfg["joint_offsets"]
            self.joint_signs = dxl_cfg["joint_signs"]
            self.gripper_config = dxl_cfg["gripper_config"]
            self.control_mode = teleop.get("control_mode", "isoteleop")

    with open(Path(__file__).parent / "config" / "cfg.yaml", "r") as f:
        cfg = yaml.safe_load(f)

    record_cfg = RecordConfig(cfg["record"])
    teleop_config = UR5eTeleopConfig(
        port=record_cfg.port,
        use_gripper=record_cfg.use_gripper,
        hardware_offsets=record_cfg.hardware_offsets,
        joint_ids=record_cfg.joint_ids,
        joint_offsets=record_cfg.joint_offsets,
        joint_signs=record_cfg.joint_signs,
        gripper_config=record_cfg.gripper_config,
        control_mode=record_cfg.control_mode,       
    )
    teleop = UR5eTeleop(teleop_config)
    teleop.connect()
    for i in range(2):
        teleop.get_action()
    # teleop.dynamixel_robot._driver.set_operating_mode(3)
    # teleop.dynamixel_robot.set_torque_mode(True)
    # teleop.dynamixel_robot.command_joint_state(np.array([3.141129970550537, -2.003148218194479, 1.5803211371051233, -1.1479324859431763, -1.5713160673724573, -0.00014955202211552887, 3]))
