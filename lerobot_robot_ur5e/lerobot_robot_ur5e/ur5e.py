import logging
import time
from typing import Any
import threading
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from pyDHgripper import PGE
from .config_ur5e import UR5eConfig
from pathlib import Path
import pinocchio as pin
from datetime import datetime
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
class UR5e(Robot):
    config_class = UR5eConfig
    name = "ur5e"

    def __init__(self, config: UR5eConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        self.config = config
        self._is_connected = False
        self._arm = {}
        self._gripper = None
        self._initial_pose = None
        self._prev_observation = None
        self._episode_reference_ee_pose = None
        self._num_joints = 6
        self._gripper_force = config.gripper_force
        self._gripper_speed = config.gripper_speed
        self._gripper_position = 1
        self._velocity = 0.5 # not used in current version
        self._acceleration = 0.5 # not used in current version
        self._last_gripper_position = 1
        self.urdf_path=Path(__file__).parents[2] / self.config.robot_urdf_path
        self.task_frame= [0,0,0,0,0,0]
        self.type=2
            
    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        if self.config.control_space not in ("joint", "joint_to_tcp_force", "tcp_force", "tcp_position"):
            raise ValueError(
                f"Unsupported control_space: {self.config.control_space}. "
                "Expected 'joint', 'joint_to_tcp_force', 'tcp_force', or 'tcp_position'."
            )
        if self.config.control_space == "tcp_force" and self.config.tcp_force_reference_frame not in ("base", "tcp"):
            raise ValueError(
                f"Unsupported tcp_force.reference_frame: {self.config.tcp_force_reference_frame}. "
                "Expected 'base' or 'tcp'."
            )
        if self.config.control_space == "tcp_position" and self.config.tcp_position_reference_frame not in ("base", "tcp"):
            raise ValueError(
                f"Unsupported tcp_position.reference_frame: {self.config.tcp_position_reference_frame}. "
                "Expected 'base' or 'tcp'."
            )

        # Connect to robot
        self._arm['rtde_r'], self._arm['rtde_c'] = self._check_ur5e_connection(self.config.robot_ip)
        
        # Set force mode gain scaling
        if self.config.control_space in ("joint_to_tcp_force", "tcp_force"):
            self._arm["rtde_c"].forceModeSetGainScaling(self.config.gain_scale)
        
        # Init_pinocchio
        self._init_pinocchio(self.urdf_path, base_frame="base", ee_frame="tool0")
        
        # Initialize gripper
        if self.config.use_gripper:
            self._gripper = self._check_gripper_connection(self.config.gripper_port)

            # Start gripper state reader
            self._start_gripper_state_reader()

        # Connect cameras
        logger.info("\n===== [CAM] Initializing Cameras =====")
        for cam_name, cam in self.cameras.items():
            cam.connect()
            logger.info(f"[CAM] {cam_name} connected successfully.")
        logger.info("===== [CAM] Cameras Initialized Successfully =====\n")

        self.is_connected = True
        logger.info(f"[INFO] {self.name} env initialization completed successfully.\n")


    def _check_gripper_connection(self, port: str):
        logger.info("\n===== [GRIPPER] Initializing gripper...")
        gripper = PGE(port)
        gripper.init_feedback()
        gripper.set_force(self._gripper_force)
        gripper.set_vel(self._gripper_speed)
        logger.info(f"[GRIPPER] Force: {self._gripper_force}, speed: {self._gripper_speed}")
        logger.info("===== [GRIPPER] Gripper initialized successfully.\n")
        return gripper


    def _check_ur5e_connection(self, robot_ip: str):
        try:
            logger.info("\n===== [ROBOT] Connecting to UR5e robot =====")
            rtde_r = RTDEReceiveInterface(robot_ip)
            rtde_c = RTDEControlInterface(robot_ip)

            joint_positions = rtde_r.getActualQ()
            if joint_positions is not None and len(joint_positions) == 6:
                formatted_joints = [round(j, 4) for j in joint_positions]
                logger.info(f"[ROBOT] Current joint positions: {formatted_joints}")
                logger.info("===== [ROBOT] UR5e connected successfully =====\n")
            else:
                logger.info("===== [ERROR] Failed to read joint positions. Check connection or remote control mode =====")

        except Exception as e:
            logger.info("===== [ERROR] Failed to connect to UR5e robot =====")
            logger.info(f"Exception: {e}\n")

        return rtde_r, rtde_c

    def _start_gripper_state_reader(self):
        threading.Thread(target=self._read_gripper_state, daemon=True).start()

    def _read_gripper_state(self):
        self._gripper.pos = None
        while True:
            gripper_position = 0.0 if self._gripper_position  < self.config.close_threshold else 1.0
            if self.config.gripper_reverse:
                gripper_position = 1 - gripper_position

            if gripper_position != self._last_gripper_position:
                self._gripper.set_pos(val=int(1000 * gripper_position), blocking=False)
                self._last_gripper_position = gripper_position

            gripper_pos = self._gripper.read_pos() / 1000.0
            if self.config.gripper_reverse:
                gripper_pos = 1 - gripper_pos

            self._gripper.pos = gripper_pos
            time.sleep(0.01)

    @property
    def _motors_ft(self) -> dict[str, type]:
        joint_pos_features = {f"joint_{i}.pos": float for i in range(1, 7)}
        gripper_features = {
            "gripper_raw_position": float, # raw position in [0,1]
            "gripper_raw_bin": float, # raw position bin (0 or 1)
            "gripper_action_bin": float, # action command bin (0 or 1)
        }
        remaining_features = {
            **{f"joint_{i}.vel": float for i in range(1, 7)},
            **{f"joint_{i}.acc": float for i in range(1, 7)},
            **{f"joint_{i}.force": float for i in range(1, 7)},
        }
        tcp_pose_features = {f"tcp_pose.{axis}": float for axis in ["x", "y", "z", "rx", "ry", "rz"]}
        tcp_vel_features = {f"tcp_speed.{axis}": float for axis in ["x", "y", "z", "rx", "ry", "rz"]}
        tcp_acc_features = {f"tcp_acc.{axis}": float for axis in ["x", "y", "z"]}
        tcp_force_features = {f"tcp_force.{axis}": float for axis in ["x", "y", "z"]}
        tcp_torque_features = {f"tcp_force.{axis}": float for axis in ["rx", "ry", "rz"]}
        return {
            **joint_pos_features,
            **gripper_features,
            **remaining_features,
            **tcp_pose_features,
            **tcp_vel_features,
            **tcp_acc_features,
            **tcp_force_features,
            **tcp_torque_features,
        }

    @property
    def action_features(self) -> dict[str, type]:
        if self.config.control_space in ("tcp_force", "tcp_position"):
            features = {
                "delta_x": float,
                "delta_y": float,
                "delta_z": float,
                "delta_rx": float,
                "delta_ry": float,
                "delta_rz": float,
            }
            if self.config.use_gripper:
                features["gripper_position"] = float
            return features

        return {
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
            "gripper_position": float,
        }
        
    def _init_pinocchio(self, urdf_path: str, base_frame: str = "base", ee_frame: str = "tool0"):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.base_id = self.model.getFrameId(base_frame)
        self.ee_frame_id = self.model.getFrameId(ee_frame)
        self.data=self.model.createData()
        
    def _calculate_force(self, target_pos, curr_pos, curr_vel):
        # position
        diff_p = np.clip(np.array(target_pos[:3]) - np.array(curr_pos[:3]), -self.config.pos_delta, self.config.pos_delta)
        diff_d = np.clip(-np.array(curr_vel[:3]), -self.config.vel_delta, self.config.vel_delta)
        force_pos = self.config.kp * diff_p + self.config.kd * diff_d
        
        # orientation (Pinocchio version)
        R_target = pin.exp3(np.array(target_pos[3:]))
        R_curr   = pin.exp3(np.array(curr_pos[3:]))
        R_err = R_target @ R_curr.T
        rot_err = pin.log3(R_err)
        torque = (self.config.kp_rot * rot_err - self.config.kd_rot * np.array(curr_vel[3:])) / self.config.rtde_freq

        return np.concatenate((force_pos, torque))  

    def _fk(self, joint_positions):
        q = np.array(joint_positions)
        
        # forwardKinematics
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        M_tool = self.data.oMf[self.ee_frame_id]
        M_base = self.data.oMf[self.base_id]

        M_rel = M_base.inverse() * M_tool

        position = M_rel.translation
        rotvec = pin.log3(M_rel.rotation)

        return np.concatenate([position, rotvec])
    
    def _calculate_ft_target(self, action: dict[str, Any]) -> list[float]:
        joint_positions = [float(action[f"joint_{i+1}.pos"]) for i in range(self._num_joints)]
        target_pose = self._fk(joint_positions)
        curr_pose = [
            float(self.obs_dict["tcp_pose.x"]), float(self.obs_dict["tcp_pose.y"]), float(self.obs_dict["tcp_pose.z"]),
            float(self.obs_dict["tcp_pose.rx"]), float(self.obs_dict["tcp_pose.ry"]), float(self.obs_dict["tcp_pose.rz"])
        ]
        curr_vel = [
            float(self.obs_dict["tcp_speed.x"]), float(self.obs_dict["tcp_speed.y"]), float(self.obs_dict["tcp_speed.z"]),
            float(self.obs_dict["tcp_speed.rx"]), float(self.obs_dict["tcp_speed.ry"]), float(self.obs_dict["tcp_speed.rz"])
        ]
        ft_target = self._calculate_force(target_pose, curr_pose, curr_vel)  # [Fx,Fy,Fz,Tx,Ty,Tz]
        return ft_target

    def _pose_to_transform(self, pose: list[float] | np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
        transform[:3, 3] = pose[:3]
        return transform

    def _transform_to_pose(self, transform: np.ndarray) -> list[float]:
        return [
            *transform[:3, 3].tolist(),
            *R.from_matrix(transform[:3, :3]).as_rotvec().tolist(),
        ]

    def get_ee_pose(self) -> list[float]:
        tcp_pose = self._arm["rtde_r"].getActualTCPPose()
        tcp_offset = self._arm["rtde_c"].getTCPOffset()
        return self.tcp_to_ee_pose(tcp_pose, tcp_offset).tolist()

    def _ee_to_tcp_pose(self, ee_pose: list[float] | np.ndarray, tcp_offset: list[float] | np.ndarray) -> list[float]:
        ee_transform = self._pose_to_transform(ee_pose)
        offset_transform = self._pose_to_transform(tcp_offset)
        tcp_transform = ee_transform @ offset_transform
        return self._transform_to_pose(tcp_transform)

    def _target_pose_from_delta_action(self, action: dict[str, Any]) -> list[float]:
        current_tcp_pose = self._arm["rtde_r"].getActualTCPPose()
        tcp_offset = self._arm["rtde_c"].getTCPOffset()
        current_ee_pose = self.tcp_to_ee_pose(current_tcp_pose, tcp_offset)
        current_position = np.array(current_ee_pose[:3], dtype=float)
        current_rotation = R.from_rotvec(current_ee_pose[3:]).as_matrix()
        delta_position = np.array(
            [float(action["delta_x"]), float(action["delta_y"]), float(action["delta_z"])],
            dtype=float,
        )
        delta_rotation = R.from_euler(
            "xyz",
            [float(action["delta_rx"]), float(action["delta_ry"]), float(action["delta_rz"])],
        ).as_matrix()

        reference_frame = (
            self.config.tcp_force_reference_frame
            if self.config.control_space == "tcp_force"
            else self.config.tcp_position_reference_frame
        )

        if reference_frame == "base":
            target_position = current_position + delta_position
            target_rotation = delta_rotation @ current_rotation
        elif reference_frame == "tcp":
            target_position = current_position + current_rotation @ delta_position
            target_rotation = current_rotation @ delta_rotation
        else:
            raise ValueError(f"Unsupported {self.config.control_space}.reference_frame: {reference_frame}")

        target_transform = np.eye(4)
        target_transform[:3, :3] = target_rotation
        target_transform[:3, 3] = target_position
        target_ee_pose = self._transform_to_pose(target_transform)
        return self._ee_to_tcp_pose(target_ee_pose, tcp_offset)

    def _calculate_tcp_force_target(self, action: dict[str, Any]) -> list[float]:
        curr_pose = self._arm["rtde_r"].getActualTCPPose()
        curr_vel = self._arm["rtde_r"].getActualTCPSpeed()
        target_pose = self._target_pose_from_delta_action(action)
        return self._calculate_force(target_pose, curr_pose, curr_vel)
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not self.config.debug:
            t_start = self._arm["rtde_c"].initPeriod()
            if self.config.control_space == "joint":
                joint_positions = [float(action[f"joint_{i+1}.pos"]) for i in range(self._num_joints)]
                self._arm["rtde_c"].servoJ(joint_positions, self._velocity, self._acceleration, self.config.dt, self.config.look_ahead_time, self.config.gain)
            elif self.config.control_space == "joint_to_tcp_force":
                joint_positions = [float(action[f"joint_{i+1}.pos"]) for i in range(self._num_joints)]
                ft_target = self._calculate_ft_target(action)
                self._arm["rtde_c"].forceMode(self.task_frame,self.config.select_vector,ft_target,self.type,self.config.force_limit)
            elif self.config.control_space == "tcp_force":
                action_keys = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")
                if not all(key in action for key in action_keys):
                    raise ValueError(f"tcp_force action must contain {', '.join(action_keys)}.")
                ft_target = self._calculate_tcp_force_target(action)
                self._arm["rtde_c"].forceMode(self.task_frame,self.config.select_vector,ft_target,self.type,self.config.force_limit)
            elif self.config.control_space == "tcp_position":
                action_keys = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")
                if not all(key in action for key in action_keys):
                    raise ValueError(f"tcp_position action must contain {', '.join(action_keys)}.")
                target_pose = self._target_pose_from_delta_action(action)
                self._arm["rtde_c"].servoL(
                    target_pose,
                    self.config.tcp_position_speed,
                    self.config.tcp_position_acceleration,
                    self.config.tcp_position_servo_time,
                    self.config.tcp_position_lookahead_time,
                    self.config.tcp_position_gain,
                )
            else:
                raise ValueError(
                    f"Unsupported control_space: {self.config.control_space}. "
                    "Expected 'joint', 'joint_to_tcp_force', 'tcp_force', or 'tcp_position'."
                )
            self._arm["rtde_c"].waitPeriod(t_start)
                
        if "gripper_position" in action:
            self._gripper_position = float(action["gripper_position"])
            
        return action
    
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # Read joint positions
        joint_position = self._arm["rtde_r"].getActualQ()
            
        # Read joint velocities
        joint_velocity = self._arm["rtde_r"].getActualQd()

        # Read joint accelerations
        joint_acceleration = self._arm["rtde_r"].getTargetQdd()

        # Read joint forces
        joint_force = self._arm["rtde_c"].getJointTorques()

        # Read tcp pose
        tcp_pose = self._arm["rtde_r"].getActualTCPPose()
        tcp_offset = self._arm["rtde_c"].getTCPOffset()
        ee_pose = self.tcp_to_ee_pose(tcp_pose, tcp_offset)
        if self.config.control_space in ("tcp_force", "tcp_position"):
            reference_frame = (
                self.config.tcp_force_reference_frame
                if self.config.control_space == "tcp_force"
                else self.config.tcp_position_reference_frame
            )
            if reference_frame == "base":
                observation_tcp_pose = self._pose_euler(ee_pose)
            elif reference_frame == "tcp":
                observation_tcp_pose = self._relative_pose_euler(ee_pose)
            else:
                raise ValueError(f"Unsupported {self.config.control_space}.reference_frame: {reference_frame}")
        else:
            observation_tcp_pose = ee_pose

        # Read tcp speed
        tcp_speed = self._arm["rtde_r"].getActualTCPSpeed()

        # Read tcp acceleration
        tcp_acceleration = self._arm["rtde_r"].getActualToolAccelerometer()

        # Read tcp force
        tcp_force = self._arm["rtde_r"].getActualTCPForce()

        # Prepare observation dictionary
        self.obs_dict = {}

        for i in range(len(joint_position)):
            self.obs_dict[f"joint_{i+1}.pos"] = joint_position[i]
            self.obs_dict[f"joint_{i+1}.vel"] = joint_velocity[i]
            self.obs_dict[f"joint_{i+1}.acc"] = joint_acceleration[i]
            self.obs_dict[f"joint_{i+1}.force"] = joint_force[i]

        for i, axis in enumerate(["x", "y", "z","rx","ry","rz"]):
            self.obs_dict[f"tcp_pose.{axis}"] = observation_tcp_pose[i]
            self.obs_dict[f"tcp_speed.{axis}"] = tcp_speed[i]
            if i < 3: # tcp_acceleration have only 3 axes
                self.obs_dict[f"tcp_acc.{axis}"] = tcp_acceleration[i]
            self.obs_dict[f"tcp_force.{axis}"] = tcp_force[i]

        if self.config.use_gripper:
            self.obs_dict["gripper_raw_position"] = self._gripper.pos
            self.obs_dict["gripper_action_bin"] = self._last_gripper_position
            self.obs_dict["gripper_raw_bin"] = 0 if self._gripper.pos <= self.config.gripper_bin_threshold else 1
        else:
            self.obs_dict["gripper_raw_position"] = None
            self.obs_dict["gripper_action_bin"] = None
            self.obs_dict["gripper_raw_bin"] = None

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            self.obs_dict[cam_key] = cam.read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        self._prev_observation = self.obs_dict

        return self.obs_dict

    def tcp_to_ee_pose(self, tcp_pose, tcp_offset):
        T_tcp = np.eye(4)
        T_tcp[:3,:3] = R.from_rotvec(tcp_pose[3:]).as_matrix()
        T_tcp[:3,3] = tcp_pose[:3]

        T_off = np.eye(4)
        T_off[:3,:3] = R.from_rotvec(tcp_offset[3:]).as_matrix()
        T_off[:3,3] = tcp_offset[:3]

        T_ee = T_tcp @ np.linalg.inv(T_off)

        ee_pos = T_ee[:3,3]
        ee_rot = R.from_matrix(T_ee[:3,:3]).as_rotvec()
        return np.concatenate([ee_pos, ee_rot])

    def _pose_euler(self, pose: list[float] | np.ndarray) -> np.ndarray:
        pose = np.array(pose, dtype=float)
        pose_euler = np.zeros(6, dtype=float)
        pose_euler[:3] = pose[:3]
        pose_euler[3:] = R.from_rotvec(pose[3:]).as_euler("xyz")
        return pose_euler

    def _relative_pose_euler(self, pose: list[float] | np.ndarray) -> np.ndarray:
        if self._episode_reference_ee_pose is None:
            raise RuntimeError("Episode reference EE pose is not set. Call set_episode_reference_pose() first.")

        reference_transform = self._pose_to_transform(self._episode_reference_ee_pose)
        current_transform = self._pose_to_transform(pose)
        relative_transform = np.linalg.inv(reference_transform) @ current_transform

        relative_pose = np.zeros(6, dtype=float)
        relative_pose[:3] = relative_transform[:3, 3]
        relative_pose[3:] = R.from_matrix(relative_transform[:3, :3]).as_euler("xyz")
        return relative_pose

    def set_episode_reference_pose(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.control_space not in ("tcp_force", "tcp_position"):
            return

        self._episode_reference_ee_pose = np.array(self.get_ee_pose(), dtype=float)
        logger.info(f"Set episode reference EE pose: {self._episode_reference_ee_pose.tolist()}")
    
    def stop_force(self):
        self._arm["rtde_c"].forceMode(self.task_frame,[0, 0, 0, 0, 0, 0],np.array([0, 0, 0, 0, 0, 0]),self.type,self.config.force_limit)
        
    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._arm is not None:
            self._arm["rtde_c"].forceMode(self.task_frame,[0, 0, 0, 0, 0, 0],np.array([0, 0, 0, 0, 0, 0]),self.type,self.config.force_limit)
            self._arm["rtde_c"].disconnect()
            self._arm["rtde_r"].disconnect()

        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info(f"[INFO] ===== All {self.name} connections have been closed =====")

    def calibrate(self) -> None:
        pass

    def is_calibrated(self) -> bool:
        return self.is_connected
    
    def configure(self) -> None:
        pass

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
           cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value
    
    
