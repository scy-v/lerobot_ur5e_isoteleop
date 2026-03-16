from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("ur5e_robot")
@dataclass
class UR5eConfig(RobotConfig):
    use_gripper: bool = True
    gripper_reverse: bool = True
    robot_ip: str = "192.168.1.184"
    gripper_port: str = "/dev/ur5e_left_gripper"
    gripper_bin_threshold: float = 0.98
    debug: bool = True
    close_threshold: float = 0.7
    robot_urdf_path: str = "assets/urdf/ur5e.urdf"
    kp: int = 3000
    kd: int = 20
    kp_rot: int = 4000
    kd_rot: int = 800
    rtde_freq: int = 125
    select_vector: list = field(default_factory=lambda: [1, 1, 1, 1, 1, 1]) 
    force_limit: list = field(default_factory=lambda: [2, 2, 2, 2, 2, 2]) 
    look_ahead_time: int = 0.2
    dt: int = 0.002
    gain: int = 100
    pos_delta: int = 0.2
    vel_delta: int = 0.4
    gain_scale: int = 1.5
    control_space: str="force" # "joint" or "force"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
