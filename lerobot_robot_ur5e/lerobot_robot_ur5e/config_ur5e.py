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
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
