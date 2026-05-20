from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_teleoperator_ur5e")
@dataclass
class UR5eTeleopConfig(TeleoperatorConfig):
    port: str
    use_gripper: bool
    hardware_offsets: list[float]
    joint_ids: list[int]
    joint_offsets: list[float]
    joint_signs: list[int]
    gripper_config: tuple[int, float, float]
    control_mode: str = "isoteleop"
    control_space: str = "joint_to_tcp_force"
    tcp_force_reference_frame: str = "base"
    tcp_position_reference_frame: str = "base"
    robot_urdf_path: str = "assets/urdf/ur5e.urdf"
