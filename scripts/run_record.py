import yaml
from pathlib import Path
from typing import Dict, Any
from dataset_utils import generate_dataset_name, update_dataset_info
from lerobot_robot_ur5e import UR5eConfig, UR5e
from lerobot_teleoperator_ur5e import UR5eTeleopConfig, UR5eTeleop
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.processor import make_default_processors
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener
import shutil
import termios, sys
from lerobot.utils.constants import HF_LEROBOT_HOME
from teleop_joint_offsets import get_start_joints, compute_joint_offsets
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.utils.control_utils import sanity_check_dataset_robot_compatibility
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")

class RecordConfig:
    def __init__(self, cfg: Dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        time = cfg["time"]
        cam = cfg["cameras"]
        robot = cfg["robot"]
        teleop = cfg["teleop"]
        dxl_cfg = teleop["dynamixel_config"]

        # global config
        self.repo_id: str = cfg["repo_id"]
        self.fps: str = cfg.get("fps", 15)
        self.dataset_path: str = HF_LEROBOT_HOME / self.repo_id

        # teleop config
        self.port = dxl_cfg["port"]
        self.use_gripper = dxl_cfg["use_gripper"]  
        self.joint_ids = dxl_cfg["joint_ids"]
        self.joint_offsets = dxl_cfg["joint_offsets"]
        self.joint_signs = dxl_cfg["joint_signs"]
        self.gripper_config = dxl_cfg["gripper_config"]
        self.control_mode = teleop.get("control_mode", "isoteleop")
        # robot config
        self.robot_ip: str = robot["ip"]
        self.gripper_port: str = robot["gripper_port"]
        self.use_gripper: str = robot["use_gripper"]
        self.gripper_reverse: str = robot["gripper_reverse"]

        # task config
        self.num_episodes: int = task.get("num_episodes", 1)
        self.display: bool = task.get("display", True)
        self.task_description: str = task.get("description", "default task")
        self.resume: bool = task.get("resume", "False")
        self.resume_dataset: str = task["resume_dataset"]

        # time config
        self.episode_time_sec: int = time.get("episode_time_sec", 60)
        self.reset_time_sec: int = time.get("reset_time_sec", 10)
        self.save_mera_period: int = time.get("save_mera_period", 1)

        # cameras config
        self.wrist_cam_serial: str = cam["wrist_cam_serial"]
        self.exterior_cam_serial: str = cam["exterior_cam_serial"]

        # storage config
        self.push_to_hub: bool = storage.get("push_to_hub", False)


def check_joint_offsets(record_cfg: RecordConfig):
    """Check the joint_offsets is set and correct."""

    if record_cfg.joint_offsets is None:
        raise ValueError("joint_offsets is None. Please check teleop_joint_offsets.py output.")

    start_joints = get_start_joints(record_cfg)
    if start_joints is None:
        raise RuntimeError("Failed to retrieve start joints from UR5e robot.")

    joint_offsets = compute_joint_offsets(record_cfg, start_joints)

    if joint_offsets != record_cfg.joint_offsets:
        raise ValueError(
            f"Computed joint_offsets {joint_offsets} != provided joint_offsets {record_cfg.joint_offsets}. "
            "Please check teleop_joint_offsets.py output."
        )
    logging.info("Joint offsets verified successfully.")
    
def main(record_cfg: RecordConfig):
    try:
        # Check joint offsets
        check_joint_offsets(record_cfg)
        
        dataset_name, data_version = generate_dataset_name(record_cfg)
        
        # Create RealSenseCamera configurations
        wrist_image_cfg = RealSenseCameraConfig(serial_number_or_name=record_cfg.wrist_cam_serial,
                                        fps=record_cfg.fps,
                                        width=640,
                                        height=480,
                                        color_mode=ColorMode.RGB,
                                        use_depth=False,
                                        rotation=Cv2Rotation.NO_ROTATION)

        exterior_image_cfg = RealSenseCameraConfig(serial_number_or_name=record_cfg.exterior_cam_serial,
                                        fps=record_cfg.fps,
                                        width=640,
                                        height=480,
                                        color_mode=ColorMode.RGB,
                                        use_depth=False,
                                        rotation=Cv2Rotation.NO_ROTATION)

        # Create the robot and teleoperator configurations
        camera_config = {"wrist_image": wrist_image_cfg, "exterior_image": exterior_image_cfg}
        teleop_config = UR5eTeleopConfig(        
            port=record_cfg.port,
            use_gripper=record_cfg.use_gripper,
            joint_ids=record_cfg.joint_ids,
            joint_offsets=record_cfg.joint_offsets,
            joint_signs=record_cfg.joint_signs,
            gripper_config=record_cfg.gripper_config,
            control_mode=record_cfg.control_mode)
        
        robot_config = UR5eConfig(
            robot_ip=record_cfg.robot_ip,
            gripper_port=record_cfg.gripper_port,
            cameras = camera_config,
            use_gripper = record_cfg.use_gripper,
            gripper_reverse = record_cfg.gripper_reverse
        )
        # Initialize the robot and teleoperator
        robot = UR5e(robot_config)
        teleop = UR5eTeleop(teleop_config)

        # Configure the dataset features
        action_features = hw_to_dataset_features(robot.action_features, "action")
        obs_features = hw_to_dataset_features(robot.observation_features, "observation", use_video=True)
        dataset_features = {**action_features, **obs_features}

        if record_cfg.resume:
            dataset = LeRobotDataset(
                dataset_name,
            )

            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer()
            sanity_check_dataset_robot_compatibility(dataset, robot, record_cfg.fps, dataset_features)
        else:
            # # Create the dataset
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                fps=record_cfg.fps,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
            )
        # Set the episode metadata buffer size to 1, so that each episode is saved immediately
        dataset.meta.metadata_buffer_size = record_cfg.save_mera_period

        # Initialize the keyboard listener and rerun visualization
        _, events = init_keyboard_listener()
        init_rerun(session_name="recording")

        # Create processor
        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

        robot.connect()
        teleop.connect()

        episode_idx = 0

        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            logging.info(f"====== [RECORD] Recording episode {episode_idx + 1} of {record_cfg.num_episodes} ======")
            record_loop(
                robot=robot,
                events=events,
                fps=record_cfg.fps,
                teleop=teleop,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                control_time_s=record_cfg.episode_time_sec,
                single_task=record_cfg.task_description,
                display_data=record_cfg.display,
            )

            if events["rerecord_episode"]:
                logging.info("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()

            # Reset the environment if not stopping or re-recording
            if not events["stop_recording"] and (episode_idx < record_cfg.num_episodes - 1 or events["rerecord_episode"]):
                input("====== [WAIT] Press any key to reset the environment======")
                logging.info("====== [RESET] Resetting the environment ======")
                record_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    teleop=teleop,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    control_time_s=record_cfg.reset_time_sec,
                    single_task=record_cfg.task_description,
                    display_data=record_cfg.display,
                )

            episode_idx += 1

        # Clean up
        logging.info("Stop recording")
        robot.disconnect()
        teleop.disconnect()
        dataset.finalize()

        update_dataset_info(record_cfg, dataset_name, data_version)
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except Exception as e:
        dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
        if dataset_path.exists():
            print(f"====== [WARNING] Detected an incomplete dataset folder: {dataset_path} ======")
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
            ans = input("Do you want to delete it? (y/n): ").strip().lower()
            if ans == "y":
                print(f"====== [DELETE] Removing folder: {dataset_path} ======")
                shutil.rmtree(dataset_path, ignore_errors=True)  # Delete only this specific dataset folder
                print("====== [DONE] Incomplete dataset folder deleted successfully. ======")
            else:
                print("====== [KEEP] Incomplete dataset folder retained, please check manually. ======")



if __name__ == "__main__":
    with open(Path(__file__).parent / "config" / "cfg.yaml", 'r') as f:
        cfg = yaml.safe_load(f)

    record_cfg = RecordConfig(cfg["record"])
    main(record_cfg)