from pathlib import Path
from typing import Any, Dict, List
import logging

import numpy as np
import yaml
from rtde_receive import RTDEReceiveInterface

from lerobot_teleoperator_ur5e.dynamixel import DynamixelDriver

np.set_printoptions(suppress=True)

# ------------------------ Logging Setup ------------------------ #
logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


JOINT_NAMES = ["first", "second", "third", "fourth", "fifth", "sixth"]


# ------------------------ Config Loader ------------------------ #
class RecordConfig:
    """Configuration for hardware offset calibration."""

    def __init__(self, cfg: Dict[str, Any]):
        teleop = cfg["teleop"]
        robot = cfg["robot"]
        dxl_cfg = teleop["dynamixel_config"]

        self.port = dxl_cfg["port"]
        self.joint_ids = dxl_cfg["joint_ids"]
        self.joint_signs = dxl_cfg["joint_signs"]
        self.robot_ip: str = robot["ip"]


# ------------------------ Hardware Read Functions ------------------------ #
def get_ur5e_raw_joints(rtde_r: RTDEReceiveInterface) -> np.ndarray:
    """Read raw UR5e joint positions in radians."""
    joint_positions = np.array(rtde_r.getActualQ(), dtype=float)
    if joint_positions.shape[0] < 6:
        raise RuntimeError(f"Expected 6 UR5e joints, got {joint_positions.tolist()}")
    return joint_positions[:6]


def get_dynamixel_raw_joints(driver: DynamixelDriver) -> np.ndarray:
    """Read raw Dynamixel joint positions in radians, without hardware offsets."""
    joint_positions = np.array(driver.get_joints(), dtype=float)
    if joint_positions.shape[0] < 6:
        raise RuntimeError(f"Expected 6 Dynamixel joints, got {joint_positions.tolist()}")
    return joint_positions[:6]


# ------------------------ Offset Calculation ------------------------ #
def compute_hardware_offsets(cfg: RecordConfig) -> List[float]:
    """Compute hardware_offsets with joint direction signs applied.

    The existing Dynamixel driver applies hardware_offsets in degrees, so this
    function prints and returns offsets in degrees.
    """
    logger.info("Connecting to UR5e...")
    rtde_r = RTDEReceiveInterface(cfg.robot_ip)
    logger.info("UR5e connected.")

    logger.info("Connecting to Dynamixel master arm...")
    driver = DynamixelDriver(
        cfg.joint_ids,
        port=cfg.port,
        baudrate=57600,
        use_fake_fallback=False,
    )
    logger.info("Dynamixel connected.\n")

    try:
        # Warm up the background reader so the first prompted read is stable.
        for _ in range(10):
            driver.get_joints()

        hardware_offsets: List[float] = []

        logger.info("Calculating hardware zero offsets.")
        logger.info("Recommended UR joint angles from 1 to 6: [180, -90, 90, -90, -90, 0] deg")
        logger.info("Formula: offset = joint_sign * UR5e raw deg - Dynamixel raw deg\n")

        for i in range(6):
            joint_name = JOINT_NAMES[i]
            input(
                f"Joint {i + 1}: keep master/slave at a similar angle, "
                "then press Enter..."
            )

            ur5e_raw_rad = get_ur5e_raw_joints(rtde_r)
            dxl_raw_rad = get_dynamixel_raw_joints(driver)
            ur5e_raw_deg = np.rad2deg(ur5e_raw_rad)
            dxl_raw_deg = np.rad2deg(dxl_raw_rad)

            joint_sign = cfg.joint_signs[i]
            offset = float(joint_sign * ur5e_raw_deg[i] - dxl_raw_deg[i])
            rounded_offset = round(offset, 3)
            hardware_offsets.append(rounded_offset)

            logger.info(
                "Joint %d (%s): sign %+d, UR5e %.3f deg, DXL %.3f deg -> offset %.3f deg\n",
                i + 1,
                joint_name,
                joint_sign,
                ur5e_raw_deg[i],
                dxl_raw_deg[i],
                rounded_offset,
            )

        logger.info("hardware_offsets: %s", hardware_offsets)
        return hardware_offsets
    finally:
        driver.close()
        if hasattr(rtde_r, "disconnect"):
            rtde_r.disconnect()


def run(record_cfg: RecordConfig) -> List[float]:
    return compute_hardware_offsets(record_cfg)


# ------------------------ Main ------------------------ #
def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)
    record_cfg = RecordConfig(cfg["record"])
    run(record_cfg)
