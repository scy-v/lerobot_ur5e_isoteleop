from setuptools import setup, find_packages
from pathlib import Path

# ====== Project root ======
ROOT = Path(__file__).parent.resolve()

setup(
    name="lerobot_ur5e_isoteleop",
    version="0.1.0",
    description="UR5e teleoperation and dataset collection utilities",
    python_requires=">=3.10",
    packages=find_packages(where=".", include=["scripts*", "scripts.*"]),
    include_package_data=True,
    install_requires=[
        f"lerobot_robot_ur5e @ file:///{ROOT}/lerobot_robot_ur5e",
        f"lerobot_teleoperator_ur5e @ file:///{ROOT}/lerobot_teleoperator_ur5e"
    ],
    scripts=[
        "scripts/tools/map_gripper.sh",
        "scripts/tools/check_master_port.sh",
    ],
    entry_points={
        "console_scripts": [
            # core commands
            "ur5e-record = scripts.core.run_record:main",
            "ur5e-replay = scripts.core.run_replay:main",
            "ur5e-visualize = scripts.core.run_visualize:main",
            # utils commands (data utilities)
            "utils-joint-offsets = scripts.utils.teleop_joint_offsets:main",

            # tools commands (helper tools)
            "tools-check-dataset = scripts.tools.check_dataset_info:main",
            "tools-check-rs = scripts.tools.rs_devices:main",

            # test commands (testing scripts)
            "test-gripper-ctrl = scripts.test.gripper_ctrl:main",
            # unified help command
            "ur5e-help = scripts.help.help_info:main",
        ]
    },
)
