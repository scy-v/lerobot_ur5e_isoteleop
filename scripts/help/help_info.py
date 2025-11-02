def main():
    print("""
==================================================
 UR5e Teleoperation Utilities - Command Reference
==================================================

Core Commands:
  ur5e-record           Record teleoperation dataset
  ur5e-replay           Replay a recorded dataset
  ur5e-visualize        Visualize recorded dataset

Utility Commands:
  utils-joint-offsets   Compute joint offsets for teleoperation

Tool Commands:
  tools-check-dataset   Check local dataset information
  tools-check-rs        Retrieve connected RealSense camera serial numbers

Shell Tools:
  map_gripper.sh        Map Gripper Serial Port
  check_master_port.sh  Get the Master Arm's Persistent Serial Identifier

Test Commands:
  test-gripper-ctrl     Run gripper control command (operate the gripper)

--------------------------------------------------
 Tip: Use 'ur5e-help' anytime to see this summary.
==================================================
""")
