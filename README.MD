# UR5e/UR7e Master-Slave Teleoperation Data Collection
<p align="center">
  <img src="assets/robot.png" alt="Record" width="600">
  <br>
</p>

## 🛠️ 1. Environment Setup
### 1.1 Create and Activate Conda Environment
```bash
conda create -n ur_data python=3.10
conda activate ur_data
```

### 1.2 Install lerobot
```bash
# Install specific version 0.3.4
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
pip install -e .
```

### 1.3 Clone and Install UR5e Teleoperation Control Source
```bash
mkdir ur_data_collection && cd ur_data_collection
git clone https://github.com/scy-v/lerobot_ur5e_isoteleop.git
cd lerobot_ur5e_isoteleop
pip install -r requirements.txt # install all packages in editable mode
```

### 1.4 View Available Commands
Run the following command to display all available scrtips:
```bash
ur5e-help
# ==================================================
#  UR5e Teleoperation Utilities - Command Reference
# ==================================================

# Core Commands:
#   ur5e-record           Record teleoperation dataset
#   ur5e-replay           Replay a recorded dataset
#   ur5e-visualize        Visualize recorded dataset

# Utility Commands:
#   utils-hardware-offsets  Compute hardware offsets between raw UR5e and Dynamixel joints
#   utils-joint-offsets   Compute joint offsets for teleoperation

# Tool Commands:
#   tools-check-dataset   Check local dataset integrity
#   tools-check-info      Check local dataset information
#   tools-check-rs        Retrieve connected RealSense camera serial numbers
#   tools-prune-dataset   Prune episodes from dataset by Episode ID

# Shell Tools:
#   map_gripper.sh        Map Gripper Serial Port
#   check_master_port.sh  Get the Master Arm's Persistent Serial Identifier

# Test Commands:
#   test-gripper-ctrl     Run gripper control command (operate the gripper)

# --------------------------------------------------
#  Tip: Use 'ur5e-help' anytime to see this summary.
# ==================================================
```

## ⚙️ 2. Obtain and Configure Necessary Parameters

### 2.1 Get RealSense Camera Serial Number
Please make sure only one camera is connected at a time.
```bash
tools-check-rs
```

### 2.2 Map Gripper Serial Port
For example, map the gripper to `/dev/ur5e_left_gripper` and make sure only one gripper USB device is connected:
```bash
map_gripper.sh ur5e_left_gripper
```
Next, enter the mapped device into the `gripper_port` field in `cfg.yaml`.

### 2.3 Get the Master Arm's Persistent Serial Identifier
Before running this, make sure only one USB device for the master arm is connected:
```bash
check_master_port.sh
```
Next, enter the obtained serial identifier into the `port` field in `cfg.yaml` and set device access permissions:
```bash
sudo chmod 666 <your_master_port>
```

### 2.4 Obtaining Master–Slave Zero-Offset
> **⚠️ Note: The `hardware_offsets` in `cfg.yaml` store hardware zero-position offsets between the master and slave arm joints, with `joint_signs` handling joint direction.**

Follow the terminal prompts to compute each joint from the first joint onward. Keep the current master/slave joint at approximately the same angle, then press Enter; the script reads the raw UR5e joint angle and the raw unmodified Dynamixel joint angle, then prints that joint's `hardware_offset`:
```bash
utils-hardware-offsets
```
Copy the final `hardware_offsets` output into the corresponding field in `cfg.yaml`.

### 2.5 Obtain Master-Slave Joint Angle Offset Circle
> **⚠️ WARNING: `joint_offsets` align the numeric angle cycle between the master and slave joints; they do not correct hardware zero-position errors. Check them before recording, otherwise the slave arm may move unexpectedly.**


Complete `hardware_offsets` calibration first, then manually move the master arm so that its joint angles roughly match the current joint angles of the slave arm. Then run:
```bash
utils-joint-offsets
```
Copy the resulting `joint_offsets` into the corresponding fields in `cfg.yaml`.

### 2.6 Configure Control Space
Open `scripts/config/cfg.yaml` and check the robot IP, gripper port, camera serial numbers, control mode, and camera resolution.

Key robot fields:
- `control_space`: `"joint"`, `"joint_to_tcp_force"`, `"tcp_force"`, or `"tcp_position"`.
  - `joint`: the master action is joint angles, and the slave arm uses `servoJ` joint-position control.
  - `joint_to_tcp_force`: the master action is joint angles, converted through URDF forward kinematics into a target end-effector pose, and the slave arm follows it with `forceMode`.
  - `tcp_force`: the master action is an end-effector delta, and the slave arm follows it with `forceMode`.
  - `tcp_position`: the master action is an end-effector delta, and the slave arm executes it with `servoL`.
- `tcp_force.reference_frame` / `tcp_position.reference_frame`: `"base"` or `"tcp"`, used only by the corresponding `tcp_force` / `tcp_position` mode.
  - `base`: action delta is expressed in the base frame, and `tcp_pose` observation is the absolute EE/tool0 pose.
  - `tcp`: action delta is expressed in the current end-effector frame, and `tcp_pose` observation is relative to the episode reference pose.
  - To avoid inconsistent coordinates caused by temporary TCP-offset changes on the UR teach pendant, `tcp_pose` is converted from UR TCP pose and TCP offset into EE/tool0 pose.
  - `joint` and `joint_to_tcp_force` do not read `reference_frame`; their `tcp_pose` observation is the absolute EE/tool0 pose.

## 📥 3. Dataset Recording

### 3.1 Upload Data to Hugging Face (Optional)
1. Set the following in `cfg.yaml`:
```yaml
push_to_hub: True
```
2. Obtain your Hugging Face account token and login:
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} 
huggingface-cli whoami  # Verify successful login
```

### 3.2 Start Recording
> **⚠️ Recommended: Set TCP Position to `[0, 0, 0, 0, 0, 0]` on the UR teach pendant to use EE/tool0 as a fixed end-effector frame, avoid inconsistent end-effector frames caused by temporary TCP changes, and support later model training.**

1. Turn on **Remote Control** on the UR5e teach pendant.  
2. Verify that the parameters in `cfg.yaml` are correct.  
3. To ensure standardized data collection, please read `10. Dataset Naming and Recording Details` before running the following command:
```bash
ur5e-record
```

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: Record</b>
</p>

## ▶️ 4. Dataset Replay
```bash
ur5e-replay # Make sure cfg.yaml is properly configured
```

## 📊 5. Dataset Visualization
```bash
ur5e-visualize # Make sure cfg.yaml is properly configured
```

<p align="center">
  <img src="assets/episode_data.png" alt="Episode Data" width="600">
  <br>
  <b>Figure 2: Episode Data Window</b>
</p>

<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>Figure 3: Visualization</b>
</p>

The data window shows the action for each frame and lets you select one observation group at a time, such as `tcp_pose`, `tcp_force`, or joint states.

## ➕ 6. Dataset Appending and Resume
If you have already recorded a dataset under the specified `repo_id`, set `resume: True` in `cfg.yaml` and enter the dataset name in `resume_dataset` to continue recording new data on the existing dataset. Then, run the following command:

```bash
ur5e-record
```

## 🔗 7. Merge Datasets
If you recorded data in multiple stages, ensure each dataset has a unique `repo_id`. Use the following command to merge them into a single dataset:
```bash
lerobot-edit-dataset 
    --repo_id <merged_repo_id> 
    --operation.type merge 
    --operation.repo_ids "['<repo_id_1>', '<repo_id2>']"
```
- For more dataset processing commands, see [LeRobot](https://huggingface.co/docs/lerobot/using_dataset_tools).

## ✅ 8. Dataset Check and Pruning
- Run the following command to verify dataset integrity, especially for missing video frames:
- Note update `check_dataset.dataset_name` in cfg.yaml.
```bash
tools-check-dataset
```
- If missing frames are detected, create a new dataset excluding the affected episode IDs while keeping the original dataset intact:
- Note update `prune_episodes.old/new_dataset_name` in cfg.yaml.
```bash
tools-prune-dataset "[id1,...,idN]"
```
- This tool can remove episodes with missing frames or any specified episodes.
## ⌨️ 9. Recording Control Keys

1. **Right Arrow Key**  
   1. Press the right arrow key: end the current episode and save the data. The program will pause.  
   2. Enter the **reset** phase: press enter key to continue (the slave arm will follow the master arm), use the master arm to return the slave arm to its initial position.  
   3. Press the right arrow key again to start recording the next episode.

2. **Left Arrow Key**  
   - Press to restart the current episode recording (overwrite the current data).

3. **Esc Key**  
   - Press to exit the recording session and save all recorded episodes.

4. **Ctrl+C or Exception**
   - Pressing or on exception: enters exception handling and asks whether to delete the incomplete dataset.
  
## 🗂️ 10. Dataset Naming and Recording Details
### 1. Dataset Naming
<p align="center">
  <img src="assets/dataset.png" alt="dataset" width="600">
  <br>
  <b>Figure 4: Dataset</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="dataset_info" width="600">
  <br>
  <b>Figure 5: Dataset Info</b>
</p>

1. The dataset is by default stored in the `~/.cache/huggingface/lerobot` directory and contains three types of items:

   - `dataset_info.txt`: Automatically records local dataset information, including the following fields: `record_id`, `name`, `task`, `date`, `version`, `user_info`, and `type`. The `user_info` field can be annotated via the `user_notes` entry in `cfg.yaml`.

   - `dataset_info_backup`: When `tools-check-dataset` is used to manually update `dataset_info.txt`, this folder stores backups of the previous records.

   - Dataset folders: Stores the actual dataset contents.

2. Dataset names follow the format `[description]_[date]_[version]`, where:
   - `description` comes from `cfg.yaml` as `repo_id=<user_name>/<description>`;
   - `date` is generated automatically;
   - `version` is incremented automatically if a dataset with the same `repo_id` already exists.
3. Description naming rule: `task.description -> Verb_SourceObject_prep_TargetObject`.  
   For example:  
   `Pick up the green cube and put it into the trash bin -> pick_greencube_into_trashbin`.

### 2. Recording Instructions
1. Complete step `2. Get and configure required parameters`.
2. Fill in the task instruction in `cfg.yaml` under `task.description` and set `repo_id` according to the dataset naming rules.
3. Check and adjust other parameters in `cfg.yaml` to ensure correctness.
4. Run `ur5e-record`. For safety, ensure the master and slave arm joint angles are approximately aligned; `ur5e-record` will automatically perform the `2.4 Master-Slave Joint Angle Error Check`. Then follow `8. Recording Control Keys` to complete the recording.
5. After recording, a `dataset_info.txt` file will be automatically created in the same directory to store local dataset information. If you have manually deleted any datasets, update the `dataset_info.txt` using:
```bash
tools-check-info
```
