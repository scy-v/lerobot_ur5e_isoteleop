# UR5e Master-Slave Teleoperation Data Collection
## 1. Environment Setup
### 1.1 Create and Activate Conda Environment
```bash
conda create -n ur_data python=3.10
conda activate ur_data
```

### 1.2 Install lerobot
```bash
# Install specific version 0.3.4
# git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
git clone https://github.com/huggingface/lerobot.git
pip install -e lerobot
```

### 1.3 Clone and Install UR5e Teleoperation Control Source
```bash
mkdir ur_data_collection && cd ur_data_collection
git clone https://github.com/scy-v/lerobot_ur5e_isoteleop.git
cd lerobot_ur5e_isoteleop
pip install -e lerobot_robot_ur5e
pip install -e lerobot_teleoperator_ur5e
```

## 2. Obtain and Configure Necessary Parameters

### 2.1 Get RealSense Camera Serial Number
```bash
python scripts/rs_devices.py
```

### 2.2 Map Gripper Serial Port
For example, map the gripper to `/dev/ur5e_left_gripper` and make sure only one gripper USB device is connected:
```bash
bash scripts/map_gripper.sh <gripper_port_name>
```
Next, enter the mapped device into the `gripper_port` field in `cfg.yaml`.

### 2.3 Get the Master Arm's Persistent Serial Identifier
Before running this, make sure only one USB device for the master arm is connected:
```bash
bash scripts/check_master_port.sh
```
Next, enter the obtained serial identifier into the `port` field in `cfg.yaml`.

### 2.4 Obtain Master-Slave Joint Angle Offset Circle
> **⚠️ WARNING: Before recording data, you must complete this step to set the correct joint offset. Otherwise, the slave arm may move unexpectedly.**


Manually move the master arm so that its joint angles roughly match the current joint angles of the slave arm. This is done to calculate the joint angle offset cycle between the master and slave arms. Then run:
```bash
python scripts/teleop_joint_offsets.py
```
Copy the resulting `joint_offsets` into the corresponding fields in `cfg.yaml`.

## 3. Dataset Recording

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
1. Turn on **Remote Control** on the UR5e teach pendant.  
2. Verify that the parameters in `cfg.yaml` are correct.  
3. To ensure standardized data collection, please read `9. Dataset Naming and Recording Details` before running the following command:
```bash
python scripts/run_record.py
```
![record_image](assets/record.png)

## 4. Dataset Replay
```bash
python scripts/run_replay.py # Make sure cfg.yaml is properly configured
```

## 5. Dataset Visualization
```bash
python scripts/run_visualize.py # Make sure cfg.yaml is properly configured
```
![record_image](assets/visualize.png)

## 6. Dataset Appending and Resume
If you have already recorded a dataset under the specified `repo_id`, set `resume: True` in `cfg.yaml` and enter the dataset name in `resume_dataset` to continue recording new data on the existing dataset. Then, run the following command:

```bash
python scripts/run_record.py
```

## 7. Merge Datasets
If you recorded data in multiple stages, ensure each dataset has a unique `repo_id`. Use the following command to merge them into a single dataset:
```bash
lerobot-edit-dataset 
    --repo_id <merged_repo_id> 
    --operation.type merge 
    --operation.repo_ids "['<repo_id_1>', '<repo_id2>']"
```

## 8. Recording Control Keys

1. **Right Arrow Key**  
   1. Press the right arrow key: end the current episode and save the data. The program will pause.  
   2. Enter the **reset** phase: press any key to continue (the slave arm will follow the master arm), use the master arm to return the slave arm to its initial position.  
   3. Press the right arrow key again to start recording the next episode.

2. **Left Arrow Key**  
   - Press to restart the current episode recording (overwrite the current data).

3. **Esc Key**  
   - Press to exit the recording session and save all recorded episodes.

## 9. Dataset Naming and Recording Details
### 1. Dataset Naming
1. Datasets are saved by default in `~/.cache/huggingface/lerobot`.
2. Dataset names follow the format `[description]_[date]_[version]`, where:
   - `description` comes from `cfg.yaml` as `repo_id=<user_name>/<description>`;
   - `date` is generated automatically;
   - `version` is incremented automatically if a dataset with the same `repo_id` already exists.
3. Description naming rule: `task.description -> Verb_SourceObject_prep_TargetObject_additionalInfo`.  
   For example:  
   `Pick up the green cube and put it into the trash bin -> pick_greencube_into_trashbin`.

### 2. Recording Instructions
1. Complete step `2. Get and configure required parameters`.
2. Fill in the task instruction in `cfg.yaml` under `task.description` and set `repo_id` according to the dataset naming rules.
3. Check and adjust other parameters in `cfg.yaml` to ensure correctness.
4. Run `python scripts/run_record.py`. For safety, ensure the master and slave arm joint angles are approximately aligned; `run_record` will automatically perform the `2.4 Master-Slave Joint Angle Error Check`. Then follow `8. Recording Control Keys` to complete the recording.
5. After recording, a `dataset_info.txt` file will be automatically created in the same directory to store local dataset information. If you have manually deleted any datasets, update the `dataset_info.txt` using:
```bash
python scripts/check_dataset_info.py
```