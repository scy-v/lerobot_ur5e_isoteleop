import re
from pathlib import Path
from datetime import datetime
from colorama import Fore, Style, init

init(autoreset=True)  

def generate_dataset_name(cfg):
    """
    Generate dataset name: [description]_[YYYYMMDD]_[vXX]
    Determine next version based on existing folders in the dataset root parent.
    Return (dataset_name, version_str).
    """
    if cfg.resume:
        # If resuming, use the provided dataset name directly
        resume_dataset = cfg.resume_dataset
        dataset_name = resume_dataset
        version_str = resume_dataset.split("_")[-1]  # extract version from the name
        return dataset_name, version_str
    else:
        # description extracted from cfg.repo_id (format: "<user>/<description>")
        repo_id = cfg.repo_id  # e.g. "scylearning/pick_greencube_into_trashbin"
        dataset = cfg.dataset_path
        user = repo_id.split("/", 1)[0]
        description = repo_id.split("/", 1)[1]

        # dataset.root points to HF_LEROBOT_HOME / repo_id; parent is datasets storage dir
        root_path = Path(dataset)
        base_path = root_path.parent  # location where all dataset folders are stored

        # list folders that start with the description
        existing = [p.name for p in base_path.iterdir() if p.is_dir() and p.name.startswith(description + "_")]

        # find the largest existing vNN for this description
        max_v = 0
        pattern = re.compile(rf"^{re.escape(description)}_\d{{8}}_v(\d+)$")
        for name in existing:
            m = pattern.match(name)
            if m:
                try:
                    vnum = int(m.group(1))
                    if vnum > max_v:
                        max_v = vnum
                except ValueError:
                    continue

        next_v = max_v + 1
        today_str = datetime.today().strftime("%Y%m%d")
        version_str = f"v{str(next_v).zfill(2)}"
        dataset_name = f"{user}/{description}_{today_str}_{version_str}"

        return dataset_name, version_str


def update_dataset_info(cfg, dataset_name, version_str):
    """
    Append a single-line record to a readme file under info_path.
    Line format:
      <dataset_name>: task="<original task>", date="<YYYY-MM-DD HH:MM:SS>", version="<vXX>"
    Simply append chronologically (no sorting).
    """
    task_description = cfg.task_description
    info_path = Path(cfg.dataset_path).parent
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if cfg.resume:
        type_ = "resumed"
    else:
        type_ = "record"

    info_line = f'name="{dataset_name}", task="{task_description}", date="{now_str}", version="{version_str}", type="{type_}"\n'
    info_file = info_path / "dataset_info.txt"

    # Append directly (create file if not exists)
    with open(info_file, "a") as f:
        f.write(info_line)

