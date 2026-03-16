import logging
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from pathlib import Path
import yaml
logging.basicConfig(level=logging.INFO)

class EpisodeSampler(torch.utils.data.Sampler):
    """Sampler to iterate only over frames of a single episode"""
    def __init__(self, dataset: LeRobotDataset, episode_index: int):
        # Get start and end indices of the episode
        from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
        to_idx = dataset.meta.episodes["dataset_to_index"][episode_index]
        self.frame_ids = range(from_idx, to_idx)

    def __iter__(self):
        return iter(self.frame_ids)

    def __len__(self):
        return len(self.frame_ids)

def check_dataset(repo_id: str):
    logging.info("Loading dataset metadata only")
    # Load dataset metadata without loading full data
    dataset = LeRobotDataset(repo_id, episodes=None)

    num_episodes = len(dataset.meta.episodes["dataset_from_index"])
    logging.info(f"Found {num_episodes} episodes in dataset {repo_id}")

    for episode_index in range(num_episodes):
        print(f"\nChecking episode {episode_index + 1}/{num_episodes} ...")
        try:
            # Create sampler and dataloader for the current episode
            sampler = EpisodeSampler(dataset, episode_index)
            dataloader = torch.utils.data.DataLoader(
                dataset, batch_size=1, sampler=sampler
            )

            # Iterate through frames using next()
            it = iter(dataloader)
            frame_idx = 0
            while True:
                try:
                    batch = next(it)
                    frame_idx += 1
                    if frame_idx % 10 == 0:
                        print(f"  Processed {frame_idx}/{len(sampler)} frames...", end="\r")
                except StopIteration:
                    break  # End of current episode
            print(f"Episode {episode_index} OK, total frames: {frame_idx}")
        except Exception as e:
            # Print error if any frame fails
            print(f"Error in episode {episode_index} at frame {frame_idx}: {e}")

def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # Set dataset repo ID here
    repo_id = cfg["check_dataset"]["dataset_name"]
    check_dataset(repo_id)