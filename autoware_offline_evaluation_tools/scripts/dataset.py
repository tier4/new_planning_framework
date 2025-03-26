import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset


class TrajectoryDataset(Dataset):
    """
    Reads multiple CSV files and holds all data in memory.

    Each CSV file is assumed to have a header, with the first column being 'tag',
    followed by data points consisting of 3 elements + n metrics each (x, y, yaw, m_1 through m_n).
    Using the first 180 values (seq_len points * (3+n) elements) of each row, the data is reshaped,
    and the metrics portion (m_1 through m_n) is extracted.
    """

    def __init__(self, csv_files, seq_len: int, metric_num: int):
        self.samples = []
        for csv_file in csv_files:
            df = pd.read_csv(csv_file, header=0)
            current_gt_metrics = None
            current_gt_pose = None
            candidate_list = []
            for _, row in df.iterrows():
                tag = str(row.iloc[0]).strip().lower()
                data = row.iloc[1:].values.astype(np.float32)
                if data.size < seq_len * (metric_num + 3):
                    continue
                traj = data[: seq_len * (metric_num + 3)].reshape(seq_len, metric_num + 3)
                pose = traj[:, :2]  # Just use [x,y] position
                metrics = traj[:, 3 : 3 + metric_num]
                if tag == "ground_truth":
                    if current_gt_metrics is not None and len(candidate_list) > 0:
                        if len(candidate_list) == 1:
                            cand_metrics, cand_pose = candidate_list[0]
                            cand_metrics = torch.tensor(
                                cand_metrics, dtype=torch.float32
                            ).unsqueeze(0)
                            cand_pose = torch.tensor(cand_pose, dtype=torch.float32).unsqueeze(0)
                        else:
                            cand_metrics = torch.tensor(
                                [x[0] for x in candidate_list], dtype=torch.float32
                            )
                            cand_pose = torch.tensor(
                                [x[1] for x in candidate_list], dtype=torch.float32
                            )
                        self.samples.append(
                            (
                                torch.tensor(current_gt_metrics, dtype=torch.float32),
                                torch.tensor(current_gt_pose, dtype=torch.float32),
                                cand_metrics,
                                cand_pose,
                            )
                        )
                    current_gt_metrics = metrics
                    current_gt_pose = pose
                    candidate_list = []
                elif tag == "candidate":
                    if current_gt_metrics is None:
                        continue
                    candidate_list.append((metrics, pose))
                else:
                    raise ValueError("Invalid tag in dataset")

            # Assuming the last row finishes with candidate tag
            if current_gt_metrics is not None and len(candidate_list) > 0:
                if len(candidate_list) == 1:
                    cand_metrics, cand_pose = candidate_list[0]
                    cand_metrics = torch.tensor(cand_metrics, dtype=torch.float32).unsqueeze(0)
                    cand_pose = torch.tensor(cand_pose, dtype=torch.float32).unsqueeze(0)
                else:
                    cand_metrics = torch.tensor([x[0] for x in candidate_list], dtype=torch.float32)
                    cand_pose = torch.tensor([x[1] for x in candidate_list], dtype=torch.float32)
                self.samples.append(
                    (
                        torch.tensor(current_gt_metrics, dtype=torch.float32),
                        torch.tensor(current_gt_pose, dtype=torch.float32),
                        cand_metrics,
                        cand_pose,
                    )
                )

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        # Each sample returns a tuple of (gt_metrics, gt_pose, cand_metrics, cand_pose).
        return self.samples[idx]
