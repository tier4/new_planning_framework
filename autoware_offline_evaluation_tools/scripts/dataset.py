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
            current_gt = None
            candidate_list = []
            for _, row in df.iterrows():
                tag = str(row.iloc[0]).strip().lower()
                data = row.iloc[1:].values.astype(np.float32)
                if data.size < seq_len * (metric_num + 3):
                    continue
                traj = data[: seq_len * (metric_num + 3)].reshape(seq_len, metric_num + 3)
                mets = traj[:, 3 : 3 + metric_num]
                if tag == "ground_truth":
                    if current_gt is not None and len(candidate_list) > 0:
                        if len(candidate_list) == 1:
                            candidate_tensor = torch.tensor(
                                candidate_list[0], dtype=torch.float32
                            ).unsqueeze(0)
                        else:
                            candidate_tensor = torch.tensor(candidate_list, dtype=torch.float32)
                        self.samples.append(
                            (torch.tensor(current_gt, dtype=torch.float32), candidate_tensor)
                        )
                    current_gt = mets
                    candidate_list = []
                elif tag == "candidate":
                    if current_gt is None:
                        continue
                    candidate_list.append(mets)
                else:
                    continue
            if current_gt is not None and len(candidate_list) > 0:
                if len(candidate_list) == 1:
                    candidate_tensor = torch.tensor(
                        candidate_list[0], dtype=torch.float32
                    ).unsqueeze(0)
                else:
                    candidate_tensor = torch.tensor(candidate_list, dtype=torch.float32)
                self.samples.append(
                    (torch.tensor(current_gt, dtype=torch.float32), candidate_tensor)
                )

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        return self.samples[idx]
