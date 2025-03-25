import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset


class TrajectoryDataset(Dataset):
    """
    Reads multiple CSV files and holds all data in memory.
    Each CSV file is assumed to have a header, with the first column being 'tag',
    followed by data points consisting of 9 elements each (x, y, yaw, m1 through m6).

    Using the first 180 values (20 points * 9 elements) of each row, the data is reshaped, 
    and the metrics portion (m1 through m6) is extracted.

    Pairs are created by treating the first row as 'ground_truth' and the second row as 'candidate'.
    The candidate's lateral acceleration (m1) is replaced with the corresponding values from the ground_truth.
    """

    def __init__(self, csv_files):
        self.trajectories = []
        for csv_file in csv_files:
            try:
                df = pd.read_csv(csv_file, header=0)
            except pd.errors.EmptyDataError:
                print(f"Cannot read {csv_file} as it is empty")
            # To-do(go-sakayori): delete when dataset format is fixed
            if "Unnamed: 181" in df.columns:
                df = df.drop(columns=["Unnamed: 181"])

            df = df.iloc[:, 1:]  # Remove the leading tag column
            num_points = 20
            num_cols_per_point = 9
            total_expected = num_points * num_cols_per_point  # 180
            for idx, row in df.iterrows():
                row_values = row.values.astype(np.float32)[:total_expected]
                if row_values.size < total_expected:
                    continue
                values = row_values.reshape(num_points, num_cols_per_point)
                mets = values[:, 3:9]  # Metrics part（m1～m6）
                self.trajectories.append(mets)
        self.pairs = [(self.trajectories[i], self.trajectories[i+1])
                      for i in range(0, len(self.trajectories), 2)]

    def __len__(self):
        return len(self.pairs)

    def __getitem__(self, idx):
        gt_metrics_np, cand_metrics_np = self.pairs[idx]
        gt_metrics = torch.tensor(gt_metrics_np, dtype=torch.float32)
        cand_metrics = torch.tensor(cand_metrics_np, dtype=torch.float32)
        return gt_metrics, cand_metrics
