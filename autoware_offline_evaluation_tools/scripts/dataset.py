import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset

class TrajectoryDataset(Dataset):
    """
    複数のCSVファイルを読み込み、全データをメモリに保持します。
    各CSVファイルは、ヘッダーがあり、先頭列が tag、
    その後に各点の9要素（x, y, yaw, m1～m6）が記録されていると仮定します。
    
    各行の最初180個の値（20点×9要素）を用いてリシェイプし、メトリクス部分（m1～m6）を抽出します。
    ペアは、1行目を ground_truth、2行目を candidate として作成し、
    candidate の lateral acceleration (m1) は ground_truth の値に置き換えます。
    """
    def __init__(self, csv_files):
        self.trajectories = []  # 各行のメトリクス部分を保存するリスト
        for csv_file in csv_files:
            df = pd.read_csv(csv_file, header=0)
            if "Unnamed: 181" in df.columns:
                df = df.drop(columns=["Unnamed: 181"])
            # 先頭の tag 列を除去 (to-do:これをなくすようにデータを作成)
            df = df.iloc[:, 1:]
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
        # ペアは、連続する2行を (ground_truth, candidate) として作成
        self.pairs = [(self.trajectories[i], self.trajectories[i+1]) 
                      for i in range(0, len(self.trajectories), 2)]
    
    def __len__(self):
        return len(self.pairs)
    
    def __getitem__(self, idx):
        gt_metrics_np, cand_metrics_np = self.pairs[idx]
        gt_metrics = torch.tensor(gt_metrics_np, dtype=torch.float32)
        cand_metrics = torch.tensor(cand_metrics_np, dtype=torch.float32)
        return gt_metrics, cand_metrics
