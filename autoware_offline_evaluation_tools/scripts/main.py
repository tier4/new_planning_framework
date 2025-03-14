import argparse
import glob
import torch
import torch.optim as optim
from torch.utils.data import DataLoader
import os

from trajectory_selector import TrajectoryNetPerMetric
from dataset import TrajectoryDataset
from utils import load_parameters, get_device, train_model, test_model

def parse_args():
    parser = argparse.ArgumentParser(description="Train and test TrajectoryNet model.")
    parser.add_argument("--train_pattern", type=str, default="data_train*.csv",
                        help="Glob pattern for training CSV files")
    parser.add_argument("--test_pattern", type=str, default="data_test*.csv",
                        help="Glob pattern for test CSV files")
    parser.add_argument("--epochs", type=int, default=200, help="Number of training epochs")
    parser.add_argument("--use_cuda", type=lambda s: s.lower() in ['true', '1', 'yes'], default=True,
                        help="Whether to use CUDA (default: True)")
    parser.add_argument("--use_max_values", type=lambda s: s.lower() in ['true', '1', 'yes'], default=True,
                        help="Whether to use max_values for normalization (default: True)")
    parser.add_argument(
    "--param_file",
    type=str,
    default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/offline_evaluation.param.yaml"),
    help="Path to YAML parameter file"
    ) 
    return parser.parse_args()

def main():
    args = parse_args()
    device = get_device(args.use_cuda)
    print(f"Using device: {device}")

    score_weight, time_decay_weights, sample_num, metrics = load_parameters(args.param_file)
    num_metrics = len(metrics)
    
    # 固定の max_values と lower_better_mask（必要に応じて YAML から読み込む拡張も可能）
    max_values = [100.0, 50.0, 200.0, 10.0, 5.0, 2.0]
    lower_better_mask = [True, True, False, False, True, True]

    model = TrajectoryNetPerMetric(
        num_metrics=num_metrics,
        seq_len=sample_num,
        score_weight=score_weight,
        time_decay_weights=time_decay_weights,
        normalize_score=True,
        max_values=max_values,
        lower_better_mask=lower_better_mask,
        use_max_values=args.use_max_values,
        train_subnet=False
    ).to(device)
    optimizer = optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = torch.nn.MarginRankingLoss(margin=1.0)

    train_csv_files = glob.glob(args.train_pattern)
    train_csv_files.sort()
    train_dataset = TrajectoryDataset(train_csv_files)
    train_loader = DataLoader(train_dataset, batch_size=128, shuffle=True, num_workers=4, pin_memory=True)

    print("Starting training...")
    train_model(model, train_loader, optimizer, loss_fn, args.epochs, device)
    torch.save(model.state_dict(), "trained_model_weights.pth")
    print("Model weights saved to 'trained_model_weights.pth'.")

    test_csv_files = glob.glob(args.test_pattern)
    test_csv_files.sort()
    test_dataset = TrajectoryDataset(test_csv_files)
    test_loader = DataLoader(test_dataset, batch_size=128, shuffle=False, num_workers=4, pin_memory=True)

    print("Starting testing...")
    test_model(model, test_loader, margin=1.0, device=device)

if __name__ == "__main__":
    main()
