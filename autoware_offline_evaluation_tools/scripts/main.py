import argparse
import glob

import torch
import torch.optim as optim
from dataset import TrajectoryDataset
from multi_loss import pairwise_adaptive_ranking_loss
from torch.utils.data import DataLoader
from trajectory_selector import TrajectoryNetPerMetric
from utils import get_device, load_parameters, test_model, train_model


def print_model_weights(model):
    if model.reparam_weights:
        rep_metric_weights = torch.sigmoid(model.weights).detach().cpu().numpy()
        print("Reparameterized metric weights (in [0,1]):", rep_metric_weights)
    else:
        metric_weights = model.metric_weights.detach().cpu().numpy()
        print("Metric weights:", metric_weights)

    for idx, subnet in enumerate(model.subnets):
        if hasattr(subnet, "reparam_time_weights") and subnet.reparam_time_weights:
            rep_time_weights = torch.sigmoid(subnet.weights).detach().cpu().numpy()
            print(f"Subnet {idx} reparameterized time weights (in [0,1]):", rep_time_weights)
        elif hasattr(subnet, "weights"):
            time_weights = subnet.weights.detach().cpu().numpy()
            print(f"Subnet {idx} time weights (directly):", time_weights)
        else:
            print(f"Subnet {idx} has no time weights attribute")


def parse_args():
    parser = argparse.ArgumentParser(description="Train and test TrajectoryNet model.")
    parser.add_argument(
        "--train_set",
        type=str,
        default="data_train*.csv",
        help="Glob pattern for training CSV files",
    )
    parser.add_argument(
        "--test_set", type=str, default="data_test*.csv", help="Glob pattern for test CSV files"
    )
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    parser.add_argument(
        "--use_cuda",
        type=lambda s: s.lower() in ["true", "1", "yes"],
        default=True,
        help="Whether to use CUDA (default: True)",
    )
    parser.add_argument(
        "--use_max_values",
        type=lambda s: s.lower() in ["true", "1", "yes"],
        default=True,
        help="Whether to use max_values for normalization (default: True)",
    )
    parser.add_argument(
        "--reparam_weights",
        type=lambda s: s.lower() in ["true", "1", "yes"],
        default=True,
        help="Whether to reparameterize metric weights (default: True)",
    )
    parser.add_argument(
        "--reparam_time_weights",
        type=lambda s: s.lower() in ["true", "1", "yes"],
        default=True,
        help="Whether to reparameterize time decay weights (default: True)",
    )
    parser.add_argument(
        "--param_file",
        type=str,
        default="../config/offline_evaluation.param.yaml",
        help="Path to YAML parameter file",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    device = get_device(args.use_cuda)
    print(f"Using device: {device}")

    score_weight, time_decay_weights, sample_num, metrics = load_parameters(args.param_file)
    seq_len = sample_num
    num_metrics = len(metrics)

    max_values = [10.0, 100.0, 200.0, 50.0, 35.0, 0.2]
    lower_better_mask = [True, True, False, False, True, True]

    model = TrajectoryNetPerMetric(
        num_metrics=num_metrics,
        seq_len=seq_len,
        score_weight=score_weight,
        time_decay_weights=time_decay_weights,
        normalize_score=True,
        max_values=max_values,
        lower_better_mask=lower_better_mask,
        use_max_values=args.use_max_values,
        reparam_weights=args.reparam_weights,
        reparam_time_weights=args.reparam_time_weights,
        train_subnet=False,
    ).to(device)
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    # loss_fn を def を使って定義（引数: model, gt_mets, gt_pos, cand_mets, cand_pos）
    def loss_fn(model, gt_mets, gt_pos, cand_mets, cand_pos):
        return pairwise_adaptive_ranking_loss(
            model, gt_mets, gt_pos, cand_mets, cand_pos, margin=1.0
        )

    train_csv_files = glob.glob(args.train_set)
    train_csv_files.sort()
    dataset = TrajectoryDataset(train_csv_files, seq_len, num_metrics)
    train_loader = DataLoader(dataset, batch_size=32, shuffle=True, num_workers=4, pin_memory=True)

    print("Starting training...")
    train_model(model, train_loader, optimizer, loss_fn, args.epochs, device)

    print_model_weights(model)
    torch.save(model.state_dict(), "trained_model_weights.pth")
    print("Model weights saved to 'trained_model_weights.pth'.")

    test_csv_files = glob.glob(args.test_set)
    test_csv_files.sort()
    test_dataset = TrajectoryDataset(test_csv_files, seq_len, num_metrics)
    test_loader = DataLoader(
        test_dataset, batch_size=32, shuffle=False, num_workers=4, pin_memory=True
    )

    print("Starting testing...")
    test_model(model, test_loader, margin=1.0, device=device)


if __name__ == "__main__":
    main()
