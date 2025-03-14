import torch
import yaml


def load_parameters(param_file: str):
    with open(param_file, "r") as f:
        params = yaml.safe_load(f)
    ros_params = params["/**"]["ros__parameters"]
    score_weight = ros_params["score_weight"]
    time_decay_weight = ros_params["time_decay_weight"]
    sample_num = ros_params["sample_num"]
    metrics = ros_params["metrics"]
    return score_weight, time_decay_weight, sample_num, metrics


def get_device(use_cuda=True):
    return torch.device("cuda" if use_cuda and torch.cuda.is_available() else "cpu")


def train_model(model, train_loader, optimizer, loss_fn, num_epochs, device):
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0.0
        batch_count = 0
        for ground_truth, candidate in train_loader:
            ground_truth = ground_truth.to(device)
            candidate = candidate.to(device)
            optimizer.zero_grad()
            score_gt = model(ground_truth)
            score_cand = model(candidate)
            target = torch.ones_like(score_gt).to(device)
            loss = loss_fn(score_gt, score_cand, target)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            batch_count += 1
        if batch_count > 0:
            print(f"Epoch {epoch+1}/{num_epochs}, Avg Loss: {epoch_loss / batch_count:.4f}")
        else:
            print(f"Epoch {epoch+1}/{num_epochs}, No batches processed.")


def test_model(model, test_loader, margin, device):
    model.eval()
    total_samples = 0
    correct = 0
    margins = []
    with torch.no_grad():
        for ground_truth, candidate in test_loader:
            ground_truth = ground_truth.to(device)
            candidate = candidate.to(device)
            score_gt = model(ground_truth)
            score_cand = model(candidate)
            diff = score_gt - score_cand
            correct += (diff >= margin).sum().item()
            total_samples += diff.size(0)
            margins.append(diff.mean().item())
    avg_margin = sum(margins) / len(margins) if margins else 0.0
    ranking_accuracy = correct / total_samples if total_samples > 0 else 0.0
    print(f"Test Average Margin: {avg_margin:.4f}")
    print(f"Ranking Accuracy: {ranking_accuracy:.4f}")
    return avg_margin, ranking_accuracy
