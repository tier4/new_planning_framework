import torch
import yaml


def load_parameters(param_file):
    with open(param_file, "r") as f:
        params = yaml.safe_load(f)
    ros_params = None
    if "/**:" in params:
        ros_params = params["/**:"]["ros__parameters"]
    elif "ros__parameters" in params:
        ros_params = params["ros__parameters"]
    else:
        for key, value in params.items():
            if isinstance(value, dict) and "ros__parameters" in value:
                ros_params = value["ros__parameters"]
                break
    if ros_params is None:
        raise KeyError("Could not find 'ros__parameters' in the YAML file.")
    score_weight = ros_params.get("score_weight")
    time_decay_weight = ros_params.get("time_decay_weight")
    sample_num = ros_params.get("sample_num")
    metrics = ros_params.get("metrics")
    if score_weight is None or time_decay_weight is None or sample_num is None or metrics is None:
        raise KeyError("Missing one or more required parameters in YAML.")
    return score_weight, time_decay_weight, sample_num, metrics


def get_device(use_cuda=True):
    return torch.device("cuda" if use_cuda and torch.cuda.is_available() else "cpu")


def train_model(model, train_loader, optimizer, loss_fn, num_epochs, device):
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0.0
        batch_count = 0
        for ground_truth, candidates in train_loader:
            ground_truth = ground_truth.to(device)
            candidates = candidates.to(device)
            optimizer.zero_grad()
            loss = loss_fn(model, ground_truth, candidates)
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
        for ground_truth, candidates in test_loader:
            ground_truth = ground_truth.to(device)
            candidates = candidates.to(device)
            if candidates.dim() == 4:
                score_gt = model(ground_truth)
                candidates_reshaped = candidates.view(
                    candidates.size(0) * candidates.size(1), candidates.size(2), candidates.size(3)
                )
                score_candidates = model(candidates_reshaped).view(
                    candidates.size(0), candidates.size(1)
                )
                diff = score_gt - score_candidates[:, 0:1]
            else:
                raise ValueError("Invalid candidate dimensions in testing.")
            correct += (diff >= margin).sum().item()
            total_samples += diff.size(0)
            margins.append(diff.mean().item())
    avg_margin = sum(margins) / len(margins) if margins else 0.0
    ranking_accuracy = correct / total_samples if total_samples > 0 else 0.0
    print(f"Test Average Margin: {avg_margin:.4f}")
    print(f"Ranking Accuracy: {ranking_accuracy:.4f}")
    return avg_margin, ranking_accuracy
