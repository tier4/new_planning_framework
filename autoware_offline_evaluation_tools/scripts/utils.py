import torch
import yaml


def average_displacement_error(ground_truth, candidate):
    """
    Compute average displacement error.

    ground_truth: (B,T,D)
    candidate: (B,T,D)
    Returns: (B,1)
    """
    diff = candidate - ground_truth
    error = torch.norm(diff, p=2, dim=-1)
    return error.mean(dim=1, keepdim=True)


def final_displacement_error(ground_truth, candidate):
    """
    Compute final displacement error.

    ground_truth: (B,T,D)
    candidate: (B,T,D)
    Returns: (B,1)
    """
    diff = candidate - ground_truth
    error = torch.norm(diff, p=2, dim=-1)
    return error[:, -1].unsqueeze(1)


def load_parameters(param_file):
    with open(param_file, "r") as f:
        params = yaml.safe_load(f)
    ros_params = None
    if "/**:" in params:
        ros_params = params["/**:"]["ros__parameters"]
    elif "ros__parameters" in params:
        ros_params = params["ros__parameters"]
    else:
        for _, value in params.items():
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
        for gt_metrics, gt_pose, cand_metrics, cand_pose in train_loader:
            gt_metrics = gt_metrics.to(device)
            gt_pose = gt_pose.to(device)
            cand_metrics = cand_metrics.to(device)
            cand_pose = cand_pose.to(device)
            optimizer.zero_grad()
            loss = loss_fn(model, gt_metrics, gt_pose, cand_metrics, cand_pose)
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
        for gt_metrics, gt_pose, cand_metrics, cand_pose in test_loader:
            gt_metrics = gt_metrics.to(device)
            gt_pose = gt_pose.to(device)
            cand_metrics = cand_metrics.to(device)
            cand_pose = cand_pose.to(device)

            if cand_metrics.dim() == 4:
                score_gt = model(gt_metrics)
                candidates_reshaped = cand_metrics.view(
                    cand_metrics.size(0) * cand_metrics.size(1),
                    cand_metrics.size(2),
                    cand_metrics.size(3),
                )
                score_candidates = model(candidates_reshaped).view(
                    cand_metrics.size(0), cand_metrics.size(1)
                )
                diff = score_gt - score_candidates[:, 0:1]
            elif cand_metrics.dim() == 3:
                score_gt = model(gt_metrics)
                score_candidates = model(cand_metrics)
                diff = score_gt - score_candidates
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
