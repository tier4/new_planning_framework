import torch
import torch.nn as nn


class MetricSubNet(nn.Module):
    """
    Generates a single output by performing a linear combination of the 20-point values for each metric.
    """

    def __init__(self, seq_len: int, time_decay_weight, trainable=True, reparam_time_weights=True):
        super().__init__()
        if len(time_decay_weight) != seq_len:
            raise ValueError(
                f"Time decay weight length mismatch: expected {seq_len}, got {len(time_decay_weight)}")
        self.reparam_time_weights = reparam_time_weights
        if self.reparam_time_weights:
            sw = torch.tensor(time_decay_weight, dtype=torch.float32).clamp(0.01, 0.99)
            self.unconstrained_time_weights = nn.Parameter(
                torch.log(sw / (1 - sw)), requires_grad=trainable)
        else:
            self.weights = nn.Parameter(torch.tensor(
                time_decay_weight, dtype=torch.float32), requires_grad=trainable)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (batch_size, trajectory_point_num)
        if self.reparam_time_weights:
            time_weights = torch.sigmoid(self.unconstrained_time_weights)
        else:
            time_weights = self.weights
        return (x * time_weights).sum(dim=1, keepdim=True)


class TrajectoryNetPerMetric(nn.Module):
    """
     Calculates scores for each metric using MetricSubNet,
     then normalizes each metric's output by its specified maximum value.

     Metrics where lower values are preferable
     (LateralAcceleration, LongitudinalJerk, LateralDeviation, SteeringConsistency)
     are normalized as: 1 - (value / max_value).
     Metrics where higher values are preferable (TravelDistance, TimeToCollision)
     are normalized as: value / max_value.

     Subsequently, each metric is multiplied by its weight,
     and the weighted sum across metrics becomes the final score.

     Input shape: (batch_size, trajectory_point_num, num_metrics)
     Output shape: (batch_size, 1)
     """

    def __init__(self, num_metrics, seq_len, score_weight, time_decay_weights,
                 normalize_score=False, max_values=None, lower_better_mask=None, use_max_values=True, reparam_weights=True, reparam_time_weights=True, train_subnet=True):
        super().__init__()
        if num_metrics != len(score_weight):
            raise ValueError(
                f"score_weight length mismatch: expected {num_metrics}, got {len(score_weight)}")
        self.num_metrics = num_metrics
        self.subnets = nn.ModuleList([
            MetricSubNet(
                seq_len, time_decay_weights[f"s{i}"], trainable=train_subnet, reparam_time_weights=reparam_time_weights)
            for i in range(num_metrics)
        ])
        self.reparam_weights = reparam_weights
        if self.reparam_weights:
            sw = torch.tensor(score_weight, dtype=torch.float32).clamp(0.01, 0.99)
            self.unconstrained_metric_weights = nn.Parameter(torch.log(sw / (1 - sw)))
        else:
            self.metric_weights = nn.Parameter(torch.tensor(score_weight, dtype=torch.float32))

        self.normalize_score = normalize_score
        self.use_max_values = use_max_values
        if self.normalize_score:
            if self.use_max_values:
                if max_values is None:
                    raise ValueError(
                        "max_values must be provided if normalize_score is True and use_max_values is True")
                self.max_values = torch.tensor(max_values, dtype=torch.float32)
            if lower_better_mask is None:
                raise ValueError("lower_better_mask must be provided if normalize_score is True")
            self.lower_better_mask = torch.tensor(lower_better_mask, dtype=torch.bool)

    def forward(self, x):
        # x: (batch_size, trajectory_point_num, num_metrics)
        x = x.transpose(1, 2)
        outputs = torch.cat([subnet(x[:, i, :]) for i, subnet in enumerate(self.subnets)], dim=1)
        if self.reparam_weights:
            metric_weights = torch.sigmoid(self.unconstrained_metric_weights)
        else:
            metric_weights = self.metric_weights
        if self.normalize_score:
            normalized = torch.empty_like(outputs)
            for i in range(self.num_metrics):
                if self.use_max_values:
                    if self.lower_better_mask[i]:
                        normalized[:, i] = 1 - (outputs[:, i] / self.max_values[i])
                    else:
                        normalized[:, i] = outputs[:, i] / self.max_values[i]
                else:
                    if self.lower_better_mask[i]:
                        normalized[:, i] = 1 - outputs[:, i]
                    else:
                        normalized[:, i] = outputs[:, i]
            weighted = normalized * metric_weights
            score = weighted.sum(dim=1, keepdim=True)
        else:
            weighted = outputs * metric_weights
            score = weighted.sum(dim=1, keepdim=True)
        return score


def load_and_print_weights(model_path):
    seq_len = 20
    num_metrics = 6
    time_decay_weights = {f"s{i}": [1.0] * seq_len for i in range(num_metrics)}
    score_weight = [1.0] * num_metrics
    max_values = [100.0, 50.0, 200.0, 10.0, 5.0, 2.0]
    lower_better_mask = [True, True, False, False, True, True]

    model = TrajectoryNetPerMetric(
        num_metrics=num_metrics,
        seq_len=seq_len,
        score_weight=score_weight,
        time_decay_weights=time_decay_weights,
        normalize_score=True,
        max_values=max_values,
        lower_better_mask=lower_better_mask,
        train_subnet=True
    )

    state_dict = torch.load(model_path, map_location=torch.device('cpu'))
    model.load_state_dict(state_dict)


if __name__ == '__main__':
    load_and_print_weights("trained_model_weights.pth")
