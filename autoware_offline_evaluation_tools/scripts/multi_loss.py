import torch
import torch.nn as nn
from utils import average_displacement_error, final_displacement_error


def adaptive_margin_ranking_loss_with_weight(
    model,
    gt_metrics,
    gt_pose,
    cand_metrics,
    cand_pose,
    base_margin=0.0,
    lambda_factor=0.1,
    threshold=0.5,
    use_fde=False,
):
    """
    Adding sample weighting to Adaptive Margin Ranking Loss.

    ground_truth: (B, T, D)
    candidate: (B, T, D)
    """
    if use_fde:
        error_term = final_displacement_error(gt_pose, cand_pose)
    else:
        error_term = average_displacement_error(gt_pose, cand_pose)

    adaptive_margin = base_margin + lambda_factor * error_term

    sample_weight = torch.ones_like(error_term)
    sample_weight[error_term < threshold] = 0.5

    score_gt = model(gt_metrics)
    score_cand = model(cand_metrics)
    loss = torch.clamp(adaptive_margin - (score_gt - score_cand), min=0)
    weighted_loss = loss * sample_weight
    return weighted_loss.mean()


def margin_ranking_loss(model, ground_truth, candidate, margin=1.0):
    """
    Use Margin Ranking Loss if candidate trajectory size is 1.

    ground_truth: (B, T, D)
    candidate: (B, T, D)
    """
    score_gt = model(ground_truth)  # (B, 1)
    score_cand = model(candidate)  # (B, 1)
    target = torch.ones_like(score_gt)
    loss_fn = nn.MarginRankingLoss(margin=margin)
    loss = loss_fn(score_gt, score_cand, target)
    return loss


def triplet_loss(model, gt_metrics, gt_pose, cand_metrics, cand_pose, margin=1.0):
    """
    Use Triplet Loss if candidate trajectory size is more than 2.

    ground_truth: (B, T, D) [anchor]
    candidates: (B, num_candidates, T, D)
    """
    B, num_candidates, T, D = cand_metrics.shape
    diff = cand_pose - gt_pose.unsqueeze(1)
    error = torch.norm(diff, p=2, dim=-1)
    ade = error.mean(dim=-1)

    # For each sample, the candidate with the smallest ADE is considered positive, and the candidate with the largest ADE is considered negative.
    pos_idx = torch.argmin(ade, dim=1)  # (B,)
    neg_idx = torch.argmax(ade, dim=1)  # (B,)
    pos_candidates = cand_metrics[torch.arange(B), pos_idx, :, :]  # (B, T, D)
    neg_candidates = cand_metrics[torch.arange(B), neg_idx, :, :]  # (B, T, D)

    score_gt = model(gt_metrics)  # (B, 1)
    score_pos = model(pos_candidates)  # (B, 1)
    score_neg = model(neg_candidates)  # (B, 1)

    loss_pos = torch.clamp(margin - (score_gt - score_pos), min=0)
    loss_neg = torch.clamp(margin - (score_pos - score_neg), min=0)
    loss = (loss_pos + loss_neg).mean()
    return loss


def adaptive_triplet_loss(model, gt_metrics, gt_pose, cand_metrics, cand_pose, margin=1.0):
    """
    Triplet Loss based on ADE for multiple candidate trajectories.

    Parameters:
      gt_metrics: (B, T, D) Ground truth metrics.
      gt_pose:  (B, T, 2) Ground truth positions.
      cand_metrics: (B, num_candidates, T, D) Candidate metrics.
      cand_pose:  (B, num_candidates, T, 2) Candidate positions.
      margin: base margin
    """
    B, num_candidates, T, D = cand_metrics.shape
    diff = cand_pose - gt_pose.unsqueeze(1)  # (B, num_candidates, T, 2)
    error = torch.norm(diff, p=2, dim=-1)  # (B, num_candidates, T)
    ade = error.mean(dim=-1)  # (B, num_candidates)

    pos_idx = torch.argmin(ade, dim=1)  # (B,)
    neg_idx = torch.argmax(ade, dim=1)  # (B,)
    pos_candidates = cand_metrics[torch.arange(B), pos_idx, :, :]  # (B, T, D)
    neg_candidates = cand_metrics[torch.arange(B), neg_idx, :, :]  # (B, T, D)

    score_gt = model(gt_metrics)  # (B, 1)
    score_positive = model(pos_candidates)  # (B, 1)
    score_negative = model(neg_candidates)  # (B, 1)

    loss_positive = torch.clamp(margin - (score_gt - score_positive), min=0)
    loss_negative = torch.clamp(margin - (score_positive - score_negative), min=0)
    loss = loss_positive + loss_negative  # (B, 1) or (B,)

    # Calculate the variance of ADE among candidates and set sample weights
    candidate_diff = torch.max(ade, dim=1)[0] - torch.min(ade, dim=1)[0]  # (B,)
    sample_weight = torch.ones_like(candidate_diff)
    sample_weight[candidate_diff < 0.5] = 0.5

    weighted_loss = (loss.mean(dim=1) * sample_weight).mean()
    return weighted_loss


def multi_candidate_loss(
    model,
    gt_metrics,
    gt_pose,
    cand_metrics,
    cand_pose,
    margin=1.0,
    lambda_factor=0.1,
    threshold=5.0,
    use_fde=False,
):
    """
    Adaptive loss selection based on candidate count.

    If candidate trajectory count is 1 (cand_mets shape: (B, T, D)), use adaptive margin ranking loss based on ADE/FDE computed from positions.
    If candidate trajectory count is >=2 (cand_mets shape: (B, num_candidates, T, D)), use triplet loss based on ADE computed from positions.

    Parameters:
      gt_mets: (B, T, D)  Ground truth metrics.
      gt_pos:  (B, T, 2)  Ground truth positions.
      cand_mets: if single candidate → (B, T, D),
                 if multiple → (B, num_candidates, T, D)
      cand_pos:  if single candidate → (B, T, 2),
                 if multiple → (B, num_candidates, T, 2)
      margin: base margin.
      lambda_factor: scaling factor for the error term.
      threshold: error threshold for sample weighting.
      use_fde: if True, use FDE instead of ADE.
    """
    if cand_metrics.dim() == 3:  # Single candidate: (B, T, D) and (B, T, 2)
        return adaptive_margin_ranking_loss_with_weight(
            model,
            gt_metrics,
            gt_pose,
            cand_metrics,
            cand_pose,
            base_margin=margin,
            lambda_factor=lambda_factor,
            threshold=threshold,
            use_fde=use_fde,
        )
    elif cand_metrics.dim() == 4:
        num_candidates = cand_metrics.shape[1]
        if num_candidates >= 2:
            return adaptive_triplet_loss(
                model, gt_metrics, gt_pose, cand_metrics, cand_pose, margin
            )
        else:
            candidate = cand_metrics.squeeze(1)  # (B, T, D)
            cand_pos_squeezed = cand_pose.squeeze(1)  # (B, T, 2)
            return adaptive_margin_ranking_loss_with_weight(
                model,
                gt_metrics,
                gt_pose,
                candidate,
                cand_pos_squeezed,
                base_margin=margin,
                lambda_factor=lambda_factor,
                threshold=threshold,
                use_fde=use_fde,
            )
    else:
        raise ValueError("Invalid candidate dimensions")


def pairwise_adaptive_ranking_loss(
    model, gt_metrics, gt_pose, cand_metrics, cand_pose, margin=1.0, lambda_factor=1.0
):
    """
    Calculate pairwise ranking loss.

    - Within each sample, among multiple candidates, the candidate with a smaller ADE based on location information should be considered a "good candidate" and have a higher score.
    - Specifically, for each candidate pair (i,j), if the ADE of candidate i is smaller than the ADE of candidate j, the score of candidate i is trained to be sufficiently higher than the score of candidate j (score_i - score_j ≥ adaptive_margin).

    Parameters:
      model: model used. Input (B, T, D) -> output (B, 1)
      gt_metrics: (B, T, D) Ground truth metrics used as reference.
      gt_pos:  (B, T, 2) Ground truth position.
      cand_metrics:
          - Single candidate: (B, T, D)
          - Multiple candidate: (B, num_candidates, T, D)
      cand_pose:
          - Single candidate: (B, T, 2)
          - Multiple candidate: (B, num_candidates, T, 2)
      margin: base margin (Lower limit of score difference between good and bad candidates)
      lambda_factor: A coefficient to adjust the margin by multiplying the difference in ADE

    Returns:
      The average loss over all candidate pairs (scalar tensor).
    """
    # In the case of a single candidate, we use the conventional margin ranking loss.
    if cand_metrics.dim() == 3:
        return margin_ranking_loss(model, gt_metrics, cand_metrics, margin)

    B, num_candidates, T, D = cand_metrics.shape
    cand_reshaped = cand_metrics.view(B * num_candidates, T, D)
    cand_scores = model(cand_reshaped).view(B, num_candidates)  # (B, num_candidates)

    diff = cand_pose - gt_pose.unsqueeze(1)  # (B, num_candidates, T, 2)
    error = torch.norm(diff, p=2, dim=-1)  # (B, num_candidates, T)
    ade = error.mean(dim=2)  # (B, num_candidates)

    total_loss = 0.0
    total_pairs = 0
    for b in range(B):
        for i in range(num_candidates):
            for j in range(num_candidates):
                if i == j:
                    continue
                # if candidate i is better than candidate j (ADE_i < ADE_j),
                # then we want candidate i's score to be higher than candidate j's score.
                if ade[b, i] < ade[b, j]:
                    # adaptive margin can be defined as base_margin + lambda_factor * (ade_j - ade_i)
                    adaptive_margin = margin + lambda_factor * (ade[b, j] - ade[b, i])
                    # We want: score_i - score_j >= adaptive_margin
                    loss_ij = torch.clamp(
                        adaptive_margin - (cand_scores[b, i] - cand_scores[b, j]), min=0
                    )
                    total_loss += loss_ij
                    total_pairs += 1
    if total_pairs > 0:
        return total_loss / total_pairs
    else:
        return torch.tensor(0.0, device=gt_metrics.device)
