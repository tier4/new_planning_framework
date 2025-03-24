import torch
import torch.nn as nn


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


def triplet_loss(model, ground_truth, candidates, margin=1.0):
    """
    Use Triplet Loss if candidate trajectory size is more than 2.

    ground_truth: (B, T, D) [anchor]
    candidates: (B, num_candidates, T, D)
      ここでは positive = candidates[:, 0, :, :], negative = candidates[:, 1, :, :]
    """
    B, num_candidates, T, D = candidates.shape
    anchor_score = model(ground_truth)  # (B, 1)
    positive = candidates[:, 0, :, :]  # (B, T, D)
    negative = candidates[:, 1, :, :]  # (B, T, D)
    score_pos = model(positive)  # (B, 1)
    score_neg = model(negative)  # (B, 1)
    loss_pos = torch.clamp(margin - (anchor_score - score_pos), min=0)
    loss_neg = torch.clamp(margin - (score_pos - score_neg), min=0)
    loss = (loss_pos + loss_neg).mean()
    return loss


def multi_candidate_loss(model, ground_truth, candidates, margin=1.0):
    """
    Change the loss function according to candidate size.

    ground_truth: (B, T, D)
    candidates: Tensor of shape (B, num_candidates, T, D)
      - if num_candidates == 1 → use margin ranking loss
      - if num_candidates >= 2 → use triplet loss (using the first two candidates)
    """
    if candidates.dim() != 4:
        raise ValueError("Invalid candidate dimensions, expected 4D tensor.")
    num_candidates = candidates.shape[1]
    if num_candidates == 1:
        candidate = candidates.squeeze(1)  # (B, T, D)
        return margin_ranking_loss(model, ground_truth, candidate, margin)
    elif num_candidates >= 2:
        return triplet_loss(model, ground_truth, candidates, margin)
    else:
        raise ValueError("Unexpected candidate shape.")
