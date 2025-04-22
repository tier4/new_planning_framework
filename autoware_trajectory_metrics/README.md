# Trajectory Metrics

## Purpose/Role

This package provides a plugin‑based framework for computing common quality metrics over a time‑parameterised trajectory.
The metrics are designed for use inside the trajectory‑selector stack, but they can be reused by any module that needs per‑point evaluations such as comfort scores, safety indicators, or similarity measures.
It ships with seven ready‑to‑use plugins: LateralAcceleration, LongitudinalJerk, TimeToCollision, TravelDistance, LateralDeviation, TrajectoryDeviation, and SteeringConsistency.
It exposes a lightweight MetricInterface so you can add your own metrics via pluginlib.
