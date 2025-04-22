# Trajectory Selector Common

## Purpose/Role

This package is the foundation layer for the trajectory‑selector stack. It provides:

- Header‑only interfaces (MetricInterface, TrajectoryFilterInterface, RankerInterface) that plugins implement.
- Common type aliases (Trajectory, TrajectoryPoints, TrajectoryWithMetrics) so all selector components share identical data structures.
- Lightweight math and geometry helpers used by metrics and filters.

Because it is purely header‑based, linking against this package adds no runtime overhead;
it simply guarantees that every metric, filter, ranker, and adaptor in the new planning framework follows the same contract.
