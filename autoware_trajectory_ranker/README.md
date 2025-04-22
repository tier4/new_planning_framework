# Trajectory Ranker

## Purpose/Role

The Trajectory Ranker node transforms a set of viable candidate paths into an ordered list by attaching a scalar score to every trajectory.
By doing so it decouples upstream path‑generation logic from downstream motion‑control logic:
generators can remain diverse (rule‑based, ML‑based, optimization‑based, etc.) while the controller simply follows the highest‑ranked path.
The node therefore raises overall driving performance and keeps the system architecturally flexible.

## Algorithm Overview

At every planning period the node latches the most recent odometry and perception context, then enters a single pass through five logically distinct stages.
First, it ingests the raw list of candidate trajectories broadcast by upstream generators.
Second, it converts each path into a common frame and re‑samples it into an equal‑length sequence of points at a fixed spatial resolution so that subsequent metrics can be computed consistently in time and space.
Third, a configurable bank of metric plug‑ins evaluates every re‑sampled point sequence;
each metric returns a vector of cost values that reflect safety, comfort and efficiency considerations under the current map, route and obstacle context
Fourth, the node folds those metric vectors into a single scalar by applying per‑metric weights, optional clamping and an exponentially decaying time‑weight that biases the near‑term portion of the path.
Finally, it publishes the scored list together with debug artifacts (markers, detailed scores, resampled paths) so that downstream modules can pick or inspect the best trajectory

### Resampling

Every incoming trajectory is re‑interpolated relative to the ego pose so that its first point aligns with the current vehicle position.
Sub‑sampling or over‑sampling is then performed until the path comprises exactly `sample_num` points, each spaced resolution metres apart.
This step ensures that metric plugins can assume identical temporal spacing even when generators produce paths of different lengths or densities, making cost vectors comparable across candidates.

### Metric evaluation

For each re‑sampled path the evaluator executes a plugin chain whose names are specified in `metrics.name`. Currently implemented plugins are lateral acceleration, longitudinal jerk, travel distance, time‑to‑collision, lateral deviation from the reference lane, and steering consistency.

### Score aggregation

The aggregation stage first multiplies every metric trace by its scalar weight from `score_weight`, then applies an element‑wise multiplication with one of the predefined `time_decay_weight`.
The weighted traces are summed across metrics and time to form a single scalar.Finally, that scalar is multiplied by an overall `score_weight` factor so that integrators can scale scores without touching individual weights.

## Interface

### Topics

| Direction | Topic name              | Message type                                       | Purpose                  |
| --------- | ----------------------- | -------------------------------------------------- | ------------------------ |
| Subscribe | `~/input/trajectories`  | `autoware_new_planning_msgs/msg/Trajectories`      | Candidate paths          |
| Subscribe | `~/input/odometry`      | `nav_msgs/msg/Odometry`                            | Ego odometry             |
| SUbscribe | `~/input/objects`       | `autoware_perception_msgs/msg/PredictedObjects`    | Obstacle                 |
| Subscribe | `~/input/lanelet2_map`  | `autoware_lanelet2_msgs/msg/LaneletMapBin`         | HD map                   |
| Subscribe | `~/input/route`         | `autoware_lanelet2_msgs/msg/LaneletRoute`          | Active route             |
| Publish   | `~/output/trajectories` | `autoware_new_planning_msgs/msg/Trajectories`      | Trajectories with scores |
| Publish   | `~/output/score_debug`  | `autoware_new_planning_msgs/msg/TrajectoriesDebug` | Per‑metric score vectors |

### Parameters

| Name                | Type           | Default                                                                                                       | Description                                 |
| ------------------- | -------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------- |
| `sample_num`        | int            | 20                                                                                                            | Points per trajectory after resampling      |
| `resolution`        | double         | 0.5                                                                                                           | Interval between resampled points [s]       |
| `metrics.name`      | string         | LateralAcceleration, LongitudinalJerk, TravelDistance, TimeToCollision, LateralDeviation, SteeringConsistency | List of metric plug‑ins to load             |
| `metrics.maximum`   | double         | -                                                                                                             | Per‑metric clamp values                     |
| `score_weight`      | double         | -                                                                                                             | Global scaling of final score               |
| `time_decay_weight` | vector<double> | -                                                                                                             | Exponential decay applied to each time step |
