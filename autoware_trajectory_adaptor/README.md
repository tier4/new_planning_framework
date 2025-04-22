# Trajectory Adaptor

## Purpose/Role

This node converts a set of ranked candidate trajectories into a single, cleaned‑up [autoware_planning_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg).

## Algorithm Overview

Upon receiving a message, the node finds the candidate with the highest score.
If no candidate is found an empty list would be published. The selected trajectory’s points are then processed, ensuring that successive points match the conditions to conduct control modules.

## Interface

### Topics

| Direction | Topic Name                   | Message Type                                    | Description                        |
| --------- | ---------------------------- | ----------------------------------------------- | ---------------------------------- |
| Subscribe | `~/input/trajectories`       | `autoware_new_planning_msgs/msg/Trajectories`   | Candidate trajectories with scores |
| Publish   | `~/output/trajectory`        | `autoware_planning_msgs/msg/Trajectory`         | Re-arranged best‑score trajectory  |
| Publish   | `~/output/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard light information           |

### Parameters

The current implementation does not expose any ROS parameters. All behavior is hard‑coded.
