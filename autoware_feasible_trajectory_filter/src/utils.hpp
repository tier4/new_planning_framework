// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware/trajectory_selector_common/type_alias.hpp>

#include <autoware_new_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace autoware::trajectory_selector::feasible_trajectory_filter::utils
{
bool is_invalid_trajectory(const TrajectoryPoints & points);

bool is_trajectory_offtrack(
  const TrajectoryPoints & points, const geometry_msgs::msg::Point & ego_position);

bool is_out_of_lane(
  const TrajectoryPoints & points, const lanelet::LaneletMapConstPtr & lanelet_map,
  const double look_ahead_time);

bool has_collision_risk(
  const TrajectoryPoints & points, const PredictedObjects & objects, double look_ahead_time);
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter::utils

#endif  // UTILS_HPP_
