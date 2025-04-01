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

#ifndef FEASIBLE_TRAJECTORY_FILTER__UTILS_HPP_
#define FEASIBLE_TRAJECTORY_FILTER__UTILS_HPP_

#include <autoware_new_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace autoware::trajectory_selector::feasible_trajectory_filter::utils
{
bool is_trajectory_offtrack(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Point & ego_position);
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter::utils

#endif  // FEASIBLE_TRAJECTORY_FILTER__UTILS_HPP_
