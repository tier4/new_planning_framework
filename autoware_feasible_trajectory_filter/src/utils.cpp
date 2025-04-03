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

#include "utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

namespace autoware::trajectory_selector::feasible_trajectory_filter::utils
{
bool is_trajectory_offtrack(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Point & ego_position)
{
  static constexpr double epsilon = 5.0;

  const auto idx = autoware::motion_utils::findNearestIndex(trajectory.points, ego_position);
  const auto target_position = trajectory.points.at(idx).pose.position;

  return autoware_utils::calc_squared_distance2d(ego_position, target_position) > epsilon;
}

bool out_of_lane(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const double look_ahead_time)
{
  if (!lanelet_map) {
    return false;
  }

  for (const auto & point : trajectory.points) {
    if (rclcpp::Duration(point.time_from_start).seconds() > look_ahead_time) break;
    const auto nearest_lanelet = lanelet::geometry::findWithin2d(
      lanelet_map->laneletLayer,
      lanelet::BasicPoint2d(point.pose.position.x, point.pose.position.y),
      1.0);  // Todo (go-sakayori): remove hard code value
    if (nearest_lanelet.empty()) return true;
  }
  return false;
}
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter::utils
