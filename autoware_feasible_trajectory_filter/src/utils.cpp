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
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <vector>

namespace autoware::trajectory_selector::feasible_trajectory_filter::utils
{
bool check_finite(const TrajectoryPoint & point)
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  if (!p_result) RCLCPP_INFO(rclcpp::get_logger(__func__), "p infinite");
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  if (!quat_result) RCLCPP_INFO(rclcpp::get_logger(__func__), "o infinite");
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  if (!v_result) RCLCPP_INFO(rclcpp::get_logger(__func__), "v infinite");
  const bool w_result = isfinite(point.heading_rate_rps);
  if (!w_result) RCLCPP_INFO(rclcpp::get_logger(__func__), "w infinite");
  const bool a_result = isfinite(point.acceleration_mps2);
  if (!a_result) RCLCPP_INFO(rclcpp::get_logger(__func__), "a infinite");

  return quat_result && p_result && v_result && w_result && a_result;
}

bool is_invalid_trajectory(const TrajectoryPoints & points)
{
  if (points.size() < 2) return true;

  std::vector<double> timestamps;
  for (const auto & point : points) {
    timestamps.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  if (!interpolation::isIncreasing(timestamps)) return true;

  if (rclcpp::Duration(points.back().time_from_start).seconds() < 8.0)
    return true;  // To-do (go-sakayori): remove hard code parameter

  const auto is_finite = std::all_of(
    points.begin(), points.end(), [](const auto & point) { return check_finite(point); });

  return !is_finite;
}

bool is_trajectory_offtrack(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Point & ego_position)
{
  static constexpr double epsilon = 5.0;

  const auto idx = autoware::motion_utils::findNearestIndex(trajectory.points, ego_position);
  const auto target_position = trajectory.points.at(idx).pose.position;

  return autoware_utils::calc_squared_distance2d(ego_position, target_position) > epsilon;
}

bool is_out_of_lane(
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
      lanelet::BasicPoint2d(point.pose.position.x, point.pose.position.y), 0.0);
    if (nearest_lanelet.empty()) return true;
  }
  return false;
}
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter::utils
