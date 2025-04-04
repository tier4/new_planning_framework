// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_

#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <tf2/LinearMath/Vector3.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::utils
{
Point vector2point(const geometry_msgs::msg::Vector3 & v);

tf2::Vector3 from_msg(const Point & p);

tf2::Vector3 get_velocity_in_world_coordinate(const Pose & p_world, const Vector3 & v_local);

tf2::Vector3 get_velocity_in_world_coordinate(const PredictedObjectKinematics & kinematics);
tf2::Vector3 get_velocity_in_world_coordinate(const Odometry & odometry);

tf2::Vector3 get_velocity_in_world_coordinate(const TrajectoryPoint & point);

TrajectoryPoint calc_extended_point(const TrajectoryPoint & end_point, const double extension_time);

double time_to_collision(
  const TrajectoryPoint & point, const size_t idx,
  const autoware_perception_msgs::msg::PredictedObject & object);

auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double time_resolution) -> TrajectoryPoints;

auto sampling_with_time(
  const TrajectoryPoints & points, const size_t sample_num, const double resolution,
  const std::optional<size_t> start_idx) -> TrajectoryPoints;

auto find_nearest_timestamp(
  const TrajectoryPoints & points, const double target_timestamp,
  const size_t start_index) -> std::optional<size_t>;
}  // namespace autoware::trajectory_selector::utils

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_
