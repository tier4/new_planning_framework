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

#include "utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils_geometry/boost_polygon_utils.hpp"

#include <autoware/trajectory_selector_common/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <algorithm>

namespace autoware::trajectory_selector::trajectory_metrics::utils
{
namespace internal
{
geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::msg::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

double calcRadius(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  constexpr double RADIUS_MAX = 1e9;
  const double denominator = 2 * transformToRelativeCoordinate2D(target, current_pose).y;
  const double numerator =
    autoware_utils_geometry::calc_squared_distance2d(target, current_pose.position);

  if (fabs(denominator) > 0) {
    return numerator / denominator;
  }
  return RADIUS_MAX;
}

double curvature(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  constexpr double KAPPA_MAX = 1e9;
  const double radius = calcRadius(target, current_pose);

  if (fabs(radius) > 0) {
    return 1 / radius;
  }
  return KAPPA_MAX;
}

auto pure_pursuit(const std::shared_ptr<TrajectoryPoints> & points, const Pose & ego_pose) -> double
{
  const auto target_point = [&points, ego_pose]() {
    constexpr double lookahead_distance = 10.0;
    const auto p = autoware::motion_utils::calcLongitudinalOffsetPoint(
      *points, ego_pose.position, lookahead_distance);

    if (p.has_value()) return p.value();

    return autoware_utils_geometry::get_point(points->back());
  }();

  return curvature(target_point, ego_pose);
}

}  // namespace internal

auto time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects, const size_t idx) -> double
{
  static double constexpr max_ttc_value = 10.0;
  if (!objects || objects->objects.empty()) {
    return max_ttc_value;
  }

  std::vector<double> time_to_collisions;
  time_to_collisions.reserve(objects->objects.size());

  for (const auto & object : objects->objects) {
    const auto time = points->at(idx).time_from_start;
    time_to_collisions.push_back(
      autoware::trajectory_selector::utils::time_to_collision(points->at(idx), time, object));
  }

  std::sort(time_to_collisions.begin(), time_to_collisions.end());

  return time_to_collisions.front();
}

auto time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects,
  const std::shared_ptr<VehicleInfo> & vehicle_info) -> std::vector<double>
{
  if (objects->objects.empty()) {
    return std::vector<double>(20, 10000.0);
  }

  const double base_to_front = vehicle_info->max_longitudinal_offset_m;
  const double base_to_rear = vehicle_info->rear_overhang_m;
  const double width = vehicle_info->vehicle_width_m;

  std::vector<double> time_to_collisions(points->size());

  for (size_t i = 0; i < 20; i++) {
    const auto & ego_pose = points->at(i).pose;
    const auto ego_polygon =
      autoware_utils_geometry::to_footprint(ego_pose, base_to_front, base_to_rear, width);

    for (const auto & object : objects->objects) {
      const auto max_confidence_path = std::max_element(
        object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
        [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

      if (max_confidence_path == object.kinematics.predicted_paths.end()) continue;

      if (max_confidence_path->path.size() < i + 1) continue;

      const auto object_pose = max_confidence_path->path.at(i);
      const auto object_polygon = autoware_utils_geometry::to_polygon2d(object_pose, object.shape);

      if (!boost::geometry::disjoint(ego_polygon, object_polygon)) {
        std::reverse(time_to_collisions.begin(), time_to_collisions.end());
        for (size_t j = time_to_collisions.size() - 1 - i; j < time_to_collisions.size(); j++) {
          time_to_collisions.at(j) = (j + i - time_to_collisions.size()) * 0.5;
        }
        std::reverse(time_to_collisions.begin(), time_to_collisions.end());
        for (size_t j = i; j < time_to_collisions.size(); j++) {
          time_to_collisions.at(j) = 0.0;
        }
        return time_to_collisions;
      }
    }
  }

  return std::vector<double>(20, 10000.0);
}

auto steer_command(
  const std::shared_ptr<TrajectoryPoints> & points, const Pose & ego_pose,
  const double wheel_base) -> double
{
  return std::atan(wheel_base * internal::pure_pursuit(points, ego_pose));
}

}  // namespace autoware::trajectory_selector::trajectory_metrics::utils
