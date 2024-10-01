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

#include "autoware/trajectory_evaluator/utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>

namespace autoware::trajectory_selector::trajectory_evaluator::utils
{

using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;

Point vector2point(const geometry_msgs::msg::Vector3 & v)
{
  return autoware::universe_utils::createPoint(v.x, v.y, v.z);
}

tf2::Vector3 from_msg(const Point & p)
{
  return tf2::Vector3(p.x, p.y, p.z);
}

tf2::Vector3 get_velocity_in_world_coordinate(const PredictedObjectKinematics & kinematics)
{
  const auto pose = kinematics.initial_pose_with_covariance.pose;
  const auto v_local = kinematics.initial_twist_with_covariance.twist.linear;
  const auto v_world = autoware::universe_utils::transformPoint(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const Odometry & odometry)
{
  const auto pose = odometry.pose.pose;
  const auto v_local = odometry.twist.twist.linear;
  const auto v_world = autoware::universe_utils::transformPoint(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const TrajectoryPoint & point)
{
  const auto pose = point.pose;
  const auto v_local =
    geometry_msgs::build<Vector3>().x(point.longitudinal_velocity_mps).y(0.0).z(0.0);
  const auto v_world = autoware::universe_utils::transformPoint(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

auto time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects, const size_t idx) -> double
{
  if (objects->objects.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto p_ego = points->at(idx).pose;
  const auto v_ego = utils::get_velocity_in_world_coordinate(points->at(idx));

  std::vector<double> time_to_collisions(objects->objects.size());

  for (const auto & object : objects->objects) {
    const auto max_confidence_path = std::max_element(
      object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
      [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

    if (max_confidence_path == object.kinematics.predicted_paths.end()) continue;

    if (max_confidence_path->path.size() < idx + 1) continue;

    const auto & p_object = max_confidence_path->path.at(idx);
    const auto v_ego2object =
      autoware::universe_utils::point2tfVector(p_ego.position, p_object.position);

    const auto v_object = get_velocity_in_world_coordinate(object.kinematics);
    const auto v_relative = tf2::tf2Dot(v_ego2object.normalized(), v_ego) -
                            tf2::tf2Dot(v_ego2object.normalized(), v_object);

    time_to_collisions.push_back(v_ego2object.length() / v_relative);
  }

  const auto itr = std::remove_if(
    time_to_collisions.begin(), time_to_collisions.end(),
    [](const auto & value) { return value < 1e-3; });
  time_to_collisions.erase(itr, time_to_collisions.end());

  std::sort(time_to_collisions.begin(), time_to_collisions.end());

  return time_to_collisions.front();
}

auto to_marker(
  const std::shared_ptr<DataInterface> & data, const SCORE & score_type, const size_t id) -> Marker
{
  if (data == nullptr) return Marker{};

  const auto idx = static_cast<size_t>(score_type);
  const auto score = data->scores().at(idx);
  std::stringstream ss;
  ss << magic_enum::enum_name(score_type);

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ss.str(), id, Marker::LINE_STRIP,
    createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

  if (!data->feasible()) {
    for (const auto & point : *data->points()) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(createMarkerColor(0.1, 0.1, 0.1, 0.3));
    }
  } else {
    for (const auto & point : *data->points()) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(createMarkerColor(1.0 - score, score, 0.0, std::min(0.5, score)));
    }
  }

  return marker;
}

}  // namespace autoware::trajectory_selector::trajectory_evaluator::utils
