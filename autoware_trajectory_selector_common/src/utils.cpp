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

#include "autoware/trajectory_selector_common/utils.hpp"

#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <magic_enum.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/detail/predicted_object__struct.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_path__struct.hpp>
#include <autoware_planning_msgs/msg/detail/lanelet_route__builder.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__builder.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <vector>

namespace autoware::trajectory_selector::utils
{
struct FrenetPoint
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

TrajectoryPoint calcInterpolatedPoint(
  const TrajectoryPoint & curr_pt, const TrajectoryPoint & next_pt, const double ratio,
  const bool use_zero_order_hold_for_twist)
{
  TrajectoryPoint interpolated_point{};

  // pose interpolation
  interpolated_point.pose =
    autoware_utils_geometry::calc_interpolated_pose(curr_pt, next_pt, ratio);

  // twist interpolation
  if (use_zero_order_hold_for_twist) {
    interpolated_point.longitudinal_velocity_mps = curr_pt.longitudinal_velocity_mps;
    interpolated_point.lateral_velocity_mps = curr_pt.lateral_velocity_mps;
    interpolated_point.acceleration_mps2 = curr_pt.acceleration_mps2;
  } else {
    interpolated_point.longitudinal_velocity_mps = autoware::interpolation::lerp(
      curr_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, ratio);
    interpolated_point.lateral_velocity_mps = autoware::interpolation::lerp(
      curr_pt.lateral_velocity_mps, next_pt.lateral_velocity_mps, ratio);
    interpolated_point.acceleration_mps2 =
      autoware::interpolation::lerp(curr_pt.acceleration_mps2, next_pt.acceleration_mps2, ratio);
  }

  // heading rate interpolation
  interpolated_point.heading_rate_rps =
    autoware::interpolation::lerp(curr_pt.heading_rate_rps, next_pt.heading_rate_rps, ratio);

  // wheel interpolation
  interpolated_point.front_wheel_angle_rad = autoware::interpolation::lerp(
    curr_pt.front_wheel_angle_rad, next_pt.front_wheel_angle_rad, ratio);
  interpolated_point.rear_wheel_angle_rad = autoware::interpolation::lerp(
    curr_pt.rear_wheel_angle_rad, next_pt.rear_wheel_angle_rad, ratio);

  // time interpolation
  const double interpolated_time = autoware::interpolation::lerp(
    rclcpp::Duration(curr_pt.time_from_start).seconds(),
    rclcpp::Duration(next_pt.time_from_start).seconds(), ratio);
  interpolated_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time);

  return interpolated_point;
}

TrajectoryPoint calcInterpolatedPoint(
  const TrajectoryPoints & points, const geometry_msgs::msg::Pose & target_pose,
  const bool use_zero_order_hold_for_twist, const double dist_threshold, const double yaw_threshold)
{
  if (points.empty()) {
    TrajectoryPoint interpolated_point{};
    interpolated_point.pose = target_pose;
    return interpolated_point;
  }
  if (points.size() == 1) {
    return points.front();
  }

  const size_t segment_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, target_pose, dist_threshold, yaw_threshold);

  // Calculate interpolation ratio
  const auto & curr_pt = points.at(segment_idx);
  const auto & next_pt = points.at(segment_idx + 1);
  const auto v1 = autoware_utils_geometry::point_2_tf_vector(curr_pt, next_pt);
  const auto v2 = autoware_utils_geometry::point_2_tf_vector(curr_pt, target_pose);
  if (v1.length2() < 1e-3) {
    return curr_pt;
  }

  const double ratio = v1.dot(v2) / v1.length2();
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  return calcInterpolatedPoint(curr_pt, next_pt, clamped_ratio, use_zero_order_hold_for_twist);
}

template <class T>
auto convertToFrenetPoint(const T & points, const Point & search_point_geom, const size_t seg_idx)
  -> FrenetPoint
{
  FrenetPoint frenet_point;

  const double longitudinal_length =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, search_point_geom);
  frenet_point.length =
    autoware::motion_utils::calcSignedArcLength(points, 0, seg_idx) + longitudinal_length;
  frenet_point.distance =
    autoware::motion_utils::calcLateralOffset(points, search_point_geom, seg_idx);

  return frenet_point;
}

TrajectoryPoints create_trajectory_points(
  const autoware_perception_msgs::msg::PredictedPath & path,
  const geometry_msgs::msg::Vector3 velocity)
{
  TrajectoryPoints trajectory;
  trajectory.reserve(path.path.size());

  for (size_t i = 0; i < path.path.size(); i++) {
    const auto time = rclcpp::Duration::from_seconds(
      rclcpp::Duration(path.time_step.sec, path.time_step.nanosec).seconds() *
      static_cast<double>(i));
    const auto trajectory_point = autoware_planning_msgs::build<TrajectoryPoint>()
                                    .time_from_start(time)
                                    .pose(path.path.at(i))
                                    .longitudinal_velocity_mps(velocity.x)
                                    .lateral_velocity_mps(velocity.y)
                                    .acceleration_mps2(0.0)
                                    .heading_rate_rps(0.0)
                                    .front_wheel_angle_rad(0.0)
                                    .rear_wheel_angle_rad(0.0);
    trajectory.push_back(trajectory_point);
  }
  return trajectory;
}

Point vector2point(const geometry_msgs::msg::Vector3 & v)
{
  return autoware_utils_geometry::create_point(v.x, v.y, v.z);
}

tf2::Vector3 from_msg(const Point & p)
{
  return tf2::Vector3(p.x, p.y, p.z);
}

tf2::Vector3 get_velocity_in_world_coordinate(const Pose & p_world, const Vector3 & v_local)
{
  const auto v_world = autoware_utils_geometry::transform_point(vector2point(v_local), p_world);

  return from_msg(v_world) - from_msg(p_world.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const PredictedObjectKinematics & kinematics)
{
  const auto pose = kinematics.initial_pose_with_covariance.pose;
  const auto v_local = kinematics.initial_twist_with_covariance.twist.linear;
  const auto v_world = autoware_utils_geometry::transform_point(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const Odometry & odometry)
{
  const auto pose = odometry.pose.pose;
  const auto v_local = odometry.twist.twist.linear;
  const auto v_world = autoware_utils_geometry::transform_point(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const TrajectoryPoint & point)
{
  const auto pose = point.pose;
  const auto v_local = geometry_msgs::build<Vector3>()
                         .x(point.longitudinal_velocity_mps)
                         .y(point.lateral_velocity_mps)
                         .z(0.0);
  const auto v_world = autoware_utils_geometry::transform_point(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

TrajectoryPoint calc_extended_point(const TrajectoryPoint & end_point, const double extension_time)
{
  const auto new_yaw =
    tf2::getYaw(end_point.pose.orientation) + end_point.heading_rate_rps * extension_time;
  const auto new_time =
    rclcpp::Duration(end_point.time_from_start) + rclcpp::Duration::from_seconds(extension_time);
  const auto world_coordinate_velocity = get_velocity_in_world_coordinate(end_point);

  const auto new_point =
    geometry_msgs::build<Point>()
      .x(end_point.pose.position.x + world_coordinate_velocity.x() * extension_time)
      .y(end_point.pose.position.y + world_coordinate_velocity.y() * extension_time)
      .z(end_point.pose.position.z);

  tf2::Quaternion new_quaternion;
  new_quaternion.setRPY(0.0, 0.0, new_yaw);
  new_quaternion.normalize();

  const auto new_pose =
    geometry_msgs::build<Pose>().position(new_point).orientation(tf2::toMsg(new_quaternion));
  const auto new_trajectory_point =
    autoware_planning_msgs::build<TrajectoryPoint>()
      .time_from_start(new_time)
      .pose(new_pose)
      .longitudinal_velocity_mps(
        end_point.longitudinal_velocity_mps + end_point.acceleration_mps2 * extension_time)
      .lateral_velocity_mps(end_point.lateral_velocity_mps)
      .acceleration_mps2(end_point.acceleration_mps2)
      .heading_rate_rps(end_point.heading_rate_rps)
      .front_wheel_angle_rad(end_point.front_wheel_angle_rad)
      .rear_wheel_angle_rad(end_point.rear_wheel_angle_rad);

  return new_trajectory_point;
}

double time_to_collision(
  const TrajectoryPoint & ego_point, const rclcpp::Duration & duration,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  static double constexpr max_ttc_value = 10.0;
  const auto max_confidence_path = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  if (max_confidence_path == object.kinematics.predicted_paths.end()) return max_ttc_value;

  const auto object_trajectory = create_trajectory_points(
    *max_confidence_path, object.kinematics.initial_twist_with_covariance.twist.linear);
  const auto idx = find_nearest_timestamp(object_trajectory, duration);

  if (!idx.has_value()) return max_ttc_value;

  TrajectoryPoint object_point;

  if (idx.value() >= object_trajectory.size() - 1) {
    object_point = object_trajectory.back();
  } else {
    const double t1 = rclcpp::Duration(object_trajectory.at(idx.value()).time_from_start).seconds();
    const double t2 =
      rclcpp::Duration(object_trajectory.at(idx.value() + 1).time_from_start).seconds();
    if (t2 == t1) {
      object_point = object_trajectory.at(idx.value());
    } else {
      const double ratio = (duration.seconds() - t1) / (t2 - t1);
      const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
      object_point = calcInterpolatedPoint(
        object_trajectory.at(idx.value()), object_trajectory.at(idx.value() + 1), clamped_ratio,
        false);
    }
  }

  const auto vector_ego_object =
    autoware_utils_geometry::point_2_tf_vector(ego_point, object_point);

  const auto ego_world_velocity = get_velocity_in_world_coordinate(ego_point);
  const auto object_world_velocity = get_velocity_in_world_coordinate(object_point);
  const auto relative_velocity = tf2::tf2Dot(vector_ego_object.normalized(), ego_world_velocity) -
                                 tf2::tf2Dot(vector_ego_object.normalized(), object_world_velocity);
  if (relative_velocity < 1e-03) return max_ttc_value;

  return std::min(max_ttc_value, vector_ego_object.length() / relative_velocity);
}

auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double resolution) -> TrajectoryPoints
{
  const auto ego_seg_idx = autoware::motion_utils::findNearestIndex(points, p_ego, 10.0, M_PI_2);

  return sampling_with_time(points, sample_num, resolution, ego_seg_idx);
}

auto sampling_with_time(
  const TrajectoryPoints & points, const size_t sample_num, const double resolution,
  const std::optional<size_t> start_idx) -> TrajectoryPoints
{
  TrajectoryPoints output;
  output.reserve(sample_num);

  if (points.empty() || !start_idx.has_value() || start_idx.value() >= points.size()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger(__func__), "Error: points is empty or start_idx is out of range!");
    return output;
  }

  const double start_time =
    rclcpp::Duration(points.at(start_idx.value()).time_from_start).seconds();

  for (size_t i = 0; i < sample_num; i++) {
    const auto elapsed_time =
      rclcpp::Duration::from_seconds(static_cast<double>(i) * resolution + start_time);
    const auto index = find_nearest_timestamp(points, elapsed_time, start_idx.value());
    if (!index.has_value()) {
      output.push_back(points.back());
      continue;
    }
    if (index.value() >= points.size() - 1) {
      output.push_back(points.back());
      continue;
    }
    const double t1 = rclcpp::Duration(points.at(index.value()).time_from_start).seconds();
    const double t2 = rclcpp::Duration(points.at(index.value() + 1).time_from_start).seconds();

    if (t2 == t1) {
      output.push_back(points.at(index.value()));
      continue;
    }

    const double ratio = (elapsed_time.seconds() - t1) / (t2 - t1);
    const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
    output.push_back(calcInterpolatedPoint(
      points.at(index.value()), points.at(index.value() + 1), clamped_ratio, false));
  }
  return output;
}

std::optional<size_t> find_nearest_timestamp(
  const TrajectoryPoints & points, const rclcpp::Duration & target_timestamp, size_t start_index)
{
  for (size_t i = start_index; i < points.size(); i++) {
    rclcpp::Duration time(points.at(i).time_from_start.sec, points.at(i).time_from_start.nanosec);
    if (time > target_timestamp) return (i - 1);
  }
  return std::nullopt;
}
}  // namespace autoware::trajectory_selector::utils
