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

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>
#include <rclcpp/duration.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>

#include <autoware_planning_msgs/msg/detail/lanelet_route__builder.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__builder.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <cmath>
#include <optional>
#include <vector>

namespace autoware::trajectory_selector::utils
{

using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

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
  interpolated_point.pose = autoware_utils::calc_interpolated_pose(curr_pt, next_pt, ratio);

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
  const auto v1 = autoware_utils::point_2_tf_vector(curr_pt, next_pt);
  const auto v2 = autoware_utils::point_2_tf_vector(curr_pt, target_pose);
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

Point vector2point(const geometry_msgs::msg::Vector3 & v)
{
  return autoware_utils::create_point(v.x, v.y, v.z);
}

tf2::Vector3 from_msg(const Point & p)
{
  return tf2::Vector3(p.x, p.y, p.z);
}

tf2::Vector3 get_velocity_in_world_coordinate(const PredictedObjectKinematics & kinematics)
{
  const auto pose = kinematics.initial_pose_with_covariance.pose;
  const auto v_local = kinematics.initial_twist_with_covariance.twist.linear;
  const auto v_world = autoware_utils::transform_point(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const Odometry & odometry)
{
  const auto pose = odometry.pose.pose;
  const auto v_local = odometry.twist.twist.linear;
  const auto v_world = autoware_utils::transform_point(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

tf2::Vector3 get_velocity_in_world_coordinate(const TrajectoryPoint & point)
{
  const auto pose = point.pose;
  const auto v_local =
    geometry_msgs::build<Vector3>().x(point.longitudinal_velocity_mps).y(0.0).z(0.0);
  const auto v_world = autoware_utils::transform_point(vector2point(v_local), pose);

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

auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double resolution) -> TrajectoryPoints
{
  const auto ego_seg_idx = autoware::motion_utils::findNearestIndex(points, p_ego, 10.0, M_PI_2);

  std::vector<double> timestamps;
  for (const auto & point : points) {
    timestamps.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  const auto timestamp_increasing = interpolation::isIncreasing(timestamps);

  if (timestamp_increasing) {
    return sampling_with_time(points, sample_num, resolution, ego_seg_idx);
  }

  TrajectoryPoints output;
  FrenetPoint vehicle_pose_frenet;
  if (ego_seg_idx.has_value()) {
    vehicle_pose_frenet = convertToFrenetPoint(points, p_ego.position, ego_seg_idx.value());
  }

  double length = 0.0;
  for (size_t i = 0; i < sample_num; i++) {
    const auto pose =
      autoware::motion_utils::calcInterpolatedPose(points, vehicle_pose_frenet.length + length);
    const auto p_trajectory = calcInterpolatedPoint(
      points, pose, false, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    output.push_back(p_trajectory);

    const auto pred_accel = p_trajectory.acceleration_mps2;
    const auto pred_velocity = p_trajectory.longitudinal_velocity_mps;

    length += pred_velocity * resolution + 0.5 * pred_accel * resolution * resolution;
  }

  return output;
}

auto sampling_with_time(
  const TrajectoryPoints & points, const size_t sample_num, const double resolution,
  const std::optional<size_t> start_idx) -> TrajectoryPoints
{
  TrajectoryPoints output;
  output.reserve(sample_num);

  if (points.empty() || !start_idx.has_value() || start_idx.value() >= points.size()) {
    std::cerr << "Error: points is empty or start_idx is out of range!" << std::endl;
    return output;
  }

  const double start_time =
    rclcpp::Duration(points.at(start_idx.value()).time_from_start).seconds();

  for (size_t i = 0; i < sample_num; i++) {
    const auto elapsed_time = static_cast<double>(i) * resolution + start_time;
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

    const double ratio = (elapsed_time - t1) / (t2 - t1);
    const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
    output.push_back(calcInterpolatedPoint(
      points.at(index.value()), points.at(index.value() + 1), clamped_ratio, false));
  }
  return output;
}

auto find_nearest_timestamp(
  const TrajectoryPoints & points, const double target_timestamp,
  const size_t start_index) -> std::optional<size_t>
{
  auto start_idx = start_index > 0 ? start_index : 1;
  for (size_t i = start_idx; i < points.size(); i++) {
    const auto time = rclcpp::Duration(points.at(i).time_from_start).seconds();
    if (time > target_timestamp) return (i - 1);
  }
  return std::nullopt;
}

auto to_marker(
  const std::shared_ptr<TrajectoryPoints> & points, const double score, const bool feasible,
  const std::string & ns, const size_t id) -> Marker
{
  Marker marker = create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.0, 0.0), create_marker_color(1.0, 1.0, 1.0, 0.999));

  if (!feasible) {
    for (const auto & point : *points) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(create_marker_color(0.1, 0.1, 0.1, 0.3));
    }
  } else {
    for (const auto & point : *points) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(
        create_marker_color(1.0 - score, score, 0.0, std::clamp(score, 0.5, 0.999)));
    }
  }

  return marker;
}
}  // namespace autoware::trajectory_selector::utils
