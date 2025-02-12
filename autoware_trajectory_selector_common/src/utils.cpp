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
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <optional>
#include <vector>

namespace autoware::trajectory_selector::utils
{

using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;

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
  interpolated_point.pose = autoware::universe_utils::calcInterpolatedPose(curr_pt, next_pt, ratio);

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
  const auto v1 = autoware::universe_utils::point2tfVector(curr_pt, next_pt);
  const auto v2 = autoware::universe_utils::point2tfVector(curr_pt, target_pose);
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

auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double resolution) -> TrajectoryPoints
{
  const auto ego_seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p_ego, 10.0, M_PI_2);

  std::vector<double> timestamps;
  for (const auto & point : points) {
    timestamps.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  const auto timestamp_increasing = interpolation::isIncreasing(timestamps);

  if (timestamp_increasing) {
    const auto start_time = rclcpp::Duration(points.at(ego_seg_idx).time_from_start).seconds();
    return sampling_with_time(points, sample_num, resolution, start_time);
  }

  TrajectoryPoints output;
  const auto vehicle_pose_frenet = convertToFrenetPoint(points, p_ego.position, ego_seg_idx);

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
  const size_t start_idx) -> TrajectoryPoints
{
  TrajectoryPoints output;
  output.reserve(sample_num);

  const double start_time = rclcpp::Duration(points.at(start_idx).time_from_start).seconds();

  for (size_t i = 0; i < sample_num; i++) {
    const auto elapsed_time = sample_num * resolution + start_time;
    const auto index = find_nearest_timestamp(points, elapsed_time, start_idx);
    if (!index.has_value()) {
      output.push_back(points.at(start_idx));
      continue;
    }
    if (index == sample_num - 1) {
      output.push_back(points.back());
      continue;
    }
    const double t1 = rclcpp::Duration(points.at(index.value()).time_from_start).seconds();
    const double t2 = rclcpp::Duration(points.at(index.value() + 1).time_from_start).seconds();
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
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

  if (!feasible) {
    for (const auto & point : *points) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(createMarkerColor(0.1, 0.1, 0.1, 0.3));
    }
  } else {
    for (const auto & point : *points) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(
        createMarkerColor(1.0 - score, score, 0.0, std::clamp(score, 0.5, 0.999)));
    }
  }

  return marker;
}
}  // namespace autoware::trajectory_selector::utils
