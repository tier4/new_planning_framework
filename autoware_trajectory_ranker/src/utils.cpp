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

#include "autoware/interpolation/linear_interpolation.hpp"

namespace autoware::trajectory_selector::trajectory_ranker
{

namespace
{

struct FrenetPoint
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

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

auto vector2point(const geometry_msgs::msg::Vector3 & v) -> Point
{
  return autoware::universe_utils::createPoint(v.x, v.y, v.z);
}

auto from_msg(const Point & p)
{
  return tf2::Vector3(p.x, p.y, p.z);
}

auto get_velocity_in_world_coordinate(const PredictedObjectKinematics & kinematics) -> tf2::Vector3
{
  const auto pose = kinematics.initial_pose_with_covariance.pose;
  const auto v_local = kinematics.initial_twist_with_covariance.twist.linear;
  const auto v_world = autoware::universe_utils::transformPoint(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

auto get_velocity_in_world_coordinate(const TrajectoryPoint & point) -> tf2::Vector3
{
  const auto pose = point.pose;
  const auto v_local =
    geometry_msgs::build<Vector3>().x(point.longitudinal_velocity_mps).y(0.0).z(0.0);
  const auto v_world = autoware::universe_utils::transformPoint(vector2point(v_local), pose);

  return from_msg(v_world) - from_msg(pose.position);
}

double time_to_collision(
  const PredictedObjects & objects, const Pose & p_ego, const tf2::Vector3 & v_ego)
{
  if (objects.objects.empty()) {
    return std::numeric_limits<double>::max();
  }

  std::vector<double> time_to_collisions(objects.objects.size());

  for (const auto & object : objects.objects) {
    const auto p_object = object.kinematics.initial_pose_with_covariance.pose;
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

  // Interpolate
  TrajectoryPoint interpolated_point{};

  // pose interpolation
  interpolated_point.pose =
    autoware::universe_utils::calcInterpolatedPose(curr_pt, next_pt, clamped_ratio);

  // twist interpolation
  if (use_zero_order_hold_for_twist) {
    interpolated_point.longitudinal_velocity_mps = curr_pt.longitudinal_velocity_mps;
    interpolated_point.lateral_velocity_mps = curr_pt.lateral_velocity_mps;
    interpolated_point.acceleration_mps2 = curr_pt.acceleration_mps2;
  } else {
    interpolated_point.longitudinal_velocity_mps = autoware::interpolation::lerp(
      curr_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, clamped_ratio);
    interpolated_point.lateral_velocity_mps = autoware::interpolation::lerp(
      curr_pt.lateral_velocity_mps, next_pt.lateral_velocity_mps, clamped_ratio);
    interpolated_point.acceleration_mps2 = autoware::interpolation::lerp(
      curr_pt.acceleration_mps2, next_pt.acceleration_mps2, clamped_ratio);
  }

  // heading rate interpolation
  interpolated_point.heading_rate_rps = autoware::interpolation::lerp(
    curr_pt.heading_rate_rps, next_pt.heading_rate_rps, clamped_ratio);

  // wheel interpolation
  interpolated_point.front_wheel_angle_rad = autoware::interpolation::lerp(
    curr_pt.front_wheel_angle_rad, next_pt.front_wheel_angle_rad, clamped_ratio);
  interpolated_point.rear_wheel_angle_rad = autoware::interpolation::lerp(
    curr_pt.rear_wheel_angle_rad, next_pt.rear_wheel_angle_rad, clamped_ratio);

  // time interpolation
  const double interpolated_time = autoware::interpolation::lerp(
    rclcpp::Duration(curr_pt.time_from_start).seconds(),
    rclcpp::Duration(next_pt.time_from_start).seconds(), clamped_ratio);
  interpolated_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time);

  return interpolated_point;
}

}  // namespace

double lateral_accel(
  const VehicleInfo & vehicle_info, const TrajectoryPoints & points, const size_t idx)
{
  const auto radius = vehicle_info.wheel_base_m / std::tan(points.at(idx).front_wheel_angle_rad);
  const auto speed = points.at(idx).longitudinal_velocity_mps;
  return speed * speed / radius;
}

double longitudinal_jerk(const TrajectoryPoints & points, const size_t idx)
{
  return (points.at(idx + 1).acceleration_mps2 - points.at(idx).acceleration_mps2) / 0.5;
}

double minimum_ttc(
  const PredictedObjects & objects, const TrajectoryPoints & points, const size_t idx)
{
  const auto p_ego = points.at(idx).pose;
  const auto v_ego = get_velocity_in_world_coordinate(points.at(idx));

  return time_to_collision(objects, p_ego, v_ego);
}

double travel_distance(const TrajectoryPoints & points, const size_t idx)
{
  return autoware::motion_utils::calcSignedArcLength(points, 0L, idx);
}

double longitudinal_comfortability(const std::vector<double> & values, const double resample_num)
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 0.5;
  const auto normalize = [&min, &max](const double value) {
    return (max - std::clamp(value, min, max)) / (max - min);
  };

  for (size_t i = 0; i < resample_num; i++) {
    score += normalize(std::pow(TIME_FACTOR, i) * std::abs(values.at(i)));
  }

  return score / resample_num;
}

double lateral_comfortability(const std::vector<double> & values, const double resample_num)
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 0.5;
  const auto normalize = [&min, &max](const double value) {
    return (max - std::clamp(value, min, max)) / (max - min);
  };

  for (size_t i = 0; i < resample_num; i++) {
    score += normalize(std::pow(TIME_FACTOR, i) * std::abs(values.at(i)));
  }

  return score / resample_num;
}

double efficiency(const std::vector<double> & values, const double resample_num)
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 20.0;
  const auto normalize = [&min, &max](const double value) {
    return std::clamp(value, min, max) / (max - min);
  };

  for (size_t i = 0; i < resample_num; i++) {
    score += normalize(std::pow(TIME_FACTOR, i) * values.at(i) / 0.5);
  }

  return score / resample_num;
}

double safety(const std::vector<double> & values, const double resample_num)
{
  constexpr double TIME_FACTOR = 0.8;

  double score = 0.0;

  const auto min = 0.0;
  const auto max = 5.0;
  const auto normalize = [&min, &max](const double value) {
    return std::clamp(value, min, max) / (max - min);
  };

  for (size_t i = 0; i < resample_num; i++) {
    score += normalize(std::pow(TIME_FACTOR, i) * values.at(i));
  }

  return score / resample_num;
}

auto resampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t resample_num,
  const double time_resolution) -> TrajectoryPoints
{
  const auto ego_seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p_ego, 10.0, M_PI_2);

  TrajectoryPoints output;
  const auto vehicle_pose_frenet = convertToFrenetPoint(points, p_ego.position, ego_seg_idx);

  double length = 0.0;
  for (size_t i = 0; i < resample_num; i++) {
    const auto pose =
      autoware::motion_utils::calcInterpolatedPose(points, vehicle_pose_frenet.length + length);
    const auto p_trajectory = calcInterpolatedPoint(
      points, pose, false, std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    output.push_back(p_trajectory);

    const auto pred_accel = p_trajectory.acceleration_mps2;
    const auto pred_velocity = p_trajectory.longitudinal_velocity_mps;

    length +=
      pred_velocity * time_resolution + 0.5 * pred_accel * time_resolution * time_resolution;
  }

  return output;
}

}  // namespace autoware::trajectory_selector::trajectory_ranker
