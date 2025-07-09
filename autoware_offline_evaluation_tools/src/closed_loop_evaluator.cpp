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

#include "closed_loop_evaluator.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
ClosedLoopEvaluator::ClosedLoopEvaluator(
  rclcpp::Logger logger, std::shared_ptr<autoware::route_handler::RouteHandler> route_handler)
: logger_(logger), route_handler_(route_handler)
{
  summary_ = EvaluationSummary{};
}

void ClosedLoopEvaluator::evaluate(
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
  rosbag2_cpp::Writer * bag_writer)
{
  if (synchronized_data_list.empty()) {
    RCLCPP_WARN(logger_, "No synchronized data to evaluate");
    return;
  }

  metrics_list_.clear();
  metrics_list_.reserve(synchronized_data_list.size());

  std::shared_ptr<SynchronizedData> previous_data = nullptr;

  for (const auto & sync_data : synchronized_data_list) {
    if (!sync_data->kinematic_state || !sync_data->trajectory) {
      continue;
    }

    auto metrics = calculate_metrics(sync_data, previous_data);
    metrics_list_.push_back(metrics);

    if (bag_writer) {
      save_metrics_to_bag(metrics, sync_data, *bag_writer);
    }

    previous_data = sync_data;
  }

  calculate_summary();

  RCLCPP_INFO(
    logger_,
    "Evaluation complete. Processed %zu samples. "
    "Mean lateral error: %.3f m, Max lateral error: %.3f m, "
    "Min TTC: %.3f s",
    summary_.num_samples, summary_.mean_lateral_error, summary_.max_lateral_error,
    summary_.min_ttc);
}

TrajectoryMetrics ClosedLoopEvaluator::calculate_metrics(
  const std::shared_ptr<SynchronizedData> & current_data,
  const std::shared_ptr<SynchronizedData> & previous_data)
{
  TrajectoryMetrics metrics;
  metrics.timestamp = current_data->timestamp;

  const auto & current_pose = current_data->kinematic_state->pose.pose;
  const auto & current_twist = current_data->kinematic_state->twist.twist;
  const auto & trajectory = *current_data->trajectory;

  // Calculate position errors
  if (route_handler_ && route_handler_->isHandlerReady()) {
    // Use preferred lane centerline if available
    metrics.lateral_error = calculate_lateral_error_from_preferred_lane(current_pose);
  } else {
    // Fallback to trajectory-based error
    metrics.lateral_error = calculate_lateral_error(current_pose, trajectory);
  }
  metrics.longitudinal_error = calculate_longitudinal_error(current_pose, trajectory);

  // Calculate longitudinal acceleration
  if (current_data->acceleration) {
    const auto & accel = current_data->acceleration->accel.accel.linear;
    metrics.longitudinal_acceleration = accel.x;
  } else {
    metrics.longitudinal_acceleration = 0.0;
  }

  // Calculate jerk (if previous data available)
  if (previous_data && previous_data->acceleration && current_data->acceleration) {
    const double dt = (current_data->timestamp - previous_data->timestamp).seconds();
    if (dt > 0.0) {
      const double prev_accel = previous_data->acceleration->accel.accel.linear.x;
      const double curr_accel = current_data->acceleration->accel.accel.linear.x;
      metrics.jerk = (curr_accel - prev_accel) / dt;
    } else {
      metrics.jerk = 0.0;
    }
  } else {
    metrics.jerk = 0.0;
  }

  // Calculate curvature from steering angle
  if (current_data->steering_status) {
    const double wheelbase = 2.79;  // Default wheelbase, should be from vehicle_info
    const double steering_angle = current_data->steering_status->steering_tire_angle;
    metrics.curvature = std::tan(steering_angle) / wheelbase;
  } else {
    metrics.curvature = 0.0;
  }

  // Calculate lateral acceleration from velocity and curvature
  // lateral_acceleration = v^2 * curvature = v^2 * tan(steering_angle) / wheelbase
  const double velocity = std::hypot(current_twist.linear.x, current_twist.linear.y);
  metrics.lateral_acceleration = velocity * velocity * std::abs(metrics.curvature);

  // Calculate oscillation metrics
  calculate_oscillation_metrics(metrics, current_data, previous_data);

  // Calculate TTC if objects are available
  if (current_data->objects) {
    metrics.ttc = calculate_ttc(current_pose, current_twist, *current_data->objects);
    metrics.time_gap = metrics.ttc;  // Simplified for now
  } else {
    metrics.ttc = std::numeric_limits<double>::max();
    metrics.time_gap = std::numeric_limits<double>::max();
  }

  return metrics;
}

double ClosedLoopEvaluator::calculate_lateral_error(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & p1 = trajectory.points[i].pose.position;
    const auto & p2 = trajectory.points[i + 1].pose.position;

    // Calculate distance from point to line segment
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double l2 = dx * dx + dy * dy;

    if (l2 == 0.0) {
      // p1 and p2 are the same point
      const double dist =
        std::hypot(current_pose.position.x - p1.x, current_pose.position.y - p1.y);
      min_distance = std::min(min_distance, dist);
      continue;
    }

    // Calculate projection
    const double t = std::max(
      0.0,
      std::min(
        1.0, ((current_pose.position.x - p1.x) * dx + (current_pose.position.y - p1.y) * dy) / l2));

    const double proj_x = p1.x + t * dx;
    const double proj_y = p1.y + t * dy;

    const double dist =
      std::hypot(current_pose.position.x - proj_x, current_pose.position.y - proj_y);

    min_distance = std::min(min_distance, dist);
  }

  return min_distance;
}

double ClosedLoopEvaluator::calculate_longitudinal_error(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }

  const size_t closest_idx = find_closest_trajectory_point(current_pose, trajectory);

  // Calculate signed longitudinal distance
  double accumulated_dist = 0.0;

  // Find where the vehicle is along the trajectory
  for (size_t i = 1; i <= closest_idx && i < trajectory.points.size(); ++i) {
    const auto & p1 = trajectory.points[i - 1].pose.position;
    const auto & p2 = trajectory.points[i].pose.position;
    accumulated_dist += std::hypot(p2.x - p1.x, p2.y - p1.y);
  }

  // Add distance from closest point to actual position
  if (closest_idx < trajectory.points.size() - 1) {
    const auto & p1 = trajectory.points[closest_idx].pose.position;
    const auto & p2 = trajectory.points[closest_idx + 1].pose.position;

    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double l2 = dx * dx + dy * dy;

    if (l2 > 0.0) {
      const double t =
        ((current_pose.position.x - p1.x) * dx + (current_pose.position.y - p1.y) * dy) / l2;

      if (t >= 0.0 && t <= 1.0) {
        accumulated_dist += t * std::sqrt(l2);
      }
    }
  }

  // Expected distance based on time
  const double expected_dist =
    closest_idx > 0 ? trajectory.points[closest_idx].longitudinal_velocity_mps *
                        rclcpp::Duration(trajectory.points[closest_idx].time_from_start).seconds()
                    : 0.0;

  return accumulated_dist - expected_dist;
}

void ClosedLoopEvaluator::calculate_oscillation_metrics(
  TrajectoryMetrics & metrics, const std::shared_ptr<SynchronizedData> & current_data,
  const std::shared_ptr<SynchronizedData> & previous_data)
{
  // Get current steering angle
  if (current_data->steering_status) {
    metrics.steering_angle = current_data->steering_status->steering_tire_angle;
  } else {
    metrics.steering_angle = 0.0;
  }

  // Calculate steering angular velocity
  if (previous_data && previous_data->steering_status && current_data->steering_status) {
    const double dt = (current_data->timestamp - previous_data->timestamp).seconds();
    if (dt > 0.0) {
      const double prev_angle = previous_data->steering_status->steering_tire_angle;
      const double curr_angle = current_data->steering_status->steering_tire_angle;
      metrics.steering_angular_velocity = (curr_angle - prev_angle) / dt;
    } else {
      metrics.steering_angular_velocity = 0.0;
    }
  } else {
    metrics.steering_angular_velocity = 0.0;
  }

  // Calculate lateral jerk (change in lateral acceleration)
  if (previous_data) {
    const double dt = (current_data->timestamp - previous_data->timestamp).seconds();
    if (dt > 0.0) {
      // Calculate previous lateral acceleration
      const auto & prev_twist = previous_data->kinematic_state->twist.twist;
      const double prev_velocity = std::hypot(prev_twist.linear.x, prev_twist.linear.y);
      double prev_curvature = 0.0;
      if (previous_data->steering_status) {
        const double wheelbase = 2.79;
        const double prev_steering = previous_data->steering_status->steering_tire_angle;
        prev_curvature = std::tan(prev_steering) / wheelbase;
      }
      const double prev_lateral_accel = prev_velocity * prev_velocity * std::abs(prev_curvature);

      metrics.lateral_jerk = (metrics.lateral_acceleration - prev_lateral_accel) / dt;
    } else {
      metrics.lateral_jerk = 0.0;
    }
  } else {
    metrics.lateral_jerk = 0.0;
  }

  // Calculate yaw rate
  const auto & twist = current_data->kinematic_state->twist.twist;
  metrics.yaw_rate = twist.angular.z;
}

double ClosedLoopEvaluator::calculate_lateral_error_from_preferred_lane(
  const geometry_msgs::msg::Pose & current_pose)
{
  if (!route_handler_ || !route_handler_->isHandlerReady()) {
    return 0.0;
  }

  // Get the preferred lanes
  const auto preferred_lanes = route_handler_->getPreferredLanelets();
  if (preferred_lanes.empty()) {
    return 0.0;
  }

  // Use the same approach as LateralDeviation::evaluate
  // Calculate arc coordinates which gives the lateral distance from the lane centerline
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(preferred_lanes, current_pose);

  // Return the absolute lateral distance
  return std::abs(arc_coordinates.distance);
}

double ClosedLoopEvaluator::calculate_ttc(
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_twist,
  const autoware_perception_msgs::msg::PredictedObjects & objects)
{
  double min_ttc = std::numeric_limits<double>::max();

  const double ego_velocity = std::hypot(current_twist.linear.x, current_twist.linear.y);

  if (ego_velocity < 0.1) {  // Nearly stationary
    return min_ttc;
  }

  for (const auto & object : objects.objects) {
    if (object.kinematics.predicted_paths.empty()) {
      continue;
    }

    const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & obj_twist = object.kinematics.initial_twist_with_covariance.twist;

    // Simple TTC calculation based on relative position and velocity
    const double dx = obj_pose.position.x - current_pose.position.x;
    const double dy = obj_pose.position.y - current_pose.position.y;
    const double distance = std::hypot(dx, dy);

    // Relative velocity (simplified - assumes objects moving in same direction)
    const double relative_velocity =
      ego_velocity - std::hypot(obj_twist.linear.x, obj_twist.linear.y);

    if (relative_velocity > 0.1 && distance > 0.0) {
      const double ttc = distance / relative_velocity;
      min_ttc = std::min(min_ttc, ttc);
    }
  }

  return min_ttc;
}

size_t ClosedLoopEvaluator::find_closest_trajectory_point(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0;
  }

  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i].pose.position;
    const double distance =
      std::hypot(current_pose.position.x - point.x, current_pose.position.y - point.y);

    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }

  return closest_idx;
}

void ClosedLoopEvaluator::save_metrics_to_bag(
  const TrajectoryMetrics & metrics, const std::shared_ptr<SynchronizedData> & sync_data,
  rosbag2_cpp::Writer & bag_writer)
{
  const auto timestamp = metrics.timestamp;

  // Save lateral error
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.lateral_error;
    bag_writer.write(msg, "/evaluation/lateral_error", timestamp);
  }

  // Save acceleration
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = timestamp;
    msg.point.x = metrics.longitudinal_acceleration;
    msg.point.y = metrics.lateral_acceleration;
    msg.point.z = metrics.jerk;
    bag_writer.write(msg, "/evaluation/acceleration_metrics", timestamp);
  }

  // Save TTC
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.ttc;
    bag_writer.write(msg, "/evaluation/ttc", timestamp);
  }

  // Save oscillation metrics
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = timestamp;
    msg.point.x = metrics.steering_angle;
    msg.point.y = metrics.steering_angular_velocity;
    msg.point.z = metrics.lateral_jerk;
    bag_writer.write(msg, "/evaluation/oscillation_metrics", timestamp);
  }

  // Save yaw rate
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.yaw_rate;
    bag_writer.write(msg, "/evaluation/yaw_rate", timestamp);
  }

  // Save localization (Odometry)
  if (sync_data && sync_data->kinematic_state) {
    bag_writer.write(*sync_data->kinematic_state, "/evaluation/localization", timestamp);
  }
}

void ClosedLoopEvaluator::calculate_summary()
{
  if (metrics_list_.empty()) {
    return;
  }

  summary_ = EvaluationSummary{};
  summary_.num_samples = metrics_list_.size();

  // Calculate statistics
  double sum_lateral_error = 0.0;
  double sum_squared_lateral_error = 0.0;
  double sum_acceleration = 0.0;
  double sum_jerk = 0.0;
  double sum_steering_angle = 0.0;
  double sum_squared_steering_angle = 0.0;
  double sum_steering_angular_velocity = 0.0;
  double sum_lateral_jerk = 0.0;

  summary_.max_lateral_error = 0.0;
  summary_.max_acceleration = 0.0;
  summary_.max_jerk = 0.0;
  summary_.max_steering_angular_velocity = 0.0;
  summary_.max_lateral_jerk = 0.0;
  summary_.min_ttc = std::numeric_limits<double>::max();
  summary_.steering_reversals = 0;

  // Count steering reversals
  double prev_steering_velocity = 0.0;

  for (size_t i = 0; i < metrics_list_.size(); ++i) {
    const auto & metrics = metrics_list_[i];

    sum_lateral_error += metrics.lateral_error;
    sum_squared_lateral_error += metrics.lateral_error * metrics.lateral_error;
    sum_acceleration += std::abs(metrics.lateral_acceleration);
    sum_jerk += std::abs(metrics.jerk);
    sum_steering_angle += metrics.steering_angle;
    sum_squared_steering_angle += metrics.steering_angle * metrics.steering_angle;
    sum_steering_angular_velocity += std::abs(metrics.steering_angular_velocity);
    sum_lateral_jerk += std::abs(metrics.lateral_jerk);

    summary_.max_lateral_error = std::max(summary_.max_lateral_error, metrics.lateral_error);
    summary_.max_acceleration =
      std::max(summary_.max_acceleration, std::abs(metrics.lateral_acceleration));
    summary_.max_jerk = std::max(summary_.max_jerk, std::abs(metrics.jerk));
    summary_.max_steering_angular_velocity =
      std::max(summary_.max_steering_angular_velocity, std::abs(metrics.steering_angular_velocity));
    summary_.max_lateral_jerk = std::max(summary_.max_lateral_jerk, std::abs(metrics.lateral_jerk));

    // Count steering reversals (sign changes in steering velocity)
    if (i > 0 && std::abs(metrics.steering_angular_velocity) > 0.01) {  // Threshold to avoid noise
      if (prev_steering_velocity * metrics.steering_angular_velocity < 0) {
        summary_.steering_reversals++;
      }
      prev_steering_velocity = metrics.steering_angular_velocity;
    }

    if (metrics.ttc < std::numeric_limits<double>::max()) {
      summary_.min_ttc = std::min(summary_.min_ttc, metrics.ttc);
    }
  }

  summary_.mean_lateral_error = sum_lateral_error / static_cast<double>(summary_.num_samples);
  summary_.mean_acceleration = sum_acceleration / static_cast<double>(summary_.num_samples);
  summary_.mean_jerk = sum_jerk / static_cast<double>(summary_.num_samples);
  summary_.mean_steering_angular_velocity =
    sum_steering_angular_velocity / static_cast<double>(summary_.num_samples);
  summary_.mean_lateral_jerk = sum_lateral_jerk / static_cast<double>(summary_.num_samples);

  // Calculate standard deviations
  const double mean_lateral_error = summary_.mean_lateral_error;
  const double mean_steering_angle = sum_steering_angle / static_cast<double>(summary_.num_samples);

  summary_.std_lateral_error = std::sqrt(
    sum_squared_lateral_error / static_cast<double>(summary_.num_samples) -
    mean_lateral_error * mean_lateral_error);
  summary_.std_steering_angle = std::sqrt(
    sum_squared_steering_angle / static_cast<double>(summary_.num_samples) -
    mean_steering_angle * mean_steering_angle);

  // Calculate total distance and time
  if (metrics_list_.size() > 1) {
    const auto start_time = metrics_list_.front().timestamp;
    const auto end_time = metrics_list_.back().timestamp;
    summary_.total_time = (end_time - start_time).seconds();
  }
}

nlohmann::json ClosedLoopEvaluator::get_summary_as_json() const
{
  nlohmann::json j;

  // Position metrics
  j["position_metrics"]["lateral_error"]["mean"] = summary_.mean_lateral_error;
  j["position_metrics"]["lateral_error"]["max"] = summary_.max_lateral_error;
  j["position_metrics"]["lateral_error"]["std"] = summary_.std_lateral_error;
  j["position_metrics"]["lateral_error"]["unit"] = "meters";

  // Dynamics metrics
  j["dynamics_metrics"]["lateral_acceleration"]["mean"] = summary_.mean_acceleration;
  j["dynamics_metrics"]["lateral_acceleration"]["max"] = summary_.max_acceleration;
  j["dynamics_metrics"]["lateral_acceleration"]["unit"] = "m/s²";

  j["dynamics_metrics"]["jerk"]["mean"] = summary_.mean_jerk;
  j["dynamics_metrics"]["jerk"]["max"] = summary_.max_jerk;
  j["dynamics_metrics"]["jerk"]["unit"] = "m/s³";

  // Oscillation metrics
  j["oscillation_metrics"]["steering_reversals"] = summary_.steering_reversals;
  j["oscillation_metrics"]["steering_angular_velocity"]["mean"] =
    summary_.mean_steering_angular_velocity;
  j["oscillation_metrics"]["steering_angular_velocity"]["max"] =
    summary_.max_steering_angular_velocity;
  j["oscillation_metrics"]["steering_angular_velocity"]["unit"] = "rad/s";
  j["oscillation_metrics"]["steering_angle_std"] = summary_.std_steering_angle;
  j["oscillation_metrics"]["lateral_jerk"]["mean"] = summary_.mean_lateral_jerk;
  j["oscillation_metrics"]["lateral_jerk"]["max"] = summary_.max_lateral_jerk;
  j["oscillation_metrics"]["lateral_jerk"]["unit"] = "m/s³";

  // Safety metrics
  j["safety_metrics"]["min_ttc"] = summary_.min_ttc;
  j["safety_metrics"]["unit"] = "seconds";

  // Summary info
  j["evaluation_info"]["total_samples"] = summary_.num_samples;
  j["evaluation_info"]["total_time_seconds"] = summary_.total_time;
  j["evaluation_info"]["total_distance_meters"] = summary_.total_distance;

  return j;
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools