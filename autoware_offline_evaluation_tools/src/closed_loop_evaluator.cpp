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
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

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
#include <fstream>
#include <chrono>
#include <filesystem>
#include <iomanip>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
// Constructor implementation moved to header file

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
    bag_writer.write(msg, "/closed_loop/metrics/lateral_error", timestamp);
  }

  // Save acceleration
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = timestamp;
    msg.point.x = metrics.longitudinal_acceleration;
    msg.point.y = metrics.lateral_acceleration;
    msg.point.z = metrics.jerk;
    bag_writer.write(msg, "/closed_loop/metrics/acceleration", timestamp);
  }

  // Save TTC
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.ttc;
    bag_writer.write(msg, "/closed_loop/metrics/ttc", timestamp);
  }

  // Save oscillation metrics
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = timestamp;
    msg.point.x = metrics.steering_angle;
    msg.point.y = metrics.steering_angular_velocity;
    msg.point.z = metrics.lateral_jerk;
    bag_writer.write(msg, "/closed_loop/metrics/steering_velocity", timestamp);
  }

  // Save yaw rate
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.yaw_rate;
    bag_writer.write(msg, "/closed_loop/metrics/jerk", timestamp);
  }

  // Save localization (Odometry)
  if (sync_data && sync_data->kinematic_state) {
    bag_writer.write(*sync_data->kinematic_state, "/closed_loop/kinematic_state", timestamp);
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

nlohmann::json ClosedLoopEvaluator::get_detailed_results_as_json() const
{
  nlohmann::json j;
  j["summary"] = get_summary_as_json();
  
  // Add detailed trajectory-by-trajectory metrics if needed
  nlohmann::json trajectories = nlohmann::json::array();
  for (const auto & metrics : metrics_list_) {
    nlohmann::json traj;
    traj["lateral_error"] = metrics.lateral_error;
    traj["longitudinal_error"] = metrics.longitudinal_error;
    traj["lateral_acceleration"] = metrics.lateral_acceleration;
    traj["curvature"] = metrics.curvature;
    traj["acceleration"] = metrics.longitudinal_acceleration;
    traj["jerk"] = metrics.jerk;
    traj["ttc"] = metrics.ttc;
    trajectories.push_back(traj);
  }
  j["trajectories"] = trajectories;
  
  return j;
}

std::vector<std::pair<std::string, std::string>> ClosedLoopEvaluator::get_result_topics() const
{
  return {
    {"/closed_loop/metrics/lateral_error", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/heading_error", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/velocity_error", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/acceleration", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/jerk", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/steering_velocity", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/ttc", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/comfort_score", "std_msgs/msg/Float64"},
    {"/closed_loop/trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {"/closed_loop/kinematic_state", "nav_msgs/msg/Odometry"},
    {"/closed_loop/objects", "autoware_perception_msgs/msg/PredictedObjects"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
    {"/tf_static", "tf2_msgs/msg/TFMessage"}
  };
}

std::pair<rclcpp::Time, rclcpp::Time> ClosedLoopEvaluator::run_evaluation_from_bag(
  const std::string & bag_path,
  rosbag2_cpp::Writer * evaluation_bag_writer,
  const TopicNames & topic_names)
{
  RCLCPP_INFO(logger_, "Running closed-loop evaluation for autonomous driving data");

  // Open bag reader
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(bag_path);

  // Create bag data handler
  const double buffer_duration_sec = 20.0;  // TODO: make configurable
  const size_t max_buffer_messages = 10000;

  auto bag_data = std::make_shared<BagData>(0, topic_names, buffer_duration_sec, max_buffer_messages);

  // Read all messages from bag into buffers
  RCLCPP_INFO(logger_, "Loading rosbag data into buffers...");
  
  // Get option to use bag timestamp instead of header timestamp
  const bool use_bag_timestamp = true;  // TODO: make configurable

  while (bag_reader.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader.read_next();
    const auto & topic_name = serialized_message->topic_name;

    // Process messages using template helper
    if (topic_name == topic_names.odometry_topic) {
      process_and_append_message<Odometry>(
        serialized_message, bag_data, topic_names.odometry_topic, use_bag_timestamp, logger_);
    } 
    else if (topic_name == topic_names.trajectory_topic) {
      process_and_append_message<Trajectory>(
        serialized_message, bag_data, topic_names.trajectory_topic, use_bag_timestamp, logger_);
    } 
    else if (topic_name == topic_names.acceleration_topic) {
      process_and_append_message<AccelWithCovarianceStamped>(
        serialized_message, bag_data, topic_names.acceleration_topic, use_bag_timestamp, logger_);
    } 
    else if (topic_name == topic_names.steering_topic) {
      // SteeringReport doesn't have header, so we don't override timestamp
      process_and_append_message<SteeringReport>(
        serialized_message, bag_data, topic_names.steering_topic, false, logger_);
    } 
    else if (topic_name == topic_names.objects_topic) {
      process_and_append_message<PredictedObjects>(
        serialized_message, bag_data, topic_names.objects_topic, use_bag_timestamp, logger_);
      
      // Also write objects to evaluation bag
      if (evaluation_bag_writer) {
        try {
          PredictedObjects msg;
          rclcpp::Serialization<PredictedObjects> serializer;
          rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
          serializer.deserialize_message(&serialized_msg, &msg);
          rclcpp::Time msg_time(serialized_message->time_stamp);
          evaluation_bag_writer->write(msg, "/closed_loop/objects", msg_time);
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger_, "Failed to write objects to evaluation bag: %s", e.what());
        }
      }
    } 
    else if (topic_name == topic_names.tf_topic) {
      process_and_append_message<TFMessage>(
        serialized_message, bag_data, topic_names.tf_topic, false, logger_);
      
      // Also write tf messages to evaluation bag
      if (evaluation_bag_writer) {
        try {
          TFMessage msg;
          rclcpp::Serialization<TFMessage> serializer;
          rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
          serializer.deserialize_message(&serialized_msg, &msg);
          rclcpp::Time msg_time(serialized_message->time_stamp);
          evaluation_bag_writer->write(msg, "/tf", msg_time);
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger_, "Failed to write tf to evaluation bag: %s", e.what());
        }
      }
    }
  }

  // Get kinematic states at 100ms intervals
  const double evaluation_interval_ms = 100.0;  // TODO: make configurable
  auto kinematic_states = bag_data->get_kinematic_states_at_interval(evaluation_interval_ms);

  if (kinematic_states.empty()) {
    RCLCPP_ERROR(logger_, "No kinematic states found in the rosbag");
    return {rclcpp::Clock{RCL_ROS_TIME}.now(), rclcpp::Clock{RCL_ROS_TIME}.now()};
  }

  // Process each kinematic state with synchronized data
  std::vector<std::shared_ptr<SynchronizedData>> synchronized_data_list;
  const double sync_tolerance_ms = 50.0;  // TODO: make configurable

  for (const auto & kinematic_state : kinematic_states) {
    const auto timestamp = rclcpp::Time(kinematic_state->header.stamp).nanoseconds();
    auto sync_data = bag_data->get_synchronized_data_at_time(timestamp, sync_tolerance_ms);

    if (sync_data && sync_data->trajectory) {
      synchronized_data_list.push_back(sync_data);
    }
  }

  // Evaluate the synchronized data
  if (!synchronized_data_list.empty()) {
    if (evaluation_bag_writer) {
      // Create topics for closed-loop evaluation
      const auto topics = get_result_topics();
      for (const auto & [topic_name, topic_type] : topics) {
        const auto topic_info = rosbag2_storage::TopicMetadata{
          topic_name, topic_type, rmw_get_serialization_format(), ""};
        evaluation_bag_writer->create_topic(topic_info);
      }
    }
    
    evaluate(synchronized_data_list, evaluation_bag_writer);

    // Get and log summary
    auto summary = get_summary();
    RCLCPP_INFO(
      logger_,
      "Evaluation Summary:\n"
      "  Total samples: %zu\n"
      "  Mean lateral error: %.3f m\n"
      "  Max lateral error: %.3f m\n"
      "  Std lateral error: %.3f m\n"
      "  Mean acceleration: %.3f m/s²\n"
      "  Max acceleration: %.3f m/s²\n"
      "  Steering reversals: %zu\n"
      "  Mean steering angular velocity: %.3f rad/s\n"
      "  Min TTC: %.3f s\n"
      "  Total time: %.3f s",
      summary.num_samples, summary.mean_lateral_error, summary.max_lateral_error,
      summary.std_lateral_error, summary.mean_acceleration, summary.max_acceleration,
      summary.steering_reversals, summary.mean_steering_angular_velocity, summary.min_ttc,
      summary.total_time);

    // Save JSON output
    const std::string json_output_path = "~/evaluation_result.json";  // TODO: make configurable
    std::string expanded_path = json_output_path;

    // Expand home directory if needed
    if (expanded_path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        expanded_path = std::string(home) + expanded_path.substr(1);
      }
    }

    // Add timestamp to filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp_ss;
    timestamp_ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

    // Create filename with timestamp
    std::filesystem::path json_path(expanded_path);
    std::string filename = json_path.stem().string() + "_" + timestamp_ss.str() + ".json";
    json_path = json_path.parent_path() / filename;

    // Get JSON summary
    nlohmann::json json_output = get_summary_as_json();

    // Add evaluation info
    json_output["evaluation_info"]["timestamp"] = timestamp_ss.str();
    json_output["evaluation_info"]["bag_path"] = bag_path;
    json_output["evaluation_info"]["evaluation_mode"] = "closed_loop";

    // Write JSON file
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
      json_file << json_output.dump(2);  // Pretty print with 2 spaces
      json_file.close();
      RCLCPP_INFO(logger_, "JSON results saved to: %s", json_path.c_str());
    } else {
      RCLCPP_ERROR(logger_, "Failed to save JSON results to: %s", json_path.c_str());
    }
  }

  RCLCPP_INFO(logger_, "Closed-loop evaluation complete");

  // Return the timestamps of the first and last evaluation data
  if (!kinematic_states.empty()) {
    return {
      rclcpp::Time(kinematic_states.front()->header.stamp),
      rclcpp::Time(kinematic_states.back()->header.stamp)};
  }
  return {rclcpp::Clock{RCL_ROS_TIME}.now(), rclcpp::Clock{RCL_ROS_TIME}.now()};
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools