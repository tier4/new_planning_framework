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
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory_selector_common/utils.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
  trajectory_point_metrics_list_.clear();
  trajectory_point_metrics_list_.reserve(synchronized_data_list.size());
  
  // Reset normalized timestamp tracking for new evaluation
  first_eval_timestamp_set_ = false;

  std::shared_ptr<SynchronizedData> previous_data = nullptr;

  for (const auto & sync_data : synchronized_data_list) {
    if (!sync_data->kinematic_state || !sync_data->trajectory) {
      continue;
    }

    auto metrics = calculate_metrics(sync_data, previous_data);
    metrics_list_.push_back(metrics);

    // Calculate trajectory point metrics
    auto trajectory_metrics = calculate_trajectory_point_metrics(sync_data);
    trajectory_point_metrics_list_.push_back(trajectory_metrics);

    if (bag_writer) {
      save_metrics_to_bag(metrics, sync_data, *bag_writer);
      
      // Calculate normalized timestamp for trajectory point metrics
      if (!first_eval_timestamp_set_) {
        first_eval_timestamp_ = sync_data->bag_timestamp;
        first_eval_timestamp_set_ = true;
      }
      const auto relative_duration = sync_data->bag_timestamp - first_eval_timestamp_;
      const rclcpp::Time normalized_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME) + relative_duration;
      
      save_trajectory_point_metrics_to_bag(trajectory_metrics, sync_data, *bag_writer, 
                                         normalized_timestamp);
    }

    previous_data = sync_data;
  }

  calculate_summary();
}

ClosedLoopTrajectoryMetrics ClosedLoopEvaluator::calculate_metrics(
  const std::shared_ptr<SynchronizedData> & current_data,
  const std::shared_ptr<SynchronizedData> & previous_data)
{
  ClosedLoopTrajectoryMetrics metrics;
  metrics.timestamp = current_data->timestamp;

  const auto & current_pose = current_data->kinematic_state->pose.pose;
  const auto & current_twist = current_data->kinematic_state->twist.twist;
  //const auto & trajectory = *current_data->trajectory;

  metrics.lateral_error = calculate_lateral_error_from_preferred_lane(current_pose);

  if(current_data->kinematic_state) {
    metrics.longitudinal_velocity = current_data->kinematic_state->twist.twist.linear.x;
    metrics.yaw_rate = current_data->kinematic_state->twist.twist.angular.z;
  } else {
    metrics.longitudinal_velocity = 0.0;
    metrics.yaw_rate = 0.0;
  }

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

  // Calculate lateral acceleration from velocity and yawrate
  metrics.lateral_acceleration = metrics.longitudinal_velocity * metrics.yaw_rate;  // Simplified for now

  // Calculate oscillation metrics
  calculate_oscillation_metrics(metrics, current_data, previous_data);

  // Calculate TTC if objects are available
  if (current_data->objects) {
    metrics.min_ttc = calculate_ttc(current_pose, current_twist, *current_data->objects);
  } else {
    metrics.min_ttc = std::numeric_limits<double>::max();
  }

  return metrics;
}


void ClosedLoopEvaluator::calculate_oscillation_metrics(
  ClosedLoopTrajectoryMetrics & metrics, const std::shared_ptr<SynchronizedData> & current_data,
  const std::shared_ptr<SynchronizedData> & previous_data)
{
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

void ClosedLoopEvaluator::save_metrics_to_bag(
  const ClosedLoopTrajectoryMetrics & metrics, const std::shared_ptr<SynchronizedData> & sync_data,
  rosbag2_cpp::Writer & bag_writer)
{
  // Use normalized timestamp for bag writing to ensure proper duration
  if (!first_eval_timestamp_set_) {
    first_eval_timestamp_ = sync_data->bag_timestamp;
    first_eval_timestamp_set_ = true;
  }
  
  // Calculate relative timestamp from the first data point
  const auto relative_duration = sync_data->bag_timestamp - first_eval_timestamp_;
  const rclcpp::Time normalized_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME) + relative_duration;

  // Save lateral error
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.lateral_error;
    bag_writer.write(msg, "/closed_loop/lateral_error", normalized_timestamp);
  }

  // Save acceleration
  {
    geometry_msgs::msg::Accel msg;
    msg.linear.x = metrics.longitudinal_acceleration; 
    msg.linear.y = metrics.lateral_acceleration;
    msg.angular.z = 0.0;
    bag_writer.write(msg, "/closed_loop/acceleration", normalized_timestamp);
  }

  // Save TTC
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.min_ttc;
    bag_writer.write(msg, "ttc", normalized_timestamp);
  }

  // Save jerk
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.jerk;
    bag_writer.write(msg, "/closed_loop/jerk", normalized_timestamp);
  }

  // Save steering angle velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = metrics.steering_angular_velocity;
    bag_writer.write(msg, "/closed_loop/steering_velocity", normalized_timestamp);
  }

  // Save localization (Odometry)
  if (sync_data && sync_data->kinematic_state) {
    // Create a copy with normalized timestamp
    nav_msgs::msg::Odometry corrected_odom = *sync_data->kinematic_state;
    corrected_odom.twist.twist.linear.y = metrics.lateral_acceleration;  // Store lateral acceleration
    corrected_odom.header.stamp = normalized_timestamp;
    bag_writer.write(corrected_odom, "/kinematic_state", normalized_timestamp);
  }
  
  // Save objects with normalized timestamp
  if (sync_data && sync_data->objects) {
    PredictedObjects corrected_objects = *sync_data->objects;
    corrected_objects.header.stamp = normalized_timestamp;
    bag_writer.write(corrected_objects, "/closed_loop/objects", normalized_timestamp);
  }
  
  // Save trajectory using base class method
  write_trajectory_to_bag(sync_data, bag_writer, normalized_timestamp);
  
  // Save TF with normalized timestamp
  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped transform;
  
  // Use the normalized timestamp for consistency
  transform.header.stamp = normalized_timestamp;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  
  // Use kinematic state pose as transform
  if (sync_data && sync_data->kinematic_state) {
    transform.transform.translation.x = sync_data->kinematic_state->pose.pose.position.x;
    transform.transform.translation.y = sync_data->kinematic_state->pose.pose.position.y;
    transform.transform.translation.z = sync_data->kinematic_state->pose.pose.position.z;
    transform.transform.rotation = sync_data->kinematic_state->pose.pose.orientation;
    
    tf_msg.transforms.push_back(transform);
    bag_writer.write(tf_msg, "/tf", normalized_timestamp);
  }
}

// Implementations of calculate_trajectory_point_metrics and save_trajectory_point_metrics_to_bag 
// have been moved to base class

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
    sum_steering_angular_velocity += std::abs(metrics.steering_angular_velocity);

    summary_.max_lateral_error = std::max(summary_.max_lateral_error, metrics.lateral_error);
    summary_.max_acceleration =
      std::max(summary_.max_acceleration, std::abs(metrics.lateral_acceleration));
    summary_.max_jerk = std::max(summary_.max_jerk, std::abs(metrics.jerk));
    summary_.max_steering_angular_velocity =
      std::max(summary_.max_steering_angular_velocity, std::abs(metrics.steering_angular_velocity));

    // Count steering reversals (sign changes in steering velocity)
    if (i > 0 && std::abs(metrics.steering_angular_velocity) > 0.01) {  // Threshold to avoid noise
      if (prev_steering_velocity * metrics.steering_angular_velocity < 0) {
        summary_.steering_reversals++;
      }
      prev_steering_velocity = metrics.steering_angular_velocity;
    }

    if (metrics.min_ttc < std::numeric_limits<double>::max()) {
      summary_.min_ttc = std::min(summary_.min_ttc, metrics.min_ttc);
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
  for (size_t i = 0; i < metrics_list_.size(); ++i) {
    const auto & metrics = metrics_list_[i];
    nlohmann::json traj;
    traj["lateral_error"] = metrics.lateral_error;
    traj["lateral_acceleration"] = metrics.lateral_acceleration;
    traj["acceleration"] = metrics.longitudinal_acceleration;
    traj["jerk"] = metrics.jerk;
    traj["ttc"] = metrics.min_ttc;
    
    // Add trajectory point metrics if available
    if (i < trajectory_point_metrics_list_.size()) {
      const auto & point_metrics = trajectory_point_metrics_list_[i];
      traj["trajectory_point_metrics"]["lateral_accelerations"] = point_metrics.lateral_accelerations;
      traj["trajectory_point_metrics"]["longitudinal_jerks"] = point_metrics.longitudinal_jerks;
      traj["trajectory_point_metrics"]["ttc_values"] = point_metrics.ttc_values;
      traj["trajectory_point_metrics"]["lateral_deviations"] = point_metrics.lateral_deviations;
      traj["trajectory_point_metrics"]["travel_distances"] = point_metrics.travel_distances;
    }
    
    trajectories.push_back(traj);
  }
  j["trajectories"] = trajectories;
  
  return j;
}

std::vector<std::pair<std::string, std::string>> ClosedLoopEvaluator::get_result_topics() const
{
  return {
    {"/closed_loop/lateral_error", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/heading_error", "std_msgs/msg/Float64"},
    {"/closed_loop/acceleration", "geometry_msgs/msg/Accel"},
    {"ttc", "std_msgs/msg/Float64"},
    {"/closed_loop/jerk", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/steering_velocity", "std_msgs/msg/Float64"},
    {"/closed_loop/metrics/comfort_score", "std_msgs/msg/Float64"},
    {"/trajectory/lateral_accelerations", "std_msgs/msg/Float64MultiArray"},
    {"/trajectory/longitudinal_jerks", "std_msgs/msg/Float64MultiArray"},
    {"/trajectory/ttc_values", "std_msgs/msg/Float64MultiArray"},
    {"/trajectory/lateral_deviations", "std_msgs/msg/Float64MultiArray"},
    {"/trajectory/travel_distances", "std_msgs/msg/Float64MultiArray"},
    {"/trajectory", "autoware_planning_msgs/msg/Trajectory"},
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

  // Use base class method to process bag and get synchronized data
  auto bag_result = process_bag_common(bag_path, evaluation_bag_writer, topic_names);
  
  // Evaluate the synchronized data
  if (bag_result.synchronized_data_list.empty()) {
    RCLCPP_ERROR(logger_, "Data synchronization failed. Aborting evaluation.");
    return {rclcpp::Time(), rclcpp::Time()};
  }
  if (evaluation_bag_writer) {
    // Create topics for closed-loop evaluation
    create_topics_in_bag(*evaluation_bag_writer);
  }
  
  evaluate(bag_result.synchronized_data_list, evaluation_bag_writer);

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

  // Save JSON output using base class method
  save_json_results(get_summary_as_json(), bag_path, "closed_loop", "evaluation_result");

  RCLCPP_INFO(logger_, "Closed-loop evaluation complete");
  
  return {bag_result.evaluation_start_time, bag_result.evaluation_end_time};
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools