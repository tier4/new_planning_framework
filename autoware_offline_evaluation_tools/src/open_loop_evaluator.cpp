// Copyright 2025 TIER IV, Inc.
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

#include "open_loop_evaluator.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory_selector_common/utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

// Template helper for statistics calculation
template<typename Container>
struct Statistics {
  double mean = 0.0;
  double std_dev = 0.0;
  double max_val = 0.0;
};

template<typename Container>
Statistics<Container> calculate_statistics(const Container& values) {
  Statistics<Container> stats;
  
  if (values.empty()) {
    return stats;
  }
  
  // Calculate mean
  stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  
  // Calculate max
  stats.max_val = *std::max_element(values.begin(), values.end());
  
  // Calculate standard deviation
  double variance = 0.0;
  for (const auto& val : values) {
    variance += (val - stats.mean) * (val - stats.mean);
  }
  stats.std_dev = std::sqrt(variance / values.size());
  
  return stats;
}

// Constructor implementation moved to header file

void OpenLoopEvaluator::evaluate(
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
  rosbag2_cpp::Writer * bag_writer)
{
  metrics_list_.clear();
  // Reset normalized timestamp tracking for new evaluation
  first_bag_timestamp_set_ = false;
  
  // Get the base timestamp for relative time calculation
  rclcpp::Time base_timestamp;
  rclcpp::Time bag_base_timestamp;
  if (!synchronized_data_list.empty()) {
    base_timestamp = synchronized_data_list.front()->timestamp;
    bag_base_timestamp = synchronized_data_list.front()->bag_timestamp;
  } else {
    base_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bag_base_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  
  // For each trajectory in the data, evaluate against future ground truth
  for (size_t i = 0; i < synchronized_data_list.size(); ++i) {
    const auto & current_data = synchronized_data_list[i];
    
    // Skip if no trajectory available
    if (!current_data->trajectory) {
      continue;
    }

    const auto & trajectory = *(current_data->trajectory);
    if (trajectory.points.empty()) {
      continue;
    }
    
    // Evaluate trajectory
    auto metrics = evaluate_trajectory(current_data, synchronized_data_list);
    metrics_list_.push_back(metrics);
    
    // Save to bag if writer provided
    if (bag_writer) {
      save_metrics_to_bag(metrics, current_data, *bag_writer);
    } else {
      RCLCPP_WARN(logger_, "No bag writer provided, metrics not saved to bag");
    }
  }
  
  // Calculate summary statistics
  calculate_summary();
  
  RCLCPP_INFO(logger_, "Overall: Mean ADE=%.3fm (±%.3fm), Mean FDE=%.3fm (±%.3fm)",
    summary_.mean_ade, summary_.std_ade,
    summary_.mean_fde, summary_.std_fde);
}

OpenLoopTrajectoryMetrics OpenLoopEvaluator::evaluate_trajectory(
  const std::shared_ptr<SynchronizedData> & trajectory_data,
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list)
{
  OpenLoopTrajectoryMetrics metrics;
  
  const auto & trajectory = *(trajectory_data->trajectory);
  metrics.num_points = trajectory.points.size();
  metrics.trajectory_timestamp = trajectory_data->timestamp;
  
  if (metrics.num_points == 0) {
    return metrics;
  }
  
  // Initialize vectors
  metrics.lateral_deviations.resize(metrics.num_points, 0.0);
  metrics.longitudinal_deviations.resize(metrics.num_points, 0.0);
  metrics.displacement_errors.resize(metrics.num_points, 0.0);
  metrics.ground_truth_available.resize(metrics.num_points, false);
  metrics.ground_truth_poses.resize(metrics.num_points);
  metrics.ttc_values.resize(metrics.num_points, std::numeric_limits<double>::max());
  metrics.min_ttc = std::numeric_limits<double>::max();
  metrics.ttc_at_2s = std::numeric_limits<double>::max();
  
  // Evaluate each trajectory point
  for (size_t i = 0; i < metrics.num_points; ++i) {
    const auto & traj_point = trajectory.points[i];
    const auto point_time = trajectory_data->timestamp + 
      rclcpp::Duration(traj_point.time_from_start);
    
    
    // Interpolate ground truth at this time
    auto gt_pose_opt = interpolate_ground_truth(point_time, synchronized_data_list);
    
    if (!gt_pose_opt) {
      continue;
    }
    
    const auto & gt_pose = gt_pose_opt.value();
    metrics.ground_truth_available[i] = true;
    metrics.ground_truth_poses[i] = gt_pose;
    
    // Calculate displacement error (still useful as overall error)
    metrics.displacement_errors[i] = autoware_utils_geometry::calc_distance2d(
      traj_point.pose.position, gt_pose.position);
    
    // Calculate errors in vehicle coordinate frame
    const auto [longitudinal_error, lateral_error] = 
      calculate_errors_in_vehicle_frame(traj_point.pose, gt_pose);
    
    metrics.longitudinal_deviations[i] = longitudinal_error;
    metrics.lateral_deviations[i] = lateral_error;
    
    // TTC calculation
    // Find objects data at this time
    std::shared_ptr<PredictedObjects> objects_at_time;
    for (const auto & data : synchronized_data_list) {
      if (data->timestamp <= point_time && 
          data->objects) {
        // Use the most recent objects data before or at this time
        objects_at_time = data->objects;
      }
      if (data->timestamp > point_time) {
        break;
      }
    }

    
    // Calculate TTC for this trajectory point
    if (objects_at_time && !objects_at_time->objects.empty()) {
      double ttc_at_point = std::numeric_limits<double>::max();
      
      for (const auto & object : objects_at_time->objects) {
        const double ttc = autoware::trajectory_selector::utils::time_to_collision(
          traj_point, traj_point.time_from_start, object);
        
        if (std::isfinite(ttc) && ttc > 0.0) {
          ttc_at_point = std::min(ttc_at_point, ttc);
        }
      }
      
      metrics.ttc_values[i] = ttc_at_point;
      metrics.min_ttc = std::min(metrics.min_ttc, ttc_at_point);
    }
    
    // Check if this is the 2-second point
    const double time_from_start_seconds = rclcpp::Duration(traj_point.time_from_start).seconds();
    if (std::abs(time_from_start_seconds - 2.0) < 0.1) {  // Within 100ms of 2 seconds
      // Store TTC at 2s (placeholder - actual TTC calculation would be done by metrics system)
      if (i < metrics.ttc_values.size() && metrics.ttc_values[i] < std::numeric_limits<double>::max()) {
        metrics.ttc_at_2s = metrics.ttc_values[i];
      }
    }
  }
  
  // Calculate aggregate metrics
  metrics.num_valid_comparisons = std::count(
    metrics.ground_truth_available.begin(),
    metrics.ground_truth_available.end(), true);
  
  if (metrics.num_valid_comparisons > 0) {
    // ADE (Average Displacement Error)
    double sum_displacement = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < metrics.num_points; ++i) {
      if (metrics.ground_truth_available[i]) {
        sum_displacement += metrics.displacement_errors[i];
        count++;
      }
    }
    metrics.ade = sum_displacement / count;
    
    // FDE (Final Displacement Error) - last valid comparison
    for (int i = metrics.num_points - 1; i >= 0; --i) {
      if (metrics.ground_truth_available[i]) {
        metrics.fde = metrics.displacement_errors[i];
        break;
      }
    }
    
    // Lateral deviation statistics
    double sum_lateral = 0.0;
    double sum_lateral_sq = 0.0;
    double max_lateral = 0.0;
    count = 0;
    
    for (size_t i = 0; i < metrics.num_points; ++i) {
      if (metrics.ground_truth_available[i]) {
        const double abs_lateral = std::abs(metrics.lateral_deviations[i]);
        sum_lateral += abs_lateral;
        sum_lateral_sq += abs_lateral * abs_lateral;
        max_lateral = std::max(max_lateral, abs_lateral);
        count++;
      }
    }
    
    if (count > 0) {
      metrics.mean_lateral_deviation = sum_lateral / count;
      metrics.max_lateral_deviation = max_lateral;
      
      if (count > 1) {
        const double variance = 
          (sum_lateral_sq - sum_lateral * sum_lateral / count) / (count - 1);
        metrics.std_lateral_deviation = std::sqrt(variance);
      }
    }
  }
  
  // Calculate trajectory duration
  if (!trajectory.points.empty()) {
    metrics.trajectory_duration = 
      rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
  }
  
  // Set evaluation time window
  metrics.evaluation_start_time = trajectory_data->timestamp;
  if (!trajectory.points.empty()) {
    metrics.evaluation_end_time = trajectory_data->timestamp + 
      rclcpp::Duration(trajectory.points.back().time_from_start);
  }
  
  return metrics;
}


// calculate_distance_2d moved to base class

std::pair<double, double> OpenLoopEvaluator::calculate_errors_in_vehicle_frame(
  const geometry_msgs::msg::Pose & trajectory_pose,
  const geometry_msgs::msg::Pose & ground_truth_pose)
{
  // Get ground truth yaw angle
  const double gt_yaw = tf2::getYaw(ground_truth_pose.orientation);
  
  // Calculate position difference in global frame
  const double dx_global = trajectory_pose.position.x - ground_truth_pose.position.x;
  const double dy_global = trajectory_pose.position.y - ground_truth_pose.position.y;
  
  // Transform to vehicle coordinate frame (ground truth vehicle frame)
  // Rotate by -gt_yaw to align with vehicle frame
  const double cos_yaw = std::cos(-gt_yaw);
  const double sin_yaw = std::sin(-gt_yaw);
  
  const double dx_vehicle = dx_global * cos_yaw - dy_global * sin_yaw;  // longitudinal
  const double dy_vehicle = dx_global * sin_yaw + dy_global * cos_yaw;  // lateral
  
  return std::make_pair(dx_vehicle, dy_vehicle);
}

std::optional<geometry_msgs::msg::Pose> OpenLoopEvaluator::interpolate_ground_truth(
  const rclcpp::Time & target_time,
  const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data)
{
  if (ground_truth_data.size() < 2) {
    return std::nullopt;
  }
  
  // Find bracketing indices
  size_t lower_idx = 0;
  size_t upper_idx = ground_truth_data.size() - 1;
  
  // Check bounds
  if (target_time < ground_truth_data.front()->timestamp ||
      target_time > ground_truth_data.back()->timestamp) {
    return std::nullopt;
  }
  
  // Binary search for bracketing indices
  while (upper_idx - lower_idx > 1) {
    const size_t mid_idx = (lower_idx + upper_idx) / 2;
    if (ground_truth_data[mid_idx]->timestamp <= target_time) {
      lower_idx = mid_idx;
    } else {
      upper_idx = mid_idx;
    }
  }
  
  const auto & lower_data = ground_truth_data[lower_idx];
  const auto & upper_data = ground_truth_data[upper_idx];
  
  // Calculate interpolation ratio
  const double dt_total = (upper_data->timestamp - lower_data->timestamp).seconds();
  const double dt_target = (target_time - lower_data->timestamp).seconds();
  const double ratio = dt_target / dt_total;
  
  // Interpolate position
  geometry_msgs::msg::Pose interpolated_pose;
  const auto & p1 = lower_data->kinematic_state->pose.pose.position;
  const auto & p2 = upper_data->kinematic_state->pose.pose.position;
  
  interpolated_pose.position.x = p1.x + ratio * (p2.x - p1.x);
  interpolated_pose.position.y = p1.y + ratio * (p2.y - p1.y);
  interpolated_pose.position.z = p1.z + ratio * (p2.z - p1.z);
  
  // For orientation, use slerp (simplified to linear interpolation of yaw for 2D case)
  const double yaw1 = tf2::getYaw(lower_data->kinematic_state->pose.pose.orientation);
  const double yaw2 = tf2::getYaw(upper_data->kinematic_state->pose.pose.orientation);
  
  // Handle angle wrapping
  double yaw_diff = yaw2 - yaw1;
  while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
  
  const double interpolated_yaw = yaw1 + ratio * yaw_diff;
  
  // Convert back to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, interpolated_yaw);
  interpolated_pose.orientation = tf2::toMsg(q);
  
  return interpolated_pose;
}

void OpenLoopEvaluator::save_metrics_to_bag(
  const OpenLoopTrajectoryMetrics & metrics,
  const std::shared_ptr<SynchronizedData> & trajectory_data,
  rosbag2_cpp::Writer & bag_writer)
{
  // Use a normalized timestamp for bag writing to ensure proper duration
  // Start from 0 and use relative times from the first synchronized data point
  if (!first_bag_timestamp_set_) {
    first_bag_timestamp_ = trajectory_data->bag_timestamp;
    first_bag_timestamp_set_ = true;
  }
  
  // Calculate relative timestamp from the first data point
  const auto relative_duration = trajectory_data->bag_timestamp - first_bag_timestamp_;
  const rclcpp::Time normalized_timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME) + relative_duration;
  // Log that we're saving metrics
  RCLCPP_DEBUG(logger_, "Saving metrics to bag: ADE=%.3f, FDE=%.3f at normalized time %.3f",
    metrics.ade, metrics.fde, normalized_timestamp.seconds());
  
  // Write individual metrics as Float64 messages
  std_msgs::msg::Float64 metric_msg;
  
  // ADE
  metric_msg.data = metrics.ade;
  bag_writer.write(metric_msg, "/open_loop/metrics/ade", normalized_timestamp);
  
  // FDE
  metric_msg.data = metrics.fde;
  bag_writer.write(metric_msg, "/open_loop/metrics/fde", normalized_timestamp);
  
  // Mean lateral deviation
  metric_msg.data = metrics.mean_lateral_deviation;
  bag_writer.write(metric_msg, "/open_loop/metrics/mean_lateral_deviation", normalized_timestamp);
  
  // Max lateral deviation
  metric_msg.data = metrics.max_lateral_deviation;
  bag_writer.write(metric_msg, "/open_loop/metrics/max_lateral_deviation", normalized_timestamp);
  
  // Min TTC
  metric_msg.data = metrics.min_ttc;
  bag_writer.write(metric_msg, "/open_loop/metrics/min_ttc", normalized_timestamp);
  
  // TTC at 2 seconds
  metric_msg.data = metrics.ttc_at_2s;
  bag_writer.write(metric_msg, "/open_loop/metrics/ttc_at_2s", normalized_timestamp);
  
  // Coverage ratio
  metric_msg.data = static_cast<double>(metrics.num_valid_comparisons) / metrics.num_points;
  bag_writer.write(metric_msg, "/open_loop/metrics/coverage_ratio", normalized_timestamp);
  
  // Write point-wise metrics as Float64MultiArray
  std_msgs::msg::Float64MultiArray array_msg;
  
  // Displacement errors array
  array_msg.data = metrics.displacement_errors;
  bag_writer.write(array_msg, "/open_loop/metrics/displacement_errors_array", normalized_timestamp);
  
  // Lateral deviations array
  array_msg.data = metrics.lateral_deviations;
  bag_writer.write(array_msg, "/open_loop/metrics/lateral_deviations_array", normalized_timestamp);
  
  // Longitudinal deviations array
  array_msg.data = metrics.longitudinal_deviations;
  bag_writer.write(array_msg, "/open_loop/metrics/longitudinal_deviations_array", normalized_timestamp);
  
  // TTC values array
  array_msg.data = metrics.ttc_values;
  bag_writer.write(array_msg, "/open_loop/metrics/ttc_values_array", normalized_timestamp);
  
  // Write minimal TF for visualization (map -> base_link)
  if (trajectory_data->kinematic_state) {
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped transform;
    
    // Use the normalized timestamp for consistency
    transform.header.stamp = normalized_timestamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    
    // Use kinematic state pose as transform
    transform.transform.translation.x = trajectory_data->kinematic_state->pose.pose.position.x;
    transform.transform.translation.y = trajectory_data->kinematic_state->pose.pose.position.y;
    transform.transform.translation.z = trajectory_data->kinematic_state->pose.pose.position.z;
    transform.transform.rotation = trajectory_data->kinematic_state->pose.pose.orientation;
    
    tf_msg.transforms.push_back(transform);
    
    // Write TF with the normalized timestamp for consistency
    bag_writer.write(tf_msg, "/tf", normalized_timestamp);
    
    // Debug log
    RCLCPP_DEBUG(logger_, "Writing TF: map->base_link at normalized time %.3f (x=%.2f, y=%.2f, z=%.2f)",
      normalized_timestamp.seconds(),
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
  }
  
  // Save the original trajectory with normalized timestamp
  if (trajectory_data->trajectory) {
    // Create a copy with normalized timestamp
    autoware_planning_msgs::msg::Trajectory corrected_trajectory = *(trajectory_data->trajectory);
    corrected_trajectory.header.stamp = normalized_timestamp;
    
    bag_writer.write(
      corrected_trajectory, "/open_loop/original_trajectory",
      normalized_timestamp);
    
    // Create and save ground truth trajectory
    autoware_planning_msgs::msg::Trajectory gt_trajectory;
    gt_trajectory.header.stamp = normalized_timestamp;
    gt_trajectory.header.frame_id = corrected_trajectory.header.frame_id;
    
    // Convert ground truth poses to trajectory points
    for (size_t i = 0; i < metrics.num_points; ++i) {
      if (metrics.ground_truth_available[i]) {
        autoware_planning_msgs::msg::TrajectoryPoint point;
        point.pose = metrics.ground_truth_poses[i];
        point.time_from_start = trajectory_data->trajectory->points[i].time_from_start;
        
        // Copy velocity if available from original trajectory
        if (i < trajectory_data->trajectory->points.size()) {
          point.longitudinal_velocity_mps = trajectory_data->trajectory->points[i].longitudinal_velocity_mps;
          point.lateral_velocity_mps = trajectory_data->trajectory->points[i].lateral_velocity_mps;
          point.heading_rate_rps = trajectory_data->trajectory->points[i].heading_rate_rps;
        }
        
        gt_trajectory.points.push_back(point);
      }
    }
    
    // Save ground truth trajectory
    if (!gt_trajectory.points.empty()) {
      bag_writer.write(
        gt_trajectory, "/open_loop/ground_truth_trajectory",
        normalized_timestamp);
    }
    
  }
}


void OpenLoopEvaluator::calculate_summary()
{
  summary_ = OpenLoopEvaluationSummary{};
  
  if (metrics_list_.empty()) {
    return;
  }
  
  // Collect valid metrics
  std::vector<double> ade_values;
  std::vector<double> fde_values;
  std::vector<double> lateral_dev_values;
  std::vector<double> coverage_ratios;
  
  for (const auto & metrics : metrics_list_) {
    if (metrics.num_valid_comparisons > 0) {
      ade_values.push_back(metrics.ade);
      fde_values.push_back(metrics.fde);
      lateral_dev_values.push_back(metrics.mean_lateral_deviation);
      coverage_ratios.push_back(
        static_cast<double>(metrics.num_valid_comparisons) / metrics.num_points);
      
      if (metrics.num_valid_comparisons == metrics.num_points) {
        summary_.fully_valid_trajectories++;
      }
    }
  }
  
  summary_.total_trajectories = metrics_list_.size();
  summary_.valid_trajectories = ade_values.size();
  
  if (!ade_values.empty()) {
    // ADE statistics
    const auto ade_stats = calculate_statistics(ade_values);
    summary_.mean_ade = ade_stats.mean;
    summary_.std_ade = ade_stats.std_dev;
    summary_.max_ade = ade_stats.max_val;
    
    // FDE statistics
    const auto fde_stats = calculate_statistics(fde_values);
    summary_.mean_fde = fde_stats.mean;
    summary_.std_fde = fde_stats.std_dev;
    summary_.max_fde = fde_stats.max_val;
    
    // Lateral deviation statistics
    const auto lateral_stats = calculate_statistics(lateral_dev_values);
    summary_.mean_lateral_deviation = lateral_stats.mean;
    summary_.std_lateral_deviation = lateral_stats.std_dev;
    summary_.max_lateral_deviation = lateral_stats.max_val;
    
    // Coverage statistics
    summary_.mean_coverage_ratio = std::accumulate(
      coverage_ratios.begin(), coverage_ratios.end(), 0.0) / coverage_ratios.size();
  }
  
  // Calculate total evaluation duration
  if (!metrics_list_.empty()) {
    const auto start_time = metrics_list_.front().trajectory_timestamp;
    const auto end_time = metrics_list_.back().evaluation_end_time;
    summary_.total_evaluation_duration = (end_time - start_time).seconds();
  }
}

nlohmann::json OpenLoopEvaluator::get_summary_as_json() const
{
  nlohmann::json j;
  
  j["total_trajectories"] = summary_.total_trajectories;
  j["valid_trajectories"] = summary_.valid_trajectories;
  j["fully_valid_trajectories"] = summary_.fully_valid_trajectories;
  j["mean_coverage_ratio"] = summary_.mean_coverage_ratio;
  j["total_evaluation_duration_sec"] = summary_.total_evaluation_duration;
  
  j["ade"]["mean"] = summary_.mean_ade;
  j["ade"]["std"] = summary_.std_ade;
  j["ade"]["max"] = summary_.max_ade;
  
  j["fde"]["mean"] = summary_.mean_fde;
  j["fde"]["std"] = summary_.std_fde;
  j["fde"]["max"] = summary_.max_fde;
  
  j["lateral_deviation"]["mean"] = summary_.mean_lateral_deviation;
  j["lateral_deviation"]["std"] = summary_.std_lateral_deviation;
  j["lateral_deviation"]["max"] = summary_.max_lateral_deviation;
  
  return j;
}

nlohmann::json OpenLoopEvaluator::get_detailed_results_as_json() const
{
  nlohmann::json j;
  
  j["summary"] = get_summary_as_json();
  
  nlohmann::json trajectories = nlohmann::json::array();
  for (const auto & metrics : metrics_list_) {
    nlohmann::json traj;
    
    traj["timestamp_sec"] = metrics.trajectory_timestamp.seconds();
    traj["num_points"] = metrics.num_points;
    traj["num_valid_comparisons"] = metrics.num_valid_comparisons;
    traj["trajectory_duration_sec"] = metrics.trajectory_duration;
    
    traj["ade"] = metrics.ade;
    traj["fde"] = metrics.fde;
    traj["mean_lateral_deviation"] = metrics.mean_lateral_deviation;
    traj["max_lateral_deviation"] = metrics.max_lateral_deviation;
    traj["std_lateral_deviation"] = metrics.std_lateral_deviation;
    traj["min_ttc"] = metrics.min_ttc;
    
    // Include point-wise data if needed
    traj["lateral_deviations"] = metrics.lateral_deviations;
    traj["displacement_errors"] = metrics.displacement_errors;
    
    trajectories.push_back(traj);
  }
  
  j["trajectories"] = trajectories;
  
  return j;
}

std::vector<std::pair<std::string, std::string>> OpenLoopEvaluator::get_result_topics() const
{
  return {
    {"/open_loop/metrics/ade", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/fde", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/mean_lateral_deviation", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/max_lateral_deviation", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/min_ttc", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/ttc_at_2s", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/coverage_ratio", "std_msgs/msg/Float64"},
    {"/open_loop/metrics/displacement_errors_array", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/lateral_deviations_array", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/longitudinal_deviations_array", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/metrics/ttc_values_array", "std_msgs/msg/Float64MultiArray"},
    {"/open_loop/original_trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {"/open_loop/ground_truth_trajectory", "autoware_planning_msgs/msg/Trajectory"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
    {"/tf_static", "tf2_msgs/msg/TFMessage"}
  };
}

std::pair<rclcpp::Time, rclcpp::Time> OpenLoopEvaluator::run_evaluation_from_bag(
  const std::string & bag_path,
  rosbag2_cpp::Writer * evaluation_bag_writer,
  const TopicNames & topic_names)
{
  RCLCPP_INFO(logger_, "Running open-loop evaluation for trajectory analysis");
  
  // Open bag reader
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(bag_path);
  
  // Create bag data handler
  const double buffer_duration_sec = 20.0;  // TODO: make configurable
  const size_t max_buffer_messages = 10000;
  
  auto bag_data = std::make_shared<BagData>(0, topic_names, buffer_duration_sec, max_buffer_messages);
  
  // Find the time range of the bag
  rclcpp::Time bag_start_time = rclcpp::Time(std::numeric_limits<int64_t>::max());
  rclcpp::Time bag_end_time = rclcpp::Time(0);
  
  // tf_static messages
  tf2_msgs::msg::TFMessage tf_static_msgs;
  
  // First pass: scan for time range and collect all data
  while (bag_reader.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader.read_next();
    rclcpp::Time msg_time(serialized_message->time_stamp);
    
    if (msg_time < bag_start_time) bag_start_time = msg_time;
    if (msg_time > bag_end_time) bag_end_time = msg_time;
    
    const auto & topic_name = serialized_message->topic_name;
    
    // Get option to use bag timestamp instead of header timestamp
    const bool use_bag_timestamp = true;  // TODO: make configurable
    
    // Process messages using template helper
    if (topic_name == topic_names.odometry_topic) {
      process_and_append_message<Odometry>(
        serialized_message, bag_data, topic_names.odometry_topic, use_bag_timestamp, logger_);
    }
    else if (topic_name == topic_names.trajectory_topic) {
      process_and_append_message<Trajectory>(
        serialized_message, bag_data, topic_names.trajectory_topic, use_bag_timestamp, logger_);
    }
    else if (topic_name == topic_names.objects_topic) {
      process_and_append_message<PredictedObjects>(
        serialized_message, bag_data, topic_names.objects_topic, use_bag_timestamp, logger_);
    }
    else if (topic_name == topic_names.tf_topic) {
      // TF messages don't have header.stamp, so we don't override timestamp
      process_and_append_message<TFMessage>(
        serialized_message, bag_data, topic_names.tf_topic, false, logger_);
    }
    else if (topic_name == "/tf_static") {
      try {
        tf2_msgs::msg::TFMessage msg;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        // Accumulate all tf_static transforms
        tf_static_msgs.transforms.insert(
          tf_static_msgs.transforms.end(), msg.transforms.begin(), msg.transforms.end());
      } catch (const std::exception & e) {
        RCLCPP_WARN(logger_, "Failed to deserialize tf_static message: %s", e.what());
      }
    }
  }
  
  // Get all data points with synchronized localization and trajectory data
  const double evaluation_interval_ms = 100.0;  // TODO: make configurable
  
  // Collect synchronized data for evaluation
  std::vector<std::shared_ptr<SynchronizedData>> synchronized_data_list;
  const double sync_tolerance_ms = 50.0;  // TODO: make configurable
  
  // Get all kinematic states at regular intervals
  auto kinematic_states = bag_data->get_kinematic_states_at_interval(evaluation_interval_ms);
  
  if (kinematic_states.empty()) {
    RCLCPP_ERROR(logger_, "No kinematic states found in the rosbag");
    return {bag_start_time, bag_end_time};
  }
  
  // For each kinematic state, try to get synchronized data
  for (const auto & kin_state : kinematic_states) {
    const auto timestamp = rclcpp::Time(kin_state->header.stamp).nanoseconds();
    auto sync_data = bag_data->get_synchronized_data_at_time(timestamp, sync_tolerance_ms);
    if (sync_data) {
      synchronized_data_list.push_back(sync_data);
    }
  }
  
  // Sort by timestamp
  std::sort(synchronized_data_list.begin(), synchronized_data_list.end(),
    [](const auto & a, const auto & b) { return a->timestamp < b->timestamp; });
    
  // Write tf_static with normalized timestamp
  if (evaluation_bag_writer && !tf_static_msgs.transforms.empty()) {
    // Write tf_static with normalized timestamp (start from 0)
    rclcpp::Time tf_time(0, 0, RCL_ROS_TIME);
    
    // Also normalize timestamps in the transforms
    tf2_msgs::msg::TFMessage normalized_tf_static = tf_static_msgs;
    for (auto& transform : normalized_tf_static.transforms) {
      transform.header.stamp = tf_time;
    }
    
    evaluation_bag_writer->write(normalized_tf_static, "/tf_static", tf_time);
  }
  
  // Run open-loop evaluation
  if (!synchronized_data_list.empty()) {
    if (evaluation_bag_writer) {
      // Create topics for open-loop evaluation
      const auto topics = get_result_topics();
      for (const auto & [topic_name, topic_type] : topics) {
        const auto topic_info = rosbag2_storage::TopicMetadata{
          topic_name, topic_type, rmw_get_serialization_format(), ""};
        evaluation_bag_writer->create_topic(topic_info);
      }
      evaluate(synchronized_data_list, evaluation_bag_writer);
    } else {
      evaluate(synchronized_data_list, nullptr);
    }
    
    // Get and save evaluation results
    auto summary_json = get_summary_as_json();
    auto detailed_json = get_detailed_results_as_json();
    
    // Write results to file
    const std::string output_dir = ".";  // TODO: make configurable
    const auto json_path = output_dir + "/open_loop_evaluation_results.json";
    
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
      json_file << detailed_json.dump(2);
      json_file.close();
      RCLCPP_INFO(logger_, "Saved evaluation results to: %s", json_path.c_str());
    }
    
    // Log summary
    RCLCPP_INFO(logger_, "Open-loop evaluation summary:");
    if (summary_json.contains("ade") && summary_json["ade"].contains("mean")) {
      RCLCPP_INFO(logger_, "  Mean ADE: %.3f m", 
        static_cast<double>(summary_json["ade"]["mean"]));
      RCLCPP_INFO(logger_, "  Mean FDE: %.3f m", 
        static_cast<double>(summary_json["fde"]["mean"]));
    }
  }
  
  RCLCPP_INFO(logger_, "Open-loop evaluation complete");
  
  // Return the time range of kinematic states (not the entire bag)
  if (!kinematic_states.empty()) {
    rclcpp::Time eval_start_time(kinematic_states.front()->header.stamp);
    rclcpp::Time eval_end_time(kinematic_states.back()->header.stamp);
    return {eval_start_time, eval_end_time};
  }
  
  // Fallback to bag time range if no kinematic states
  // But check if we actually found any messages
  if (bag_start_time.nanoseconds() == std::numeric_limits<int64_t>::max() || 
      bag_end_time.nanoseconds() == 0) {
    // No valid messages found, use current time as fallback
    auto current = rclcpp::Clock{RCL_ROS_TIME}.now();
    return {current, current};
  }
  return {bag_start_time, bag_end_time};
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools