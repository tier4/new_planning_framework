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
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

OpenLoopEvaluator::OpenLoopEvaluator(
  rclcpp::Logger logger,
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler)
: logger_(logger), route_handler_(route_handler)
{
}

void OpenLoopEvaluator::evaluate(
  const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
  rosbag2_cpp::Writer * bag_writer)
{
  RCLCPP_INFO(logger_, "Starting open-loop evaluation with %zu data points",
    synchronized_data_list.size());

  metrics_list_.clear();

  // For each trajectory in the data, evaluate against future ground truth
  for (size_t i = 0; i < synchronized_data_list.size(); ++i) {
    const auto & current_data = synchronized_data_list[i];
    
    // Skip if no trajectory available
    if (!current_data->trajectory) {
      continue;
    }

    // Collect future ground truth data for evaluation
    std::vector<std::shared_ptr<SynchronizedData>> ground_truth_data;
    
    // Calculate trajectory duration
    const auto & trajectory = *(current_data->trajectory);
    if (trajectory.points.empty()) {
      continue;
    }
    
    const auto trajectory_duration = 
      rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    const auto evaluation_end_time = 
      current_data->timestamp + rclcpp::Duration::from_seconds(trajectory_duration * 1.5);
    
    // Collect ground truth data within evaluation window
    for (size_t j = i; j < synchronized_data_list.size(); ++j) {
      if (synchronized_data_list[j]->timestamp > evaluation_end_time) {
        break;
      }
      if (synchronized_data_list[j]->kinematic_state) {
        ground_truth_data.push_back(synchronized_data_list[j]);
      }
    }
    
    if (ground_truth_data.size() < 2) {
      RCLCPP_WARN(logger_, "Insufficient ground truth data for trajectory at time %.3f",
        current_data->timestamp.seconds());
      continue;
    }
    
    // Evaluate trajectory
    auto metrics = evaluate_trajectory(current_data, ground_truth_data);
    metrics_list_.push_back(metrics);
    
    // Save to bag if writer provided
    if (bag_writer) {
      save_metrics_to_bag(metrics, current_data, *bag_writer);
    }
  }
  
  // Calculate summary statistics
  calculate_summary();
  
  RCLCPP_INFO(logger_, "Open-loop evaluation complete. Evaluated %zu trajectories",
    metrics_list_.size());
  RCLCPP_INFO(logger_, "Overall: Mean ADE=%.3fm (±%.3fm), Mean FDE=%.3fm (±%.3fm)",
    summary_.mean_ade, summary_.std_ade,
    summary_.mean_fde, summary_.std_fde);
}

OpenLoopTrajectoryMetrics OpenLoopEvaluator::evaluate_trajectory(
  const std::shared_ptr<SynchronizedData> & trajectory_data,
  const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data)
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
  metrics.displacement_errors.resize(metrics.num_points, 0.0);
  metrics.ground_truth_available.resize(metrics.num_points, false);
  metrics.ground_truth_poses.resize(metrics.num_points);
  
  // Evaluate each trajectory point
  for (size_t i = 0; i < metrics.num_points; ++i) {
    const auto & traj_point = trajectory.points[i];
    const auto point_time = trajectory_data->timestamp + 
      rclcpp::Duration(traj_point.time_from_start);
    
    // Interpolate ground truth at this time
    auto gt_pose_opt = interpolate_ground_truth(point_time, ground_truth_data);
    
    if (!gt_pose_opt) {
      continue;
    }
    
    const auto & gt_pose = gt_pose_opt.value();
    metrics.ground_truth_available[i] = true;
    metrics.ground_truth_poses[i] = gt_pose;  // Store the ground truth pose
    
    // Calculate displacement error
    metrics.displacement_errors[i] = calculate_distance_2d(
      traj_point.pose.position, gt_pose.position);
    
    // Find two nearest ground truth points for lateral deviation
    size_t nearest_idx = 0;
    double min_time_diff = std::numeric_limits<double>::max();
    
    for (size_t j = 0; j < ground_truth_data.size(); ++j) {
      const double time_diff = 
        std::abs((ground_truth_data[j]->timestamp - point_time).seconds());
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        nearest_idx = j;
      }
    }
    
    // Calculate lateral deviation using adjacent ground truth points
    if (nearest_idx > 0 && nearest_idx < ground_truth_data.size() - 1) {
      const auto & prev_gt = ground_truth_data[nearest_idx - 1]->kinematic_state->pose.pose;
      const auto & next_gt = ground_truth_data[nearest_idx + 1]->kinematic_state->pose.pose;
      
      metrics.lateral_deviations[i] = calculate_lateral_deviation(
        traj_point.pose.position, prev_gt, next_gt);
    } else if (nearest_idx == 0 && ground_truth_data.size() > 1) {
      const auto & curr_gt = ground_truth_data[0]->kinematic_state->pose.pose;
      const auto & next_gt = ground_truth_data[1]->kinematic_state->pose.pose;
      
      metrics.lateral_deviations[i] = calculate_lateral_deviation(
        traj_point.pose.position, curr_gt, next_gt);
    } else if (nearest_idx == ground_truth_data.size() - 1 && ground_truth_data.size() > 1) {
      const auto & prev_gt = ground_truth_data[nearest_idx - 1]->kinematic_state->pose.pose;
      const auto & curr_gt = ground_truth_data[nearest_idx]->kinematic_state->pose.pose;
      
      metrics.lateral_deviations[i] = calculate_lateral_deviation(
        traj_point.pose.position, prev_gt, curr_gt);
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
  if (!ground_truth_data.empty()) {
    metrics.evaluation_end_time = ground_truth_data.back()->timestamp;
  }
  
  return metrics;
}

double OpenLoopEvaluator::calculate_lateral_deviation(
  const geometry_msgs::msg::Point & trajectory_point,
  const geometry_msgs::msg::Pose & segment_start,
  const geometry_msgs::msg::Pose & segment_end)
{
  // Calculate vector from segment start to end
  const double dx = segment_end.position.x - segment_start.position.x;
  const double dy = segment_end.position.y - segment_start.position.y;
  const double segment_length = std::sqrt(dx * dx + dy * dy);
  
  if (segment_length < 1e-6) {
    return calculate_distance_2d(trajectory_point, segment_start.position);
  }
  
  // Normalize segment vector
  const double seg_unit_x = dx / segment_length;
  const double seg_unit_y = dy / segment_length;
  
  // Vector from segment start to trajectory point
  const double to_point_x = trajectory_point.x - segment_start.position.x;
  const double to_point_y = trajectory_point.y - segment_start.position.y;
  
  // Project point onto segment
  const double projection_length = to_point_x * seg_unit_x + to_point_y * seg_unit_y;
  const double clamped_projection = std::max(0.0, std::min(segment_length, projection_length));
  
  // Calculate closest point on segment
  const double closest_x = segment_start.position.x + clamped_projection * seg_unit_x;
  const double closest_y = segment_start.position.y + clamped_projection * seg_unit_y;
  
  // Calculate lateral deviation
  const double lateral_distance = std::sqrt(
    (trajectory_point.x - closest_x) * (trajectory_point.x - closest_x) +
    (trajectory_point.y - closest_y) * (trajectory_point.y - closest_y));
  
  // Determine sign using cross product
  const double cross = seg_unit_x * to_point_y - seg_unit_y * to_point_x;
  
  return (cross >= 0) ? lateral_distance : -lateral_distance;
}

double OpenLoopEvaluator::calculate_distance_2d(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
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
  // Create diagnostic message with evaluation results
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = trajectory_data->timestamp;
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "open_loop_evaluation";
  status.message = "Open-loop trajectory evaluation metrics";
  
  // Add key-value pairs
  diagnostic_msgs::msg::KeyValue kv;
  
  kv.key = "ade";
  kv.value = std::to_string(metrics.ade);
  status.values.push_back(kv);
  
  kv.key = "fde";
  kv.value = std::to_string(metrics.fde);
  status.values.push_back(kv);
  
  kv.key = "mean_lateral_deviation";
  kv.value = std::to_string(metrics.mean_lateral_deviation);
  status.values.push_back(kv);
  
  kv.key = "max_lateral_deviation";
  kv.value = std::to_string(metrics.max_lateral_deviation);
  status.values.push_back(kv);
  
  kv.key = "num_valid_comparisons";
  kv.value = std::to_string(metrics.num_valid_comparisons);
  status.values.push_back(kv);
  
  kv.key = "coverage_ratio";
  kv.value = std::to_string(static_cast<double>(metrics.num_valid_comparisons) / metrics.num_points);
  status.values.push_back(kv);
  
  diag_array.status.push_back(status);
  
  // Write to bag
  bag_writer.write(
    diag_array, "/evaluation/open_loop_metrics",
    trajectory_data->timestamp);
  
  // Save the original trajectory
  if (trajectory_data->trajectory) {
    bag_writer.write(
      *(trajectory_data->trajectory), "/evaluation/original_trajectory",
      trajectory_data->timestamp);
    
    // Create and save ground truth trajectory
    autoware_planning_msgs::msg::Trajectory gt_trajectory;
    gt_trajectory.header.stamp = trajectory_data->timestamp;
    gt_trajectory.header.frame_id = trajectory_data->trajectory->header.frame_id;
    
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
        gt_trajectory, "/evaluation/ground_truth_trajectory",
        trajectory_data->timestamp);
    }
    
  }
}

visualization_msgs::msg::MarkerArray OpenLoopEvaluator::create_evaluation_markers(
  const OpenLoopTrajectoryMetrics & metrics,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<geometry_msgs::msg::Pose> & ground_truth_poses)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Create markers for displacement errors at each point
  for (size_t i = 0; i < metrics.num_points; ++i) {
    if (!metrics.ground_truth_available[i]) {
      continue;
    }
    
    visualization_msgs::msg::Marker marker;
    marker.header = trajectory.header;
    marker.ns = "displacement_error";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = trajectory.points[i].pose;
    
    // Scale based on error magnitude
    const double scale = 0.1 + 0.5 * metrics.displacement_errors[i];
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    
    // Color based on error (green to red)
    const double normalized_error = std::min(1.0, metrics.displacement_errors[i] / 5.0);
    marker.color.r = normalized_error;
    marker.color.g = 1.0 - normalized_error;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    marker_array.markers.push_back(marker);
  }
  
  // Create text markers showing ADE and FDE
  visualization_msgs::msg::Marker text_marker;
  text_marker.header = trajectory.header;
  text_marker.ns = "evaluation_text";
  text_marker.id = 0;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  
  if (!trajectory.points.empty()) {
    text_marker.pose = trajectory.points.front().pose;
    text_marker.pose.position.z += 2.0;
  }
  
  text_marker.scale.z = 0.5;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  
  text_marker.text = "ADE: " + std::to_string(metrics.ade) + "m\n" +
                     "FDE: " + std::to_string(metrics.fde) + "m";
  
  marker_array.markers.push_back(text_marker);
  
  // Create ground truth trajectory visualization
  if (!ground_truth_poses.empty()) {
    visualization_msgs::msg::Marker gt_line_marker;
    gt_line_marker.header = trajectory.header;
    gt_line_marker.ns = "ground_truth_trajectory";
    gt_line_marker.id = 0;
    gt_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    gt_line_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Add points from ground truth
    for (size_t i = 0; i < metrics.num_points; ++i) {
      if (metrics.ground_truth_available[i]) {
        geometry_msgs::msg::Point pt;
        pt.x = ground_truth_poses[i].position.x;
        pt.y = ground_truth_poses[i].position.y;
        pt.z = ground_truth_poses[i].position.z;
        gt_line_marker.points.push_back(pt);
      }
    }
    
    gt_line_marker.scale.x = 0.2;  // Line width
    gt_line_marker.color.r = 0.0;
    gt_line_marker.color.g = 1.0;
    gt_line_marker.color.b = 0.0;
    gt_line_marker.color.a = 0.8;
    gt_line_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    marker_array.markers.push_back(gt_line_marker);
    
    // Create original trajectory visualization
    visualization_msgs::msg::Marker traj_line_marker;
    traj_line_marker.header = trajectory.header;
    traj_line_marker.ns = "original_trajectory";
    traj_line_marker.id = 0;
    traj_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_line_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Add points from original trajectory
    for (const auto & point : trajectory.points) {
      geometry_msgs::msg::Point pt;
      pt.x = point.pose.position.x;
      pt.y = point.pose.position.y;
      pt.z = point.pose.position.z;
      traj_line_marker.points.push_back(pt);
    }
    
    traj_line_marker.scale.x = 0.2;  // Line width
    traj_line_marker.color.r = 0.0;
    traj_line_marker.color.g = 0.0;
    traj_line_marker.color.b = 1.0;
    traj_line_marker.color.a = 0.8;
    traj_line_marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    marker_array.markers.push_back(traj_line_marker);
  }
  
  return marker_array;
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
    summary_.mean_ade = std::accumulate(
      ade_values.begin(), ade_values.end(), 0.0) / ade_values.size();
    summary_.max_ade = *std::max_element(ade_values.begin(), ade_values.end());
    
    double ade_variance = 0.0;
    for (const auto & val : ade_values) {
      ade_variance += (val - summary_.mean_ade) * (val - summary_.mean_ade);
    }
    summary_.std_ade = std::sqrt(ade_variance / ade_values.size());
    
    // FDE statistics
    summary_.mean_fde = std::accumulate(
      fde_values.begin(), fde_values.end(), 0.0) / fde_values.size();
    summary_.max_fde = *std::max_element(fde_values.begin(), fde_values.end());
    
    double fde_variance = 0.0;
    for (const auto & val : fde_values) {
      fde_variance += (val - summary_.mean_fde) * (val - summary_.mean_fde);
    }
    summary_.std_fde = std::sqrt(fde_variance / fde_values.size());
    
    // Lateral deviation statistics
    summary_.mean_lateral_deviation = std::accumulate(
      lateral_dev_values.begin(), lateral_dev_values.end(), 0.0) / lateral_dev_values.size();
    
    double lateral_variance = 0.0;
    for (const auto & val : lateral_dev_values) {
      lateral_variance += (val - summary_.mean_lateral_deviation) * 
                         (val - summary_.mean_lateral_deviation);
    }
    summary_.std_lateral_deviation = std::sqrt(lateral_variance / lateral_dev_values.size());
    
    summary_.max_lateral_deviation = *std::max_element(
      lateral_dev_values.begin(), lateral_dev_values.end());
    
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
    
    // Include point-wise data if needed
    traj["lateral_deviations"] = metrics.lateral_deviations;
    traj["displacement_errors"] = metrics.displacement_errors;
    
    trajectories.push_back(traj);
  }
  
  j["trajectories"] = trajectories;
  
  return j;
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools