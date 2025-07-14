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

#ifndef OPEN_LOOP_EVALUATOR_HPP_
#define OPEN_LOOP_EVALUATOR_HPP_

#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "bag_handler.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

struct OpenLoopTrajectoryMetrics
{
  // Point-wise metrics
  std::vector<double> lateral_deviations;      // Lateral deviation at each trajectory point
  std::vector<double> displacement_errors;     // Euclidean distance at each trajectory point
  std::vector<bool> ground_truth_available;    // Whether ground truth was available at each point
  
  // Ground truth trajectory for this evaluation
  std::vector<geometry_msgs::msg::Pose> ground_truth_poses;  // Interpolated ground truth poses
  
  // Aggregate metrics
  double ade;                         // Average Displacement Error
  double fde;                         // Final Displacement Error
  double mean_lateral_deviation;      // Mean absolute lateral deviation
  double max_lateral_deviation;       // Maximum absolute lateral deviation
  double std_lateral_deviation;       // Standard deviation of lateral deviation
  
  // Trajectory info
  size_t num_points;                  // Number of trajectory points
  size_t num_valid_comparisons;       // Number of points with valid ground truth
  double trajectory_duration;         // Total trajectory duration in seconds
  
  rclcpp::Time trajectory_timestamp;  // When the trajectory was published
  rclcpp::Time evaluation_start_time; // Start time of evaluation window
  rclcpp::Time evaluation_end_time;   // End time of evaluation window
};

struct OpenLoopEvaluationSummary
{
  // Overall statistics
  double mean_ade;
  double std_ade;
  double max_ade;
  double mean_fde;
  double std_fde;
  double max_fde;
  
  double mean_lateral_deviation;
  double std_lateral_deviation;
  double max_lateral_deviation;
  
  // Evaluation coverage
  size_t total_trajectories;
  size_t valid_trajectories;      // Trajectories with at least one valid comparison
  size_t fully_valid_trajectories; // Trajectories with all points having ground truth
  double mean_coverage_ratio;      // Average ratio of valid comparisons per trajectory
  
  double total_evaluation_duration;
};

class OpenLoopEvaluator
{
public:
  explicit OpenLoopEvaluator(
    rclcpp::Logger logger,
    std::shared_ptr<autoware::route_handler::RouteHandler> route_handler = nullptr);

  /**
   * @brief Evaluate trajectories against ground truth localization data
   * @param synchronized_data_list List of synchronized bag data containing trajectories and localization
   * @param bag_writer Optional writer to save evaluation results to a new bag
   */
  void evaluate(
    const std::vector<std::shared_ptr<SynchronizedData>> & synchronized_data_list,
    rosbag2_cpp::Writer * bag_writer = nullptr);

  OpenLoopEvaluationSummary get_summary() const { return summary_; }

  std::vector<OpenLoopTrajectoryMetrics> get_metrics() const { return metrics_list_; }

  nlohmann::json get_summary_as_json() const;
  
  nlohmann::json get_detailed_results_as_json() const;

private:
  /**
   * @brief Evaluate a single trajectory against ground truth
   * @param trajectory_data Data containing the trajectory to evaluate
   * @param ground_truth_data List of future ground truth data points
   * @return Metrics for this trajectory
   */
  OpenLoopTrajectoryMetrics evaluate_trajectory(
    const std::shared_ptr<SynchronizedData> & trajectory_data,
    const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data);

  /**
   * @brief Calculate lateral deviation from trajectory point to ground truth segment
   * @param trajectory_point Trajectory point position
   * @param segment_start Start of ground truth segment
   * @param segment_end End of ground truth segment
   * @return Signed lateral deviation (positive = left, negative = right)
   */
  double calculate_lateral_deviation(
    const geometry_msgs::msg::Point & trajectory_point,
    const geometry_msgs::msg::Pose & segment_start,
    const geometry_msgs::msg::Pose & segment_end);

  /**
   * @brief Calculate 2D Euclidean distance between two points
   */
  double calculate_distance_2d(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);

  /**
   * @brief Find ground truth data at a specific time using interpolation
   * @param target_time Time to interpolate ground truth
   * @param ground_truth_data List of ground truth data points
   * @return Interpolated pose or nullptr if interpolation not possible
   */
  std::optional<geometry_msgs::msg::Pose> interpolate_ground_truth(
    const rclcpp::Time & target_time,
    const std::vector<std::shared_ptr<SynchronizedData>> & ground_truth_data);

  /**
   * @brief Save evaluation metrics to bag
   */
  void save_metrics_to_bag(
    const OpenLoopTrajectoryMetrics & metrics,
    const std::shared_ptr<SynchronizedData> & trajectory_data,
    rosbag2_cpp::Writer & bag_writer);

  /**
   * @brief Create visualization markers for evaluation results
   */
  visualization_msgs::msg::MarkerArray create_evaluation_markers(
    const OpenLoopTrajectoryMetrics & metrics,
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const std::vector<geometry_msgs::msg::Pose> & ground_truth_poses);

  /**
   * @brief Calculate summary statistics from all evaluations
   */
  void calculate_summary();

  rclcpp::Logger logger_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  std::vector<OpenLoopTrajectoryMetrics> metrics_list_;
  OpenLoopEvaluationSummary summary_;
};

}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // OPEN_LOOP_EVALUATOR_HPP_