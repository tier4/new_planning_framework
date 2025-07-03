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

#ifndef CLOSED_LOOP_EVALUATOR_HPP_
#define CLOSED_LOOP_EVALUATOR_HPP_

#include "bag_handler.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nlohmann/json.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <vector>
#include <map>
#include <string>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

struct TrajectoryMetrics
{
  double lateral_error;           // Lateral error from preferred lane centerline
  double longitudinal_error;      // Longitudinal error from planned trajectory
  double lateral_acceleration;    // Lateral acceleration
  double longitudinal_acceleration; // Longitudinal acceleration
  double jerk;                   // Jerk (rate of change of acceleration)
  double curvature;              // Path curvature
  double time_gap;               // Time gap to closest obstacle
  double ttc;                    // Time to collision
  
  // Oscillation metrics
  double steering_angle;         // Current steering angle
  double steering_angular_velocity; // Steering angle change rate
  double lateral_jerk;           // Lateral acceleration change rate
  double yaw_rate;               // Yaw angular velocity
  
  rclcpp::Time timestamp;
};

struct EvaluationSummary
{
  double mean_lateral_error;
  double max_lateral_error;
  double std_lateral_error;      // Standard deviation of lateral error
  double mean_acceleration;
  double max_acceleration;
  double mean_jerk;
  double max_jerk;
  double min_ttc;
  
  // Oscillation statistics
  double mean_steering_angular_velocity;
  double max_steering_angular_velocity;
  double std_steering_angle;     // Standard deviation of steering angle
  size_t steering_reversals;     // Number of steering direction changes
  double mean_lateral_jerk;
  double max_lateral_jerk;
  
  double total_distance;
  double total_time;
  size_t num_samples;
};

class ClosedLoopEvaluator
{
public:
  explicit ClosedLoopEvaluator(rclcpp::Logger logger, std::shared_ptr<autoware::route_handler::RouteHandler> route_handler = nullptr);

  void evaluate(
    const std::vector<std::shared_ptr<SynchronizedData>>& synchronized_data_list,
    rosbag2_cpp::Writer* bag_writer = nullptr);

  EvaluationSummary get_summary() const { return summary_; }
  
  std::vector<TrajectoryMetrics> get_metrics() const { return metrics_list_; }
  
  nlohmann::json get_summary_as_json() const;

private:
  TrajectoryMetrics calculate_metrics(
    const std::shared_ptr<SynchronizedData>& current_data,
    const std::shared_ptr<SynchronizedData>& previous_data = nullptr);

  double calculate_lateral_error(
    const geometry_msgs::msg::Pose& current_pose,
    const autoware_planning_msgs::msg::Trajectory& trajectory);
  
  double calculate_lateral_error_from_preferred_lane(
    const geometry_msgs::msg::Pose& current_pose);

  double calculate_longitudinal_error(
    const geometry_msgs::msg::Pose& current_pose,
    const autoware_planning_msgs::msg::Trajectory& trajectory);
    
  void calculate_oscillation_metrics(
    TrajectoryMetrics& metrics,
    const std::shared_ptr<SynchronizedData>& current_data,
    const std::shared_ptr<SynchronizedData>& previous_data);

  double calculate_ttc(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Twist& current_twist,
    const autoware_perception_msgs::msg::PredictedObjects& objects);

  size_t find_closest_trajectory_point(
    const geometry_msgs::msg::Pose& current_pose,
    const autoware_planning_msgs::msg::Trajectory& trajectory);

  void save_metrics_to_bag(
    const TrajectoryMetrics& metrics,
    const std::shared_ptr<SynchronizedData>& sync_data,
    rosbag2_cpp::Writer& bag_writer);

  void calculate_summary();
  
  void create_lanelet_map_markers(
    visualization_msgs::msg::MarkerArray& marker_array) const;
  
  void create_route_markers(
    visualization_msgs::msg::MarkerArray& marker_array) const;

  rclcpp::Logger logger_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  std::vector<TrajectoryMetrics> metrics_list_;
  EvaluationSummary summary_;
};

}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // CLOSED_LOOP_EVALUATOR_HPP_