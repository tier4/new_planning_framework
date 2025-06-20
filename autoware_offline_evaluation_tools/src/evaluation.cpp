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

#include "evaluation.hpp"

#include "autoware/offline_evaluation_tools/utils.hpp"
#include "autoware/trajectory_selector_common/utils.hpp"
#include "bag_handler.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
BagEvaluator::BagEvaluator(
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::shared_ptr<VehicleInfo> & vehicle_info,
  const std::shared_ptr<DataAugmentParameters> & parameters)
: Evaluator{route_handler, vehicle_info}, parameters_{parameters}
{
}

void BagEvaluator::setup(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  tf_ = std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
          ->get(bag_data->timestamp);

  steering_ =
    std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers.at(TOPIC::STEERING))
      ->get(bag_data->timestamp);

  objects_ = objects(bag_data, parameters_);

  preferred_lanes_ = preferred_lanes(bag_data, route_handler());

  // add actual driving data
  {
    const auto core_data = std::make_shared<CoreData>(
      ground_truth(bag_data, parameters_), previous_points, objects_, preferred_lanes_,
      "ground_truth");

    add(core_data);
  }

  // data augmentation
  for (const auto & points : augment_data(bag_data, vehicle_info(), parameters_)) {
    const auto core_data =
      std::make_shared<CoreData>(points, previous_points, objects_, preferred_lanes_, "candidates");

    add(core_data);
  }

  // Evaluator::setup(previous_points);
}

auto BagEvaluator::preferred_lanes(
  const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<RouteHandler> & route_handler)
  const -> std::shared_ptr<lanelet::ConstLanelets>
{
  const auto lanes = route_handler->getPreferredLanelets();

  const auto odometry =
    std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY))
      ->get(bag_data->timestamp);

  lanelet::ConstLanelet nearest{};
  if (!route_handler->getClosestPreferredLaneletWithinRoute(odometry->pose.pose, &nearest)) {
    RCLCPP_ERROR(rclcpp::get_logger(__func__), "couldn't find nearest preferred lane.");
  }

  const auto itr = std::find_if(lanes.begin(), lanes.end(), [&nearest](const auto & lanelet) {
    return lanelet.id() == nearest.id();
  });

  double length = 0.0;

  const auto preferred_lanes = std::make_shared<lanelet::ConstLanelets>();
  std::for_each(itr, lanes.end(), [&length, &preferred_lanes](const auto & lanelet) {
    length += static_cast<double>(boost::geometry::length(lanelet.centerline().basicLineString()));
    constexpr double threshold = 150.0;
    if (length < threshold) preferred_lanes->push_back(lanelet);
  });

  return preferred_lanes;
}

auto BagEvaluator::objects(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<DataAugmentParameters> & parameters) const
  -> std::shared_ptr<PredictedObjects>
{
  const auto objects_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS));

  const auto current_objects = objects_buffer_ptr->get(bag_data->timestamp);

  // remove predicted_paths.
  std::for_each(
    current_objects->objects.begin(), current_objects->objects.end(), [](auto & object) {
      object.kinematics.predicted_paths.clear();
      object.kinematics.predicted_paths.push_back(PredictedPath{});
    });

  // overwrite predicted_paths by future data.
  for (size_t i = 0; i < parameters->sample_num; i++) {
    const auto objects_ptr =
      objects_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!objects_ptr) {
      break;
    }

    for (const auto & a : objects_ptr->objects) {
      const auto itr = std::find_if(
        current_objects->objects.begin(), current_objects->objects.end(),
        [&a](const auto & b) { return a.object_id == b.object_id; });
      if (itr == current_objects->objects.end()) continue;

      itr->kinematics.predicted_paths.at(0).path.push_back(
        a.kinematics.initial_pose_with_covariance.pose);
    }
  }

  return current_objects;
}

auto BagEvaluator::ground_truth(
  const std::shared_ptr<BagData> & bag_data,
  const std::shared_ptr<DataAugmentParameters> & parameters) const
  -> std::shared_ptr<TrajectoryPoints>
{
  TrajectoryPoints points;

  const auto odometry_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY));

  const auto acceleration_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
      bag_data->buffers.at(TOPIC::ACCELERATION));

  const auto steering_buffer_ptr =
    std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers.at(TOPIC::STEERING));

  for (size_t i = 0; i < parameters->sample_num; i++) {
    const auto odometry_ptr =
      odometry_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!odometry_ptr) {
      throw std::logic_error("data is not enough.");
    }

    const auto accel_ptr =
      acceleration_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!accel_ptr) {
      throw std::logic_error("data is not enough.");
    }

    const auto opt_steer =
      steering_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!opt_steer) {
      throw std::logic_error("data is not enough.");
    }

    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(odometry_ptr->pose.pose)
                         .longitudinal_velocity_mps(odometry_ptr->twist.twist.linear.x)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(accel_ptr->accel.accel.linear.x)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(opt_steer->steering_tire_angle)
                         .rear_wheel_angle_rad(0.0);
    points.push_back(point);
  }

  return std::make_shared<TrajectoryPoints>(points);
}

auto BagEvaluator::ground_truth_from_live_trajectory(
  const std::shared_ptr<ReplayEvaluationData> & replay_data,
  const Trajectory & live_trajectory) const
  -> std::shared_ptr<TrajectoryPoints>
{
  auto ground_truth_points = std::make_shared<TrajectoryPoints>();
  ground_truth_points->reserve(live_trajectory.points.size());

  for (const auto & traj_point : live_trajectory.points) {
    // Calculate target timestamp: current replay time + time_from_start
    const auto time_from_start_ns = static_cast<rcutils_time_point_value_t>(
      traj_point.time_from_start.sec) * 1000000000LL + 
      static_cast<rcutils_time_point_value_t>(traj_point.time_from_start.nanosec);
    
    const auto target_timestamp = replay_data->timestamp + time_from_start_ns;

    // Get localization data at target time
    const auto localization_msg = get_localization_at_time(replay_data, target_timestamp);
    
    if (localization_msg) {
      // Convert localization to trajectory point
      const auto ground_truth_point = convert_localization_to_trajectory_point(
        *localization_msg, traj_point.time_from_start);
      ground_truth_points->push_back(ground_truth_point);
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("BagEvaluator"), 
        "No localization data found for timestamp %lld", target_timestamp);
    }
  }

  return ground_truth_points;
}

auto BagEvaluator::get_localization_at_time(
  const std::shared_ptr<ReplayEvaluationData> & replay_data,
  const rcutils_time_point_value_t target_timestamp) const
  -> std::shared_ptr<Odometry>
{
  const auto odometry_buffer = 
    std::dynamic_pointer_cast<Buffer<Odometry>>(replay_data->buffers.at(TOPIC::ODOMETRY));

  if (!odometry_buffer || odometry_buffer->msgs.empty()) {
    return nullptr;
  }

  // Find the closest message by timestamp
  std::shared_ptr<Odometry> closest_before = nullptr;
  std::shared_ptr<Odometry> closest_after = nullptr;
  rcutils_time_point_value_t min_diff_before = std::numeric_limits<rcutils_time_point_value_t>::max();
  rcutils_time_point_value_t min_diff_after = std::numeric_limits<rcutils_time_point_value_t>::max();

  for (const auto & odom_msg : odometry_buffer->msgs) {
    const auto msg_timestamp = rclcpp::Time(odom_msg.header.stamp).nanoseconds();
    
    if (msg_timestamp <= target_timestamp) {
      const auto diff = target_timestamp - msg_timestamp;
      if (diff < min_diff_before) {
        min_diff_before = diff;
        closest_before = std::make_shared<Odometry>(odom_msg);
      }
    } else {
      const auto diff = msg_timestamp - target_timestamp;
      if (diff < min_diff_after) {
        min_diff_after = diff;
        closest_after = std::make_shared<Odometry>(odom_msg);
      }
    }
  }

  // If we have both before and after, interpolate
  if (closest_before && closest_after) {
    const auto before_time = rclcpp::Time(closest_before->header.stamp).nanoseconds();
    const auto after_time = rclcpp::Time(closest_after->header.stamp).nanoseconds();
    const auto total_diff = after_time - before_time;
    
    if (total_diff > 0) {
      const auto ratio = static_cast<double>(target_timestamp - before_time) / total_diff;
      return interpolate_localization(closest_before, closest_after, ratio);
    }
  }

  // Return the closest available message
  if (closest_before && (!closest_after || min_diff_before <= min_diff_after)) {
    return closest_before;
  } else if (closest_after) {
    return closest_after;
  }

  return nullptr;
}

auto BagEvaluator::interpolate_localization(
  const std::shared_ptr<Odometry> & odom1,
  const std::shared_ptr<Odometry> & odom2,
  const double ratio) const
  -> std::shared_ptr<Odometry>
{
  auto interpolated = std::make_shared<Odometry>();
  
  // Copy header from first message and interpolate timestamp
  interpolated->header = odom1->header;
  const auto time1 = rclcpp::Time(odom1->header.stamp).nanoseconds();
  const auto time2 = rclcpp::Time(odom2->header.stamp).nanoseconds();
  const auto interpolated_time = time1 + static_cast<rcutils_time_point_value_t>((time2 - time1) * ratio);
  interpolated->header.stamp = rclcpp::Time(interpolated_time);

  // Interpolate position
  interpolated->pose.pose.position.x = odom1->pose.pose.position.x + 
    (odom2->pose.pose.position.x - odom1->pose.pose.position.x) * ratio;
  interpolated->pose.pose.position.y = odom1->pose.pose.position.y + 
    (odom2->pose.pose.position.y - odom1->pose.pose.position.y) * ratio;
  interpolated->pose.pose.position.z = odom1->pose.pose.position.z + 
    (odom2->pose.pose.position.z - odom1->pose.pose.position.z) * ratio;

  // For orientation, use SLERP (simplified linear interpolation for small angles)
  interpolated->pose.pose.orientation.x = odom1->pose.pose.orientation.x + 
    (odom2->pose.pose.orientation.x - odom1->pose.pose.orientation.x) * ratio;
  interpolated->pose.pose.orientation.y = odom1->pose.pose.orientation.y + 
    (odom2->pose.pose.orientation.y - odom1->pose.pose.orientation.y) * ratio;
  interpolated->pose.pose.orientation.z = odom1->pose.pose.orientation.z + 
    (odom2->pose.pose.orientation.z - odom1->pose.pose.orientation.z) * ratio;
  interpolated->pose.pose.orientation.w = odom1->pose.pose.orientation.w + 
    (odom2->pose.pose.orientation.w - odom1->pose.pose.orientation.w) * ratio;

  // Interpolate velocity
  interpolated->twist.twist.linear.x = odom1->twist.twist.linear.x + 
    (odom2->twist.twist.linear.x - odom1->twist.twist.linear.x) * ratio;
  interpolated->twist.twist.linear.y = odom1->twist.twist.linear.y + 
    (odom2->twist.twist.linear.y - odom1->twist.twist.linear.y) * ratio;
  interpolated->twist.twist.angular.z = odom1->twist.twist.angular.z + 
    (odom2->twist.twist.angular.z - odom1->twist.twist.angular.z) * ratio;

  // Copy covariance from first message
  interpolated->pose.covariance = odom1->pose.covariance;
  interpolated->twist.covariance = odom1->twist.covariance;

  return interpolated;
}

auto BagEvaluator::convert_localization_to_trajectory_point(
  const Odometry & localization,
  const builtin_interfaces::msg::Duration & time_from_start) const
  -> autoware_planning_msgs::msg::TrajectoryPoint
{
  autoware_planning_msgs::msg::TrajectoryPoint traj_point;
  
  // Set timing
  traj_point.time_from_start = time_from_start;
  
  // Copy pose
  traj_point.pose = localization.pose.pose;
  
  // Copy velocities
  traj_point.longitudinal_velocity_mps = localization.twist.twist.linear.x;
  traj_point.lateral_velocity_mps = localization.twist.twist.linear.y;
  
  // Set other fields to default values
  traj_point.acceleration_mps2 = 0.0;
  traj_point.heading_rate_rps = localization.twist.twist.angular.z;
  traj_point.front_wheel_angle_rad = 0.0;
  traj_point.rear_wheel_angle_rad = 0.0;

  return traj_point;
}

std::pair<double, double> BagEvaluator::calculate_displacement_errors(
  const std::shared_ptr<TrajectoryPoints> & candidate_trajectory,
  const std::shared_ptr<TrajectoryPoints> & ground_truth_trajectory) const
{
  if (!candidate_trajectory || !ground_truth_trajectory || 
      candidate_trajectory->empty() || ground_truth_trajectory->empty()) {
    return std::make_pair(0.0, 0.0);
  }
  
  // Find minimum size for comparison
  const auto min_size = std::min(candidate_trajectory->size(), ground_truth_trajectory->size());
  
  double sum_errors = 0.0;
  
  // Calculate point-wise displacement errors
  for (size_t i = 0; i < min_size; ++i) {
    const auto& candidate_pose = candidate_trajectory->at(i).pose;
    const auto& ground_truth_pose = ground_truth_trajectory->at(i).pose;
    
    const double displacement_error = calculate_euclidean_distance(candidate_pose, ground_truth_pose);
    sum_errors += displacement_error;
  }
  
  // Calculate ADE (Average Displacement Error)
  const double ade = min_size > 0 ? sum_errors / static_cast<double>(min_size) : 0.0;
  
  // Calculate FDE (Final Displacement Error)
  double fde = 0.0;
  if (min_size > 0) {
    const auto& final_candidate_pose = candidate_trajectory->at(min_size - 1).pose;
    const auto& final_ground_truth_pose = ground_truth_trajectory->at(min_size - 1).pose;
    fde = calculate_euclidean_distance(final_candidate_pose, final_ground_truth_pose);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("BagEvaluator"), "ADE: %.3fm, FDE: %.3fm", ade, fde);
  
  return std::make_pair(ade, fde);
}

std::pair<bool, double> BagEvaluator::check_speed_limit_violations(
  const std::shared_ptr<TrajectoryPoints> & trajectory,
  const double speed_limit_mps) const
{
  if (!trajectory || trajectory->empty()) {
    return std::make_pair(false, 0.0);
  }

  bool has_violation = false;
  double max_violation = 0.0;
  
  for (const auto & point : *trajectory) {
    const double speed = std::abs(point.longitudinal_velocity_mps);
    if (speed > speed_limit_mps) {
      has_violation = true;
      const double violation_amount = speed - speed_limit_mps;
      max_violation = std::max(max_violation, violation_amount);
    }
  }
  
  if (has_violation) {
    RCLCPP_WARN(
      rclcpp::get_logger("BagEvaluator"), 
      "Speed limit violation detected: max %.2f m/s over limit (%.2f km/h over)", 
      max_violation, max_violation * 3.6);
  }
  
  return std::make_pair(has_violation, max_violation);
}

std::pair<bool, double> BagEvaluator::check_lane_keeping_violations(
  const std::shared_ptr<TrajectoryPoints> & trajectory,
  const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes) const
{
  if (!trajectory || trajectory->empty() || !preferred_lanes || preferred_lanes->empty()) {
    return std::make_pair(false, 0.0);
  }

  bool has_violation = false;
  double max_deviation = 0.0;
  
  for (const auto & point : *trajectory) {
    // Use existing lanelet2 utility to calculate lateral deviation
    const auto arc_coordinates = lanelet::utils::getArcCoordinates(
      *preferred_lanes, point.pose);
    
    const double lateral_deviation = std::abs(arc_coordinates.distance);
    max_deviation = std::max(max_deviation, lateral_deviation);
    
    // Assume lane width of 3.5m, violation if more than 1.75m from center
    const double lane_half_width = 1.75;
    if (lateral_deviation > lane_half_width) {
      has_violation = true;
    }
  }
  
  if (has_violation) {
    RCLCPP_WARN(
      rclcpp::get_logger("BagEvaluator"), 
      "Lane keeping violation detected: max %.2fm lateral deviation", 
      max_deviation);
  }
  
  return std::make_pair(has_violation, max_deviation);
}

std::tuple<double, double, double> BagEvaluator::calculate_comfort_metrics(
  const std::shared_ptr<TrajectoryPoints> & trajectory) const
{
  if (!trajectory || trajectory->size() < 2) {
    return std::make_tuple(0.0, 0.0, 0.0);
  }
  
  double max_lateral_accel = 0.0;
  double max_longitudinal_accel = 0.0;
  double max_jerk = 0.0;
  
  std::vector<double> longitudinal_accels;
  longitudinal_accels.reserve(trajectory->size());
  
  // Calculate maximum accelerations
  for (const auto & point : *trajectory) {
    // Direct acceleration from trajectory point
    const double lon_accel = std::abs(point.acceleration_mps2);
    max_longitudinal_accel = std::max(max_longitudinal_accel, lon_accel);
    longitudinal_accels.push_back(point.acceleration_mps2);
    
    // Lateral acceleration (simplified using lateral velocity)
    const double lat_accel = std::abs(point.lateral_velocity_mps);
    max_lateral_accel = std::max(max_lateral_accel, lat_accel);
  }
  
  // Calculate maximum jerk (rate of acceleration change)
  constexpr double dt = 0.1; // Assume 100ms time resolution
  for (size_t i = 1; i < longitudinal_accels.size(); ++i) {
    const double jerk = std::abs((longitudinal_accels[i] - longitudinal_accels[i-1]) / dt);
    max_jerk = std::max(max_jerk, jerk);
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("BagEvaluator"), 
    "Comfort metrics - Max lateral accel: %.2f m/s², Max longitudinal accel: %.2f m/s², Max jerk: %.2f m/s³", 
    max_lateral_accel, max_longitudinal_accel, max_jerk);
  
  return std::make_tuple(max_lateral_accel, max_longitudinal_accel, max_jerk);
}

void BagEvaluator::evaluate_trajectory_safety(
  const std::shared_ptr<TrajectoryPoints> & trajectory,
  const std::shared_ptr<PredictedObjects> & objects,
  const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes) const
{
  if (!trajectory || trajectory->empty()) {
    RCLCPP_WARN(rclcpp::get_logger("BagEvaluator"), "Empty trajectory provided for safety evaluation");
    return;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("BagEvaluator"), "=== Trajectory Safety Evaluation ===");
  
  // Calculate TTC
  const double min_ttc = calculate_minimum_ttc(trajectory, objects);
  
  // Check traffic rule compliance
  const auto [speed_violation, max_speed_violation] = check_speed_limit_violations(trajectory);
  const auto [lane_violation, max_lateral_deviation] = check_lane_keeping_violations(trajectory, preferred_lanes);
  
  // Calculate comfort metrics
  const auto [max_lat_accel, max_lon_accel, max_jerk] = calculate_comfort_metrics(trajectory);
  
  // Log comprehensive summary
  RCLCPP_INFO(
    rclcpp::get_logger("BagEvaluator"), 
    "Safety Summary - TTC: %.2fs, Speed violation: %s (%.2f m/s), Lane violation: %s (%.2fm)",
    min_ttc, 
    speed_violation ? "YES" : "NO", max_speed_violation,
    lane_violation ? "YES" : "NO", max_lateral_deviation);
  
  // Evaluate against thresholds
  bool is_safe = true;
  if (min_ttc < 3.0) {
    RCLCPP_WARN(rclcpp::get_logger("BagEvaluator"), "UNSAFE: TTC below 3 seconds!");
    is_safe = false;
  }
  if (speed_violation) {
    RCLCPP_WARN(rclcpp::get_logger("BagEvaluator"), "UNSAFE: Speed limit violation!");
    is_safe = false;
  }
  if (lane_violation) {
    RCLCPP_WARN(rclcpp::get_logger("BagEvaluator"), "UNSAFE: Lane keeping violation!");
    is_safe = false;
  }
  if (max_lat_accel > 3.0 || max_lon_accel > 4.0 || max_jerk > 3.0) {
    RCLCPP_WARN(rclcpp::get_logger("BagEvaluator"), "UNCOMFORTABLE: High accelerations/jerk!");
  }
  
  if (is_safe) {
    RCLCPP_INFO(rclcpp::get_logger("BagEvaluator"), "\u2713 Trajectory is SAFE");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("BagEvaluator"), "\u26a0 Trajectory has SAFETY ISSUES");
  }
}

double BagEvaluator::calculate_euclidean_distance(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2) const
{
  const double dx = pose1.position.x - pose2.position.x;
  const double dy = pose1.position.y - pose2.position.y;
  const double dz = pose1.position.z - pose2.position.z;
  
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double BagEvaluator::calculate_minimum_ttc(
  const std::shared_ptr<TrajectoryPoints> & trajectory,
  const std::shared_ptr<PredictedObjects> & objects) const
{
  if (!trajectory || !objects || trajectory->empty() || objects->objects.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_ttc = std::numeric_limits<double>::infinity();
  
  // Calculate TTC for each trajectory point using existing autoware utilities
  for (size_t i = 0; i < trajectory->size(); ++i) {
    const auto & traj_point = trajectory->at(i);
    
    // Calculate TTC for all objects at this trajectory point
    for (const auto & object : objects->objects) {
      // Use existing autoware TTC implementation
      const double ttc = autoware::trajectory_selector::utils::time_to_collision(
        traj_point, traj_point.time_from_start, object);
      
      if (std::isfinite(ttc) && ttc > 0.0) {
        min_ttc = std::min(min_ttc, ttc);
      }
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("BagEvaluator"), "Minimum TTC: %.2f seconds", min_ttc);
  return min_ttc;
}



auto BagEvaluator::augment_data(
  const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<VehicleInfo> & vehicle_info,
  const std::shared_ptr<DataAugmentParameters> & parameters) const
  -> std::vector<std::shared_ptr<TrajectoryPoints>>
{
  std::vector<std::shared_ptr<TrajectoryPoints>> augment_data;

  const auto odometry_ptr =
    std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY))
      ->get(bag_data->timestamp);
  if (!odometry_ptr) {
    throw std::logic_error("data is not enough.");
  }

  const auto accel_ptr = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
                           bag_data->buffers.at(TOPIC::ACCELERATION))
                           ->get(bag_data->timestamp);
  if (!accel_ptr) {
    throw std::logic_error("data is not enough.");
  }

  const auto trajectory_ptr =
    std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
      ->get(bag_data->timestamp);
  if (!trajectory_ptr) {
    throw std::logic_error("data is not enough.");
  }

  for (const auto & points : utils::augment(
         *trajectory_ptr, odometry_ptr->pose.pose, odometry_ptr->twist.twist.linear.x,
         accel_ptr->accel.accel.linear.x, vehicle_info, parameters)) {
    augment_data.push_back(std::make_shared<TrajectoryPoints>(points));
  }

  return augment_data;
}

auto BagEvaluator::loss(const std::shared_ptr<EvaluatorParameters> & parameters)
  -> std::pair<double, std::shared_ptr<TrajectoryPoints>>
{
  const auto best_data = best(parameters, "ground_truth");

  if (best_data == nullptr) {
    return std::make_pair(0.0, nullptr);
  }

  const auto ground_truth = get("ground_truth");
  const auto min_size = std::min(ground_truth->points()->size(), best_data->points()->size());

  double mse = 0.0;
  for (size_t i = 0; i < min_size; i++) {
    const auto & p1 = autoware_utils::get_pose(ground_truth->points()->at(i));
    const auto & p2 = autoware_utils::get_pose(best_data->points()->at(i));
    mse = (mse * i + autoware_utils::calc_squared_distance2d(p1, p2)) / (i + 1);
  }

  if (!std::isfinite(mse)) {
    throw std::logic_error("loss value is invalid.");
  }

  return std::make_pair(mse, best_data->points());
}

auto BagEvaluator::marker() const -> std::shared_ptr<MarkerArray>
{
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;

  MarkerArray msg;

  const auto ground_truth = get("ground_truth");
  if (ground_truth != nullptr) {
    for (size_t i = 0; i < ground_truth->points()->size(); ++i) {
      Marker marker = create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "ground_truth", i, Marker::ARROW,
        create_marker_scale(0.7, 0.3, 0.3), create_marker_color(1.0, 0.0, 0.0, 0.999));
      marker.pose = ground_truth->points()->at(i).pose;
      msg.markers.push_back(marker);
    }
  }

  autoware_utils::append_marker_array(*Evaluator::marker(), &msg);

  return std::make_shared<MarkerArray>(msg);
}
}  // namespace autoware::trajectory_selector::offline_evaluation_tools
