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

#include <autoware/trajectory_selector_common/structs.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>

#include <autoware_vehicle_msgs/msg/detail/steering_report__struct.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <cstddef>
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
  const std::shared_ptr<TrajectoryPoints> & previous_points, const bool create_augmented_data)
{
  tf_ = std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
          ->get(bag_data->timestamp);

  steering_ =
    std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers.at(TOPIC::STEERING))
      ->get(bag_data->timestamp);

  objects_ = objects(bag_data, parameters_);

  trajectory_ =
    std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
      ->get(bag_data->timestamp);

  preferred_lanes_ = preferred_lanes(bag_data, route_handler(), parameters_->max_trajectory_length);

  if (!trajectory_) return;
  if (odometry_->twist.twist.linear.x < 1e-3) return;
  if (std::all_of(trajectory_->points.begin(), trajectory_->points.end(), [](const auto & points) {
        return points.longitudinal_velocity_mps < 1e-3;
      }))
    return;

  // Remove zero velocity at end point if necessary
  const auto trajectory_size = trajectory_->points.size();
  if (trajectory_size > 1 && trajectory_->points.back().longitudinal_velocity_mps < 1e-03) {
    const auto velocity_diff =
      trajectory_->points.back().longitudinal_velocity_mps -
      trajectory_->points.at(trajectory_size - 2).longitudinal_velocity_mps;
    if (velocity_diff > 2.0) {
      trajectory_->points.back().longitudinal_velocity_mps =
        trajectory_->points.at(trajectory_size - 2).longitudinal_velocity_mps;
    }
  }

  // add candidate path
  {
    const auto points = autoware::trajectory_selector::utils::sampling(
      trajectory_->points, odometry_->pose.pose, parameters_->sample_num, parameters_->resolution);
    const auto core_data = std::make_shared<CoreData>(
      std::make_shared<TrajectoryPoints>(points), previous_points, objects_, odometry_, steering_,
      preferred_lanes_, "candidate");
    add(core_data);
  }

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
}

// Evaluator::setup(previous_points);
}

auto BagEvaluator::preferred_lanes(
  const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<RouteHandler> & route_handler,
  const double max_trajectory_length) const -> std::shared_ptr<lanelet::ConstLanelets>
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
  std::for_each(
    itr, lanes.end(), [&length, &preferred_lanes, &max_trajectory_length](const auto & lanelet) {
      length +=
        static_cast<double>(boost::geometry::length(lanelet.centerline().basicLineString()));
      if (length < max_trajectory_length) preferred_lanes->push_back(lanelet);
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
  if (!current_objects) {
    return nullptr;
  }

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
    auto odometry_ptr =
      odometry_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!odometry_ptr) {
      odometry_ptr = std::make_shared<Odometry>(odometry_buffer_ptr->msgs.back());
    }

    auto accel_ptr =
      acceleration_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!accel_ptr) {
      accel_ptr =
        std::make_shared<AccelWithCovarianceStamped>(acceleration_buffer_ptr->msgs.back());
    }

    auto opt_steer =
      steering_buffer_ptr->get(bag_data->timestamp + 1e9 * parameters->resolution * i);
    if (!opt_steer) {
      opt_steer = std::make_shared<SteeringReport>(steering_buffer_ptr->msgs.back());
    }

    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(odometry_ptr->pose.pose)
                         .longitudinal_velocity_mps(odometry_ptr->twist.twist.linear.x)
                         .lateral_velocity_mps(odometry_ptr->twist.twist.linear.y)
                         .acceleration_mps2(accel_ptr->accel.accel.linear.x)
                         .heading_rate_rps(odometry_ptr->twist.twist.angular.z)
                         .front_wheel_angle_rad(opt_steer->steering_tire_angle)
                         .rear_wheel_angle_rad(0.0);
    points.push_back(point);
  }

  return std::make_shared<TrajectoryPoints>(points);
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

std::vector<TrajectoryWithMetrics> BagEvaluator::calc_metric_values(
  const size_t metrics_size, std::shared_ptr<TrajectoryPoints> & previous_points)
{
  std::vector<TrajectoryWithMetrics> results;
  std::vector<double> max_values(metrics_size, std::numeric_limits<double>::max());
  evaluate(max_values);
  const auto ground_truth = get("ground_truth");
  const auto candidate = get("candidate");
  TrajectoryWithMetrics ground_truth_with_metrics;
  ground_truth_with_metrics.tag = "ground_truth";
  ground_truth_with_metrics.points.reserve(20);
  TrajectoryWithMetrics candidate_with_metrics;
  candidate_with_metrics.tag = "candidate";
  candidate_with_metrics.points.reserve(20);

  std::vector<std::vector<double>> metric_values;
  for (size_t i = 0; i < metrics_size; i++) {
    metric_values.push_back(ground_truth->get_metric(i));
  }

  for (size_t idx = 0; idx < ground_truth->points()->size(); idx++) {
    TrajectoryPointWithMetrics point_with_metrics;
    point_with_metrics.point.pose = ground_truth->points()->at(idx).pose;
    point_with_metrics.point.longitudinal_velocity_mps =
      ground_truth->points()->at(idx).longitudinal_velocity_mps;
    point_with_metrics.point.lateral_velocity_mps =
      ground_truth->points()->at(idx).lateral_velocity_mps;
    point_with_metrics.point.acceleration_mps2 = ground_truth->points()->at(idx).acceleration_mps2;
    point_with_metrics.point.heading_rate_rps = ground_truth->points()->at(idx).heading_rate_rps;
    point_with_metrics.point.front_wheel_angle_rad =
      ground_truth->points()->at(idx).front_wheel_angle_rad;
    point_with_metrics.point.rear_wheel_angle_rad =
      ground_truth->points()->at(idx).rear_wheel_angle_rad;
    point_with_metrics.metrics.reserve(metrics_size);
    for (size_t i = 0; i < metrics_size; i++) {
      point_with_metrics.metrics.push_back(metric_values.at(i).at(idx));
    }
    ground_truth_with_metrics.points.push_back(point_with_metrics);
  }
  results.push_back(ground_truth_with_metrics);

  metric_values.clear();
  for (size_t i = 0; i < metrics_size; i++) {
    metric_values.push_back(candidate->get_metric(i));
  }

  for (size_t idx = 0; idx < candidate->points()->size(); idx++) {
    TrajectoryPointWithMetrics point_with_metrics;
    point_with_metrics.point.pose = candidate->points()->at(idx).pose;
    point_with_metrics.point.longitudinal_velocity_mps =
      candidate->points()->at(idx).longitudinal_velocity_mps;
    point_with_metrics.point.lateral_velocity_mps =
      candidate->points()->at(idx).lateral_velocity_mps;
    point_with_metrics.point.acceleration_mps2 = candidate->points()->at(idx).acceleration_mps2;
    point_with_metrics.point.heading_rate_rps = candidate->points()->at(idx).heading_rate_rps;
    point_with_metrics.point.front_wheel_angle_rad =
      candidate->points()->at(idx).front_wheel_angle_rad;
    point_with_metrics.point.rear_wheel_angle_rad =
      candidate->points()->at(idx).rear_wheel_angle_rad;
    point_with_metrics.metrics.reserve(metrics_size);
    for (size_t i = 0; i < metrics_size; i++) {
      point_with_metrics.metrics.push_back(metric_values.at(i).at(idx));
    }
    candidate_with_metrics.points.push_back(point_with_metrics);
  }
  results.push_back(candidate_with_metrics);

  previous_points = ground_truth != nullptr ? ground_truth->points() : nullptr;

  return results;
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
