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

#include "autoware/trajectory_metrics/metrics.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <rcl/subscription.h>

#include <cstddef>
#include <vector>

namespace autoware::trajectory_selector::trajectory_metrics
{

void LateralAcceleration::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  std::vector<double> metric;
  constexpr double epsilon = 1.0e-3;
  const double time_resolution = resolution() > epsilon ? resolution() : epsilon;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    const auto lateral_acc = (result->points()->at(i + 1).lateral_velocity_mps -
                              result->points()->at(i).lateral_velocity_mps) /
                             time_resolution;
    metric.push_back(std::min(max_value, std::abs(lateral_acc)));
  }
  metric.push_back(metric.back());

  result->set_metric(index(), metric);
}

void LongitudinalJerk::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  if (result->points()->size() < 2) return;

  std::vector<double> metric;
  std::vector<double> acceleration;
  constexpr double epsilon = 1.0e-3;
  const double time_resolution = resolution() > epsilon ? resolution() : epsilon;

  acceleration.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    acceleration.push_back(
      (result->points()->at(i + 1).longitudinal_velocity_mps -
       result->points()->at(i).longitudinal_velocity_mps) /
      time_resolution);
  }
  acceleration.push_back(acceleration.back());

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < acceleration.size() - 1; i++) {
    const auto jerk = (acceleration.at(i + 1) - acceleration.at(i)) / time_resolution;
    metric.push_back(std::min(max_value, std::abs(jerk)));
  }
  metric.push_back(metric.back());

  result->set_metric(index(), metric);
}

void TimeToCollision::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size(); i++) {
    metric.push_back(
      std::min(max_value, utils::time_to_collision(result->points(), result->objects(), i)));
  }

  result->set_metric(index(), metric);
}

void TravelDistance::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size(); i++) {
    metric.push_back(
      std::min(max_value, autoware::motion_utils::calcSignedArcLength(*result->points(), 0L, i)));
  }

  result->set_metric(index(), metric);
}

void LateralDeviation::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size(); i++) {
    const auto arc_coordinates = lanelet::utils::getArcCoordinates(
      *result->preferred_lanes(), autoware_utils::get_pose(result->points()->at(i)));
    metric.push_back(std::min(max_value, std::abs(arc_coordinates.distance)));
  }

  result->set_metric(index(), metric);
}

void TrajectoryDeviation::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  if (result->previous() == nullptr) return;

  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size(); i++) {
    const auto & p1 = result->points()->at(i).pose;
    const auto & p2 = result->previous()->at(i).pose;
    metric.push_back(std::min(max_value, autoware_utils::calc_squared_distance2d(p1, p2)));
  }

  result->set_metric(index(), metric);
}

void SteeringConsistency::evaluate(
  const std::shared_ptr<DataInterface> & result, const double max_value) const
{
  std::vector<double> metric;
  metric.reserve(result->points()->size());

  if (result->previous() == nullptr) return;

  const auto wheel_base = vehicle_info()->wheel_base_m;

  metric.reserve(result->points()->size());
  for (const auto & point : *result->points()) {
    const auto current = utils::steer_command(result->points(), point.pose, wheel_base);
    const auto previous = utils::steer_command(result->previous(), point.pose, wheel_base);

    metric.push_back(std::min(max_value, std::abs(current - previous)));
  }

  result->set_metric(index(), metric);
}

}  // namespace autoware::trajectory_selector::trajectory_metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::LateralAcceleration,
  autoware::trajectory_selector::MetricInterface)

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::LongitudinalJerk,
  autoware::trajectory_selector::MetricInterface)

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::TimeToCollision,
  autoware::trajectory_selector::MetricInterface)

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::TravelDistance,
  autoware::trajectory_selector::MetricInterface)

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::LateralDeviation,
  autoware::trajectory_selector::MetricInterface)

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::TrajectoryDeviation,
  autoware::trajectory_selector::MetricInterface)

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_selector::trajectory_metrics::SteeringConsistency,
  autoware::trajectory_selector::MetricInterface)
