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
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

namespace autoware::trajectory_selector::trajectory_metrics
{

void LateralAcceleration::evaluate(const std::shared_ptr<DataInterface> & result) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size(); i++) {
    const auto radius =
      vehicle_info_->wheel_base_m / std::tan(result->points()->at(i).front_wheel_angle_rad);
    const auto speed = result->points()->at(i).longitudinal_velocity_mps;
    metric.push_back(std::abs(speed * speed / radius));
  }

  result->set_metric(index(), metric);
}

void LongitudinalJerk::evaluate(const std::shared_ptr<DataInterface> & result) const
{
  if (result->points()->size() < 2) return;

  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    const auto jerk =
      (result->points()->at(i + 1).acceleration_mps2 - result->points()->at(i).acceleration_mps2) /
      0.5;
    metric.push_back(std::abs(jerk));
  }
  metric.push_back(metric.back());

  result->set_metric(index(), metric);
}

void TimeToCollision::evaluate(const std::shared_ptr<DataInterface> & result) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    metric.push_back(utils::time_to_collision(result->points(), result->objects(), i));
  }

  result->set_metric(index(), metric);
}

void TravelDistance::evaluate(const std::shared_ptr<DataInterface> & result) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    metric.push_back(autoware::motion_utils::calcSignedArcLength(*result->points(), 0L, i));
  }

  result->set_metric(index(), metric);
}

void LateralDeviation::evaluate(const std::shared_ptr<DataInterface> & result) const
{
  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    const auto arc_coordinates = lanelet::utils::getArcCoordinates(
      *result->preferred_lanes(), autoware::universe_utils::getPose(result->points()->at(i)));
    metric.push_back(std::abs(arc_coordinates.distance));
  }

  result->set_metric(index(), metric);
}

void TrajectoryDeviation::evaluate(const std::shared_ptr<DataInterface> & result) const
{
  if (result->previous() == nullptr) return;

  std::vector<double> metric;

  metric.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    const auto & p1 = result->points()->at(i).pose;
    const auto & p2 = result->previous()->at(i).pose;
    metric.push_back(autoware::universe_utils::calcSquaredDistance2d(p1, p2));
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
