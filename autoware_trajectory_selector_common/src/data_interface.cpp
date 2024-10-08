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

#include "autoware/trajectory_selector_common/data_interface.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

namespace autoware::trajectory_selector
{

DataInterface::DataInterface(
  const std::shared_ptr<CoreData> & core_data, const std::shared_ptr<RouteHandler> & route_handler,
  const std::shared_ptr<VehicleInfo> & vehicle_info)
: core_data_{core_data},
  route_handler_{route_handler},
  vehicle_info_{vehicle_info},
  metrics_(static_cast<size_t>(METRIC::SIZE), std::vector<double>(core_data->points->size(), 0.0)),
  scores_(std::make_shared<std::vector<double>>(static_cast<size_t>(SCORE::SIZE), 0.0))
{
  for (size_t i = 0; i < core_data_->points->size(); i++) {
    metrics_.at(static_cast<size_t>(METRIC::LATERAL_ACCEL)).at(i) = lateral_accel(i);
    metrics_.at(static_cast<size_t>(METRIC::LONGITUDINAL_JERK)).at(i) = longitudinal_jerk(i);
    metrics_.at(static_cast<size_t>(METRIC::TRAVEL_DISTANCE)).at(i) = travel_distance(i);
    metrics_.at(static_cast<size_t>(METRIC::LATERAL_DEVIATION)).at(i) = lateral_deviation(i);
  }

  // metrics_.at(static_cast<size_t>(METRIC::MINIMUM_TTC)) =
  //   autoware::trajectory_selector::utils::time_to_collision(
  //     core_data_->points, core_data_->objects, vehicle_info_);
}

void DataInterface::set_previous_points(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  previous_points_ = previous_points;
}

void DataInterface::setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  set_previous_points(previous_points);

  for (size_t i = 0; i < core_data_->points->size(); i++) {
    metrics_.at(static_cast<size_t>(METRIC::TRAJECTORY_DEVIATION)).at(i) = trajectory_deviation(i);
  }
}

void DataInterface::compress(const std::vector<std::vector<double>> & weight)
{
  scores_->at(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY)) =
    compress(weight, METRIC::LATERAL_ACCEL);
  scores_->at(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY)) =
    compress(weight, METRIC::LONGITUDINAL_JERK);
  scores_->at(static_cast<size_t>(SCORE::EFFICIENCY)) = compress(weight, METRIC::TRAVEL_DISTANCE);
  scores_->at(static_cast<size_t>(SCORE::SAFETY)) = compress(weight, METRIC::MINIMUM_TTC);
  scores_->at(static_cast<size_t>(SCORE::ACHIEVABILITY)) =
    compress(weight, METRIC::LATERAL_DEVIATION);
  scores_->at(static_cast<size_t>(SCORE::CONSISTENCY)) =
    compress(weight, METRIC::TRAJECTORY_DEVIATION);
}

double DataInterface::lateral_accel(const size_t idx) const
{
  const auto radius =
    vehicle_info_->wheel_base_m / std::tan(core_data_->points->at(idx).front_wheel_angle_rad);
  const auto speed = core_data_->points->at(idx).longitudinal_velocity_mps;
  return std::abs(speed * speed / radius);
}

double DataInterface::longitudinal_jerk(const size_t idx) const
{
  if (idx + 2 > core_data_->points->size()) return 0.0;

  const auto jerk = (core_data_->points->at(idx + 1).acceleration_mps2 -
                     core_data_->points->at(idx).acceleration_mps2) /
                    0.5;
  return std::abs(jerk);
}

double DataInterface::minimum_ttc([[maybe_unused]] const size_t idx) const
{
  // TODO(satoshi-ota): linear interpolation
  // return utils::time_to_collision(core_data_->points, core_data_->objects, idx);
  return 0.0;
}

double DataInterface::travel_distance(const size_t idx) const
{
  return autoware::motion_utils::calcSignedArcLength(*core_data_->points, 0L, idx);
}

double DataInterface::lateral_deviation(const size_t idx) const
{
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(
    *core_data_->preferred_lanes, autoware::universe_utils::getPose(core_data_->points->at(idx)));
  return std::abs(arc_coordinates.distance);
}

double DataInterface::trajectory_deviation(const size_t idx) const
{
  if (previous_points_ == nullptr) return 0.0;

  if (idx + 1 > previous_points_->size()) return 0.0;

  const auto & p1 = core_data_->points->at(idx).pose;
  const auto & p2 = previous_points_->at(idx).pose;
  return autoware::universe_utils::calcSquaredDistance2d(p1, p2);
}

bool DataInterface::feasible() const
{
  const auto idx = autoware::motion_utils::findNearestIndex(
    *core_data_->points, core_data_->odometry->pose.pose.position);
  const auto & p1 = core_data_->points->at(idx).pose.position;
  const auto & p2 = core_data_->odometry->pose.pose.position;
  if (autoware::universe_utils::calcSquaredDistance2d(p1, p2) > 10.0) {
    return false;
  }

  const auto condition = [](const auto & p) { return p.longitudinal_velocity_mps >= 0.0; };
  return std::all_of(core_data_->points->begin(), core_data_->points->end(), condition);
}

void DataInterface::normalize(
  const double min, const double max, const SCORE & score_type, const bool flip)
{
  const auto idx = static_cast<size_t>(score_type);
  if (std::abs(max - min) < std::numeric_limits<double>::epsilon()) {
    scores_->at(idx) = 1.0;
  } else {
    scores_->at(idx) =
      flip ? (max - scores_->at(idx)) / (max - min) : (scores_->at(idx) - min) / (max - min);
  }
}

auto DataInterface::compress(
  const std::vector<std::vector<double>> & weight, const METRIC & metric_type) const -> double
{
  const auto & w = weight.at(static_cast<size_t>(metric_type));
  const auto & metric = metrics_.at(static_cast<size_t>(metric_type));
  return std::inner_product(w.begin(), w.end(), metric.begin(), 0.0);
}

auto DataInterface::score(const SCORE & score_type) const -> double
{
  return scores_->at(static_cast<size_t>(score_type));
}

void DataInterface::weighting(const std::vector<double> & weight)
{
  total_ = std::inner_product(weight.begin(), weight.end(), (*scores_).begin(), 0.0);
}

}  // namespace autoware::trajectory_selector
