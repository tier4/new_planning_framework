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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_INTERFACE_HPP_

#include "autoware/trajectory_selector_common/data_structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector
{

class DataInterface
{
public:
  DataInterface(
    const std::shared_ptr<CoreData> & core_data,
    const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info);

  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points);

  void set_previous_points(const std::shared_ptr<TrajectoryPoints> & previous_points);

  void compress(const std::vector<std::vector<double>> & weight);

  void normalize(
    const double min, const double max, const SCORE & score_type, const bool flip = false);

  void weighting(const std::vector<double> & weight);

  auto total() const -> double { return total_; };

  bool feasible() const;

  auto score(const SCORE & score_type) const -> double;

  auto scores() const -> std::shared_ptr<std::vector<double>> { return scores_; }

  auto points() const -> std::shared_ptr<TrajectoryPoints> { return core_data_->points; }

  auto original() const -> std::shared_ptr<TrajectoryPoints> { return core_data_->original; }

  auto header() const -> Header { return core_data_->header; }

  auto uuid() const -> UUID { return core_data_->generator_id; }

  auto tag() const -> std::string { return core_data_->tag; }

  auto marker() const -> std::shared_ptr<MarkerArray>;

private:
  void evaluate();

  auto lateral_accel(const size_t idx) const -> double;

  auto longitudinal_jerk(const size_t idx) const -> double;

  auto minimum_ttc(const size_t idx) const -> double;

  auto travel_distance(const size_t idx) const -> double;

  auto lateral_deviation(const size_t idx) const -> double;

  auto trajectory_deviation(const size_t idx) const -> double;

  auto compress(const std::vector<std::vector<double>> & weight, const METRIC & metric_type) const
    -> double;

  std::shared_ptr<CoreData> core_data_;

  std::shared_ptr<TrajectoryPoints> previous_points_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  std::vector<std::vector<double>> metrics_;

  std::shared_ptr<std::vector<double>> scores_;

  double total_;
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_INTERFACE_HPP_
