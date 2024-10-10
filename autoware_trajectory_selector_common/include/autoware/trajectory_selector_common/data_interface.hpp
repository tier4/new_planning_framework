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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory_selector_common/data_structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector
{

class DataInterface
{
public:
  explicit DataInterface(const std::shared_ptr<CoreData> & core_data, const size_t metrics_num)
  : core_data_{core_data},
    metrics_(metrics_num, std::vector<double>(core_data->points->size(), 0.0)),
    scores_(metrics_num, 0.0)
  {
  }

  void set_previous_points(const std::shared_ptr<TrajectoryPoints> & previous_points)
  {
    previous_points_ = previous_points;
  }

  // TODO(satoshi-ota): use previous points
  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
  {
    set_previous_points(previous_points);
  }

  void compress(const std::vector<std::vector<double>> & weight)
  {
    for (size_t i = 0; i < metrics_.size(); i++) {
      const auto & w = weight.at(i);
      const auto & metric = metrics_.at(i);
      scores_.at(i) = std::inner_product(w.begin(), w.end(), metric.begin(), 0.0);
    }
  }

  void normalize(const double min, const double max, const size_t index, const bool flip = false)
  {
    if (std::abs(max - min) < std::numeric_limits<double>::epsilon()) {
      scores_.at(index) = 1.0;
    } else {
      scores_.at(index) =
        flip ? (max - scores_.at(index)) / (max - min) : (scores_.at(index) - min) / (max - min);
    }
  }

  void weighting(const std::vector<double> & weight)
  {
    total_ = std::inner_product(weight.begin(), weight.end(), scores_.begin(), 0.0);
  }

  bool feasible() const
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

  void set_metric(const size_t idx, const std::vector<double> & metric)
  {
    metrics_.at(idx) = metric;
  }

  auto total() const -> double { return total_; };

  auto score(const size_t index) const -> double { return scores_.at(index); }

  auto points() const -> std::shared_ptr<TrajectoryPoints> { return core_data_->points; }

  auto previous() const -> std::shared_ptr<TrajectoryPoints> { return core_data_->previous_points; }

  auto original() const -> std::shared_ptr<TrajectoryPoints> { return core_data_->original; }

  auto objects() const -> std::shared_ptr<PredictedObjects> { return core_data_->objects; }

  auto preferred_lanes() const -> std::shared_ptr<lanelet::ConstLanelets>
  {
    return core_data_->preferred_lanes;
  }

  auto header() const -> Header { return core_data_->header; }

  auto uuid() const -> UUID { return core_data_->generator_id; }

  auto tag() const -> std::string { return core_data_->tag; }

private:
  std::shared_ptr<CoreData> core_data_;

  std::shared_ptr<TrajectoryPoints> previous_points_;

  std::vector<std::vector<double>> metrics_;

  std::vector<double> scores_;

  double total_;
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_INTERFACE_HPP_
