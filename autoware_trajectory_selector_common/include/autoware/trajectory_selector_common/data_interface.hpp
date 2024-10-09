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
  explicit DataInterface(const std::shared_ptr<CoreData> & core_data, const size_t metrics_num);

  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points);

  void set_previous_points(const std::shared_ptr<TrajectoryPoints> & previous_points);

  void compress(const std::vector<std::vector<double>> & weight);

  void normalize(const double min, const double max, const size_t index, const bool flip = false);

  void weighting(const std::vector<double> & weight);

  auto total() const -> double { return total_; };

  bool feasible() const;

  void set_metric(const size_t idx, const std::vector<double> & metric)
  {
    metrics_.at(idx) = metric;
  }

  auto score(const size_t index) const -> double { return scores_->at(index); }

  auto scores() const -> std::shared_ptr<std::vector<double>> { return scores_; }

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

  auto marker() const -> std::shared_ptr<MarkerArray>;

private:
  void evaluate();

  std::shared_ptr<CoreData> core_data_;

  std::shared_ptr<TrajectoryPoints> previous_points_;

  std::vector<std::vector<double>> metrics_;

  std::shared_ptr<std::vector<double>> scores_;

  double total_;
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_INTERFACE_HPP_
