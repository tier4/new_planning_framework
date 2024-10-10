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

DataInterface::DataInterface(const std::shared_ptr<CoreData> & core_data, const size_t metrics_num)
: core_data_{core_data},
  metrics_(metrics_num, std::vector<double>(core_data->points->size(), 0.0)),
  scores_(metrics_num, 0.0)
{
}

void DataInterface::set_previous_points(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  previous_points_ = previous_points;
}

// TODO(satoshi-ota): use previous points
void DataInterface::setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  set_previous_points(previous_points);
}

void DataInterface::compress(const std::vector<std::vector<double>> & weight)
{
  for (size_t i = 0; i < metrics_.size(); i++) {
    const auto & w = weight.at(i);
    const auto & metric = metrics_.at(i);
    scores_.at(i) = std::inner_product(w.begin(), w.end(), metric.begin(), 0.0);
  }
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
  const double min, const double max, const size_t index, const bool flip)
{
  if (std::abs(max - min) < std::numeric_limits<double>::epsilon()) {
    scores_.at(index) = 1.0;
  } else {
    scores_.at(index) =
      flip ? (max - scores_.at(index)) / (max - min) : (scores_.at(index) - min) / (max - min);
  }
}

void DataInterface::weighting(const std::vector<double> & weight)
{
  total_ = std::inner_product(weight.begin(), weight.end(), scores_.begin(), 0.0);
}

}  // namespace autoware::trajectory_selector
