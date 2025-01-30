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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR_COMMON__EVALUATION_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR_COMMON__EVALUATION_HPP_

#include "autoware/trajectory_selector_common/interface/data_interface.hpp"
#include "autoware/trajectory_selector_common/interface/metrics_interface.hpp"
#include "autoware/trajectory_selector_common/structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware_new_planning_msgs/msg/evaluation_info.hpp"

#include <pluginlib/class_loader.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector
{
using autoware_new_planning_msgs::msg::EvaluationInfo;
class Evaluator
{
public:
  explicit Evaluator(
    const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info)
  : plugin_loader_(
      "autoware_trajectory_selector_common", "autoware::trajectory_selector::MetricInterface"),
    route_handler_{route_handler},
    vehicle_info_{vehicle_info}
  {
  }

  void load_metric(const std::string & name, const size_t index, const double time_resolution);

  void unload_metric(const std::string & name);

  void add(const std::shared_ptr<CoreData> & core_data);

  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points);

  auto best(
    const std::shared_ptr<EvaluatorParameters> & parameters,
    const std::string & exclude = "") -> std::shared_ptr<DataInterface>;

  void clear() { results_.clear(); }

  auto results() const -> std::vector<std::shared_ptr<DataInterface>> { return results_; }

  auto get(const std::string & tag) const -> std::shared_ptr<DataInterface>;

  auto statistics(const size_t metric_index) const -> std::pair<double, double>;

  void show() const;

  auto marker() const -> std::shared_ptr<MarkerArray>;

  auto score_debug(const std::shared_ptr<EvaluatorParameters> & parameters) const -> std::shared_ptr<EvaluationInfo>;

protected:
  void pruning();

  void evaluate();

  void compress(const std::vector<std::vector<double>> & weight);

  void normalize();

  void weighting(const std::vector<double> & weight);

  auto best(const std::string & exclude = "") const -> std::shared_ptr<DataInterface>;

  auto route_handler() const -> std::shared_ptr<RouteHandler> { return route_handler_; }

  auto vehicle_info() const -> std::shared_ptr<VehicleInfo> { return vehicle_info_; }

private:
  pluginlib::ClassLoader<MetricInterface> plugin_loader_;

  std::vector<std::shared_ptr<MetricInterface>> plugins_;

  std::vector<std::shared_ptr<DataInterface>> results_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;
};
}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__EVALUATION_HPP_
