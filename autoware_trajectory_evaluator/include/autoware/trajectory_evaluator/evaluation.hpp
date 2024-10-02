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

#ifndef AUTOWARE__TRAJECTORY_EVALUATOR__EVALUATION_HPP_
#define AUTOWARE__TRAJECTORY_EVALUATOR__EVALUATION_HPP_

#include "autoware/trajectory_evaluator/data_structs.hpp"
#include "autoware/trajectory_evaluator/type_alias.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::trajectory_evaluator
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

class Evaluator
{
public:
  explicit Evaluator(
    const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info)
  : route_handler_{route_handler}, vehicle_info_{vehicle_info}
  {
  }

  void add(const std::shared_ptr<CoreData> & core_data);

  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points);

  auto best(
    const std::shared_ptr<EvaluatorParameters> & parameters,
    const std::string & exclude = "") -> std::shared_ptr<DataInterface>;

  void clear() { results_.clear(); }

  auto results() const -> std::vector<std::shared_ptr<DataInterface>> { return results_; }

  auto get(const std::string & tag) const -> std::shared_ptr<DataInterface>;

  auto statistics(const SCORE & score_type) const -> std::pair<double, double>;

  void show() const;

protected:
  void pruning();

  void compress(const std::vector<std::vector<double>> & weight);

  void normalize();

  void weighting(const std::vector<double> & weight);

  auto best(const std::string & exclude = "") const -> std::shared_ptr<DataInterface>;

private:
  std::vector<std::shared_ptr<DataInterface>> results_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;
};
}  // namespace autoware::trajectory_selector::trajectory_evaluator

#endif  // AUTOWARE__TRAJECTORY_EVALUATOR__EVALUATION_HPP_
