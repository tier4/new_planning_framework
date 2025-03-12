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

#ifndef EVALUATION_HPP_
#define EVALUATION_HPP_

#include "autoware/offline_evaluation_tools/data_structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "bag_handler.hpp"
#include "structs.hpp"

#include <autoware/trajectory_selector_common/evaluation.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

using autoware_planning_msgs::msg::Trajectory;

class BagEvaluator : public Evaluator
{
public:
  BagEvaluator(
    const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<DataAugmentParameters> & parameters);

  void setup(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & previous_points,
    const bool create_augmented_data = true);

  auto loss(const std::shared_ptr<EvaluatorParameters> & parameters)
    -> std::pair<double, std::shared_ptr<TrajectoryPoints>>;

  std::vector<TrajectoryWithMetrics> calc_metric_values(
    const size_t metrics_size, std::shared_ptr<TrajectoryPoints> & previous_points);

  auto tf() const -> std::shared_ptr<TFMessage> { return tf_; };

  auto objects() const -> std::shared_ptr<PredictedObjects> { return objects_; }

  auto trajectory() const -> std::shared_ptr<Trajectory> { return trajectory_; }

  auto marker() const -> std::shared_ptr<MarkerArray>;

private:
  auto preferred_lanes(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<RouteHandler> & route_handler)
    const -> std::shared_ptr<lanelet::ConstLanelets>;

  auto objects(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<DataAugmentParameters> & parameters) const
    -> std::shared_ptr<PredictedObjects>;

  auto ground_truth(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<DataAugmentParameters> & parameters) const
    -> std::shared_ptr<TrajectoryPoints>;

  auto augment_data(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<DataAugmentParameters> & parameters) const
    -> std::vector<std::shared_ptr<TrajectoryPoints>>;

  std::shared_ptr<DataAugmentParameters> parameters_;

  std::shared_ptr<TFMessage> tf_;

  std::shared_ptr<Odometry> odometry_;

  std::shared_ptr<SteeringReport> steering_;

  std::shared_ptr<PredictedObjects> objects_;

  std::shared_ptr<Trajectory> trajectory_;

  std::shared_ptr<lanelet::ConstLanelets> preferred_lanes_;
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // EVALUATION_HPP_
