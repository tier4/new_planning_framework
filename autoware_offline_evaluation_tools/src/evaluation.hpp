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

#include "bag_handler.hpp"
#include "type_alias.hpp"

#include <autoware/trajectory_evaluator/evaluation.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
class BagEvaluator : public trajectory_evaluator::Evaluator
{
public:
  BagEvaluator(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<DataAugmentParameters> & parameters)
  : trajectory_evaluator::Evaluator{route_handler, vehicle_info},
    tf_{std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
          ->get(bag_data->timestamp)},
    odometry_{std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY))
                ->get(bag_data->timestamp)},
    objects_{objects(bag_data, parameters)},
    ground_truth_{ground_truth(bag_data, parameters)},
    augment_data_{augment_data(bag_data, vehicle_info, parameters)}
  {
  }

  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points);

  auto loss(const std::shared_ptr<EvaluatorParameters> & parameters)
    -> std::pair<double, std::shared_ptr<TrajectoryPoints>>;

  auto tf() const -> std::shared_ptr<TFMessage> { return tf_; };

  auto objects() const -> std::shared_ptr<PredictedObjects> { return objects_; }

  auto marker() const -> std::shared_ptr<MarkerArray>;

private:
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

  std::shared_ptr<TFMessage> tf_;

  std::shared_ptr<Odometry> odometry_;

  std::shared_ptr<PredictedObjects> objects_;

  std::shared_ptr<TrajectoryPoints> ground_truth_;

  std::vector<std::shared_ptr<TrajectoryPoints>> augment_data_;
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // EVALUATION_HPP_
