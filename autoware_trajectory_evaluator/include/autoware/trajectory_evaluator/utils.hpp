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

#ifndef AUTOWARE__TRAJECTORY_EVALUATOR__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_EVALUATOR__UTILS_HPP_

#include "autoware/trajectory_evaluator/evaluation.hpp"
#include "autoware/trajectory_evaluator/type_alias.hpp"

#include <memory>

namespace autoware::trajectory_selector::trajectory_evaluator::utils
{
auto time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects, const size_t idx) -> double;

auto to_marker(
  const std::shared_ptr<DataInterface> & data, const SCORE & score_type, const size_t id) -> Marker;
}  // namespace autoware::trajectory_selector::trajectory_evaluator::utils

#endif  // AUTOWARE__TRAJECTORY_EVALUATOR__UTILS_HPP_
