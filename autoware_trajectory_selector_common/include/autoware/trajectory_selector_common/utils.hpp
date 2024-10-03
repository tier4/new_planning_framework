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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_

#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <memory>
#include <string>

namespace autoware::trajectory_selector::utils
{
auto time_to_collision(
  const std::shared_ptr<TrajectoryPoints> & points,
  const std::shared_ptr<PredictedObjects> & objects, const size_t idx) -> double;

auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double time_resolution) -> TrajectoryPoints;

auto to_marker(
  const std::shared_ptr<TrajectoryPoints> & points, const double score, const bool feasible,
  const std::string & ns, const size_t id) -> Marker;
}  // namespace autoware::trajectory_selector::utils

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_
