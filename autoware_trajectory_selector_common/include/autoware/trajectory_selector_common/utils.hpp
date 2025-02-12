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
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::utils
{
  auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double time_resolution) -> TrajectoryPoints;

auto sampling_with_time(
  const TrajectoryPoints & points, const size_t sample_num, const double resolution,
  const size_t start_idx) -> TrajectoryPoints;

auto find_nearest_timestamp(
  const TrajectoryPoints & points, const double target_timestamp,
  const size_t start_index) -> std::optional<size_t>;

auto to_marker(
  const std::shared_ptr<TrajectoryPoints> & points, const double score, const bool feasible,
  const std::string & ns, const size_t id) -> Marker;
}  // namespace autoware::trajectory_selector::utils

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__UTILS_HPP_
