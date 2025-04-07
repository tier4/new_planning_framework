// Copyright 2025 TIER IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware/trajectory_selector_common/type_alias.hpp>

namespace autoware::trajectory_selector::valid_trajectory_filter::utils
{
lanelet::ConstLanelets get_lanes_from_trajectory(
  const TrajectoryPoints & trajectory, const lanelet::ConstLanelets & lanelets);
}  // namespace autoware::trajectory_selector::valid_trajectory_filter::utils

#endif  // UTILS_HPP_
