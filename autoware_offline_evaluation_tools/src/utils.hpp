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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware_frenet_planner/frenet_planner.hpp"
#include "autoware_path_sampler/prepare_inputs.hpp"
#include "autoware_path_sampler/utils/trajectory_utils.hpp"
#include "data_structs.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools::utils
{

using autoware_planning_msgs::msg::Trajectory;

auto augment(
  const Trajectory & trajectory, const Pose & p_ego, const double v_ego, const double a_ego,
  const std::shared_ptr<VehicleInfo> & vehicle_info,
  const std::shared_ptr<DataAugmentParameters> & parameters)
  -> std::vector<std::vector<TrajectoryPoint>>;
}  // namespace autoware::trajectory_selector::offline_evaluation_tools::utils

#endif  // UTILS_HPP_
