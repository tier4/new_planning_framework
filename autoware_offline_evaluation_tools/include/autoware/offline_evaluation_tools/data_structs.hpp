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

#ifndef AUTOWARE__OFFLINE_EVALUATION_TOOLS__DATA_STRUCTS_HPP_
#define AUTOWARE__OFFLINE_EVALUATION_TOOLS__DATA_STRUCTS_HPP_

#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
struct TargetStateParameters
{
  std::vector<double> lat_positions{};
  std::vector<double> lat_velocities{};
  std::vector<double> lat_accelerations{};
  std::vector<double> lon_positions{};
  std::vector<double> lon_velocities{};
  std::vector<double> lon_accelerations{};
};

struct DataAugmentParameters
{
  size_t sample_num{20};

  double resolution{0.5};

  TargetStateParameters target_state{};
};

struct Result
{
  Result(
    const double w0, const double w1, const double w2, const double w3, const double w4,
    const double w5)
  : weight{w0, w1, w2, w3, w4, w5}
  {
  }
  std::shared_ptr<TrajectoryPoints> previous_points{nullptr};
  std::vector<double> weight;
  double loss{0.0};
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // AUTOWARE__OFFLINE_EVALUATION_TOOLS__DATA_STRUCTS_HPP_
