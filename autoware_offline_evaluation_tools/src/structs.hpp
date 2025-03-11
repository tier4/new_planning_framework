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

#ifndef AUTOWARE__OFFLINE_EVALUATION_TOOLS__STRUCTS_HPP_
#define AUTOWARE__OFFLINE_EVALUATION_TOOLS__STRUCTS_HPP_

#include <array>
#include <vector>
#include <memory>

#include "autoware/trajectory_selector_common/type_alias.hpp"

namespace autoware::trajectory_selector
{
// サンプル点および軌道構造体の定義
struct TrajectoryPointWithMetrics {
  TrajectoryPoint point;
  std::vector<double> metrics;
};

struct TrajectoryWithMetrics {
  double time;
  std::vector<TrajectoryPointWithMetrics> points;
};
}  // namespace autoware::trajectory_selector
#endif  // AUTOWARE__OFFLINE_EVALUATION_TOOLS__STRUCTS_HPP_
