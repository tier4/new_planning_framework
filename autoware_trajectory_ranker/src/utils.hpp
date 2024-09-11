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

#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory_filter_interface/type_alias.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace autoware::trajectory_selector::trajectory_ranker
{

double lateral_accel(
  const VehicleInfo & vehicle_info, const TrajectoryPoints & points, const size_t idx);

double longitudinal_jerk(const TrajectoryPoints & points, const size_t idx);

double minimum_ttc(
  const PredictedObjects & objects, const TrajectoryPoints & points, const size_t idx);

double travel_distance(const TrajectoryPoints & points, const size_t idx);

double longitudinal_comfortability(const std::vector<double> & values, const double resample_num);

double lateral_comfortability(const std::vector<double> & values, const double resample_num);

double efficiency(const std::vector<double> & values, const double resample_num);

double safety(const std::vector<double> & values, const double resample_num);

auto resampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t resample_num,
  const double time_resolution) -> TrajectoryPoints;

}  // namespace autoware::trajectory_selector::trajectory_ranker

#endif  // UTILS_HPP_
