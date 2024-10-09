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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_STRUCTS_HPP_

#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector
{
enum class METRIC {
  LATERAL_ACCEL = 0,
  LONGITUDINAL_JERK = 1,
  TRAVEL_DISTANCE = 2,
  MINIMUM_TTC = 3,
  LATERAL_DEVIATION = 4,
  TRAJECTORY_DEVIATION = 5,
  SIZE
};

enum class SCORE {
  LATERAL_COMFORTABILITY = 0,
  LONGITUDINAL_COMFORTABILITY = 1,
  EFFICIENCY = 2,
  SAFETY = 3,
  ACHIEVABILITY = 4,
  CONSISTENCY = 5,
  SIZE
};

struct EvaluatorParameters
{
  explicit EvaluatorParameters(const size_t sample_num)
  : sample_num{sample_num},
    time_decay_weight(static_cast<size_t>(METRIC::SIZE), std::vector<double>(sample_num, 0.0)),
    score_weight(static_cast<size_t>(SCORE::SIZE), 0.0)
  {
  }

  size_t sample_num;

  double resolution;

  std::vector<std::vector<double>> time_decay_weight;

  std::vector<double> score_weight;
};

struct CoreData
{
  CoreData(
    const std::shared_ptr<TrajectoryPoints> & points,
    const std::shared_ptr<PredictedObjects> & objects, const std::shared_ptr<Odometry> & odometry,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes, const std::string & tag)
  : original{points},
    points{points},
    objects{objects},
    odometry{odometry},
    preferred_lanes{preferred_lanes},
    tag{tag}
  {
  }

  CoreData(
    const std::shared_ptr<TrajectoryPoints> & original,
    const std::shared_ptr<TrajectoryPoints> & points,
    const std::shared_ptr<PredictedObjects> & objects, const std::shared_ptr<Odometry> & odometry,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes, const Header & header,
    const UUID & generator_id)
  : original{original},
    points{points},
    previous_points{points},  // TODO(satoshi-ota): fix this line.
    objects{objects},
    odometry{odometry},
    preferred_lanes{preferred_lanes},
    tag{"__anon"},
    header{header},
    generator_id{generator_id}
  {
  }

  std::shared_ptr<TrajectoryPoints> original;

  std::shared_ptr<TrajectoryPoints> points;

  std::shared_ptr<TrajectoryPoints> previous_points;

  std::shared_ptr<PredictedObjects> objects;

  std::shared_ptr<Odometry> odometry;

  std::shared_ptr<lanelet::ConstLanelets> preferred_lanes;

  std::string tag;

  Header header;

  UUID generator_id;
};
}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__DATA_STRUCTS_HPP_
