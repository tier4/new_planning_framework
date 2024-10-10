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

#ifndef AUTOWARE__TRAJECTORY_METRICS__METRICS_HPP_
#define AUTOWARE__TRAJECTORY_METRICS__METRICS_HPP_

#include "autoware/trajectory_selector_common/interface/metrics_interface.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::trajectory_metrics
{

class LateralAcceleration : public MetricInterface
{
public:
  LateralAcceleration() : MetricInterface{"LateralAcceleration"} {}

  void evaluate(const std::shared_ptr<DataInterface> & result) const override;

  bool is_deviation() const override { return true; }
};

class LongitudinalJerk : public MetricInterface
{
public:
  LongitudinalJerk() : MetricInterface{"LongitudinalJerk"} {}

  void evaluate(const std::shared_ptr<DataInterface> & result) const override;

  bool is_deviation() const override { return true; }
};

class TimeToCollision : public MetricInterface
{
public:
  TimeToCollision() : MetricInterface{"TimeToCollision"} {}

  void evaluate(const std::shared_ptr<DataInterface> & result) const override;

  bool is_deviation() const override { return false; }
};

class TravelDistance : public MetricInterface
{
public:
  TravelDistance() : MetricInterface{"TravelDistance"} {}

  void evaluate(const std::shared_ptr<DataInterface> & result) const override;

  bool is_deviation() const override { return false; }
};

class LateralDeviation : public MetricInterface
{
public:
  LateralDeviation() : MetricInterface{"LateralDeviation"} {}

  void evaluate(const std::shared_ptr<DataInterface> & result) const override;

  bool is_deviation() const override { return true; }
};

class TrajectoryDeviation : public MetricInterface
{
public:
  TrajectoryDeviation() : MetricInterface{"TrajectoryDeviation"} {}

  void evaluate(const std::shared_ptr<DataInterface> & result) const override;

  bool is_deviation() const override { return true; }
};

}  // namespace autoware::trajectory_selector::trajectory_metrics

#endif  // AUTOWARE__TRAJECTORY_METRICS__METRICS_HPP_
