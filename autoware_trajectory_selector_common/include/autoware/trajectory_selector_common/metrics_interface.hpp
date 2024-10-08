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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR_COMMON__METRICS_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR_COMMON__METRICS_INTERFACE_HPP_

#include "autoware/trajectory_selector_common/data_interface.hpp"
#include "autoware/trajectory_selector_common/data_structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::trajectory_selector
{

class MetricInterface
{
public:
  MetricInterface(const MetricInterface &) = delete;
  MetricInterface(MetricInterface &&) = delete;
  MetricInterface & operator=(const MetricInterface &) = delete;
  MetricInterface & operator=(MetricInterface &&) = delete;
  explicit MetricInterface(std::string name) : name_{std::move(name)} {}

  virtual ~MetricInterface() = default;

  virtual void evaluate(const std::shared_ptr<DataInterface> & result) const = 0;

  auto name() const -> std::string { return name_; }

protected:
  std::string name_;
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR_COMMON__METRICS_INTERFACE_HPP_
