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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/trajectory_selector_common/interface/node_interface.hpp"
#include "autoware_feasible_trajectory_filter_param.hpp"

#include <autoware/universe_utils/system/time_keeper.hpp>

#include <memory>

namespace autoware::trajectory_selector::feasible_trajectory_filter
{

class FeasibleTrajectoryFilterNode : public TrajectoryFilterInterface
{
public:
  explicit FeasibleTrajectoryFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const Trajectories::ConstSharedPtr msg) override;

  std::unique_ptr<feasible::ParamListener> listener_;

  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_selector::feasible_trajectory_filter

#endif  // NODE_HPP_
