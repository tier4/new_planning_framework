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

#include "autoware/trajectory_filter_interface/interface.hpp"

namespace autoware::trajectory_selector::feasible_trajectory_filter
{

class FeasibleTrajectoryFilterNode : public TrajectoryFilterInterface
{
public:
  explicit FeasibleTrajectoryFilterNode(const rclcpp::NodeOptions & node_options);

private:
  auto process(const Trajectories::ConstSharedPtr msg) -> std::optional<Trajectories> override;
};

}  // namespace autoware::trajectory_selector::feasible_trajectory_filter

#endif  // NODE_HPP_
