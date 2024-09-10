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

#include "node.hpp"

namespace autoware::trajectory_selector::trajectory_ranker
{

TrajectoryRankerNode::TrajectoryRankerNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"trajectory_ranker_node", node_options}
{
}

auto TrajectoryRankerNode::process(const Trajectories::ConstSharedPtr msg)
  -> std::optional<Trajectories>
{
  if (msg == nullptr) {
    return std::nullopt;
  }

  return *msg;
}

}  // namespace autoware::trajectory_selector::trajectory_ranker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_ranker::TrajectoryRankerNode)
