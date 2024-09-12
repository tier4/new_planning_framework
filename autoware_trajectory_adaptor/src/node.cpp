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

namespace autoware::trajectory_selector::trajectory_adaptor
{

TrajectoryAdaptorNode::TrajectoryAdaptorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_adaptor_node", node_options},
  sub_trajectories_{this->create_subscription<InputMsgType>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryAdaptorNode::process, this, std::placeholders::_1))},
  pub_trajectory_{this->create_publisher<OutputMsgType>("~/output/trajectory", 1)}
{
}

void TrajectoryAdaptorNode::process(const InputMsgType::ConstSharedPtr msg)
{
  if (msg->trajectories.empty()) {
    return;
  }

  const auto itr = std::max_element(
    msg->trajectories.begin(), msg->trajectories.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });
  if (itr == msg->trajectories.end()) {
    return;
  }

  const auto trajectory =
    autoware_planning_msgs::build<OutputMsgType>().header(itr->header).points(itr->points);
  pub_trajectory_->publish(trajectory);
}

}  // namespace autoware::trajectory_selector::trajectory_adaptor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_adaptor::TrajectoryAdaptorNode)
