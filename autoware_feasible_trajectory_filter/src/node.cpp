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

namespace autoware::trajectory_selector::feasible_trajectory_filter
{

FeasibleTrajectoryFilterNode::FeasibleTrajectoryFilterNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"feasible_trajectory_filter_node", node_options},
  listener_{std::make_unique<feasible::ParamListener>(get_node_parameters_interface())}
{
  debug_processing_time_detail_pub_ =
    create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ =
    std::make_shared<autoware::universe_utils::TimeKeeper>(debug_processing_time_detail_pub_);
}

void FeasibleTrajectoryFilterNode::process(const Trajectories::ConstSharedPtr msg)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  publish(msg);
}

}  // namespace autoware::trajectory_selector::feasible_trajectory_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::feasible_trajectory_filter::FeasibleTrajectoryFilterNode)
