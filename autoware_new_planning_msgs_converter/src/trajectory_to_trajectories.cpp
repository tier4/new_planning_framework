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

#include "trajectory_to_trajectories.hpp"

namespace autoware::trajectory_selector::new_planning_msgs_converter
{
TrajectoryToTrajectories::TrajectoryToTrajectories(const rclcpp::NodeOptions & options)
: ConverterBase("trajectory_to_trajectories", options),
  generator_name_(declare_parameter<std::string>("generator_name")),
  generator_uuid_(autoware::universe_utils::generateUUID())
{
}

void TrajectoryToTrajectories::process(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  const auto new_trajectory =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(msg->header)
      .generator_id(generator_uuid_)
      .points(msg->points)
      .score(0.0);

  const auto generator_name = std_msgs::build<std_msgs::msg::String>().data(generator_name_);

  const auto generator_info =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo>()
      .generator_id(generator_uuid_)
      .generator_name(generator_name);

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories({new_trajectory})
      .generator_info({generator_info});

  pub_->publish(output);
}

}  // namespace autoware::trajectory_selector::new_planning_msgs_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::new_planning_msgs_converter::TrajectoryToTrajectories)
