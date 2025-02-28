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

#ifndef TRAJECTORY_TO_TRAJECTORIES_HPP_
#define TRAJECTORY_TO_TRAJECTORIES_HPP_

#include "autoware/planning_topic_converter/converter_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

#include "autoware_new_planning_msgs/msg/trajectories.hpp"
#include "autoware_new_planning_msgs/msg/trajectory.hpp"
#include "autoware_new_planning_msgs/msg/trajectory_generator_info.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>

namespace autoware::trajectory_selector::new_planning_msgs_converter
{

using autoware::planning_topic_converter::ConverterBase;
using unique_identifier_msgs::msg::UUID;

class TrajectoryToTrajectories
: public ConverterBase<
    autoware_planning_msgs::msg::Trajectory, autoware_new_planning_msgs::msg::Trajectories>
{
public:
  explicit TrajectoryToTrajectories(const rclcpp::NodeOptions & options);

private:
  void process(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) override;

  std::string generator_name_;
  UUID generator_uuid_;
};

}  // namespace autoware::trajectory_selector::new_planning_msgs_converter

#endif  // TRAJECTORY_TO_TRAJECTORIES_HPP_
