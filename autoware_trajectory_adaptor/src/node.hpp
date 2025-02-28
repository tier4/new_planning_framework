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

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_new_planning_msgs/msg/trajectories.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>

namespace autoware::trajectory_selector::trajectory_adaptor
{

using InputMsgType = autoware_new_planning_msgs::msg::Trajectories;
using OutputMsgType = autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::HazardLightsCommand;

class TrajectoryAdaptorNode : public rclcpp::Node
{
public:
  explicit TrajectoryAdaptorNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const InputMsgType::ConstSharedPtr msg);

  rclcpp::Subscription<InputMsgType>::SharedPtr sub_trajectories_;

  rclcpp::Publisher<OutputMsgType>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr
    hazard_signal_publisher_;  // To-do (go-sakayori): decide where to cope with turn indicator and
                               // hazard signal

  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_selector::trajectory_adaptor

#endif  // NODE_HPP_
