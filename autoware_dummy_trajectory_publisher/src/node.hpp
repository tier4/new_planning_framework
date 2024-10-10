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

#include "autoware/offline_evaluation_tools/data_structs.hpp"
#include "autoware/trajectory_selector_common/structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware_dummy_trajectory_publisher_param.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_new_planning_msgs/msg/trajectories.hpp"
#include "autoware_new_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::dummy_trajectory_publisher
{

using TrajectoryPoints = std::vector<TrajectoryPoint>;

using InputMsgType = autoware_planning_msgs::msg::Trajectory;
using OutputMsgType = autoware_new_planning_msgs::msg::Trajectories;

class DummyTrajectoryPublisherNode : public rclcpp::Node
{
public:
  explicit DummyTrajectoryPublisherNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const InputMsgType::ConstSharedPtr msg);

  auto parameters() const -> std::shared_ptr<offline_evaluation_tools::DataAugmentParameters>;

  rclcpp::Subscription<InputMsgType>::SharedPtr sub_trajectory_;

  rclcpp::Publisher<OutputMsgType>::SharedPtr pub_trajectories_;

  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};

  autoware::universe_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_acceleration_{this, "~/input/acceleration"};

  std::shared_ptr<augment::ParamListener> listener_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  std::string generator_name_;

  UUID generator_uuid_;
};

}  // namespace autoware::trajectory_selector::dummy_trajectory_publisher

#endif  // NODE_HPP_
