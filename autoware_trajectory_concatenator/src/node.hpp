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

#include "structs.hpp"

#include <autoware_trajectory_concatenator_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include "autoware_new_planning_msgs/msg/trajectories.hpp"
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::trajectory_concatenator
{

using namespace std::literals::chrono_literals;
using autoware_new_planning_msgs::msg::Trajectories;
using autoware_new_planning_msgs::msg::Trajectory;
using autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo;
using nav_msgs::msg::Odometry;
class TrajectoryConcatenatorNode : public rclcpp::Node
{
public:
  explicit TrajectoryConcatenatorNode(const rclcpp::NodeOptions & node_options);

private:
  void on_trajectories(const Trajectories::ConstSharedPtr msg);

  void on_selected_trajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  void publish();

  auto parameters() const -> std::shared_ptr<ConcatenatorParam>;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Trajectories>::SharedPtr subs_trajectories_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_selected_trajectory_;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};

  rclcpp::Publisher<Trajectories>::SharedPtr pub_trajectores_;

  std::unique_ptr<concatenator::ParamListener> listener_;

  std::map<std::string, Trajectories::ConstSharedPtr> buffer_;

  mutable std::mutex mutex_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_selector::trajectory_concatenator

#endif  // NODE_HPP_
