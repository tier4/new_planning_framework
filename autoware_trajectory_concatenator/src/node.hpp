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

#include "autoware_trajectory_concatenator_param.hpp"
#include "structs.hpp"

#include <autoware/universe_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_new_planning_msgs/msg/trajectories.hpp"

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

class TrajectoryConcatenatorNode : public rclcpp::Node
{
public:
  explicit TrajectoryConcatenatorNode(const rclcpp::NodeOptions & node_options);

private:
  void on_trajectories(const Trajectories::ConstSharedPtr msg);

  void publish();

  auto parameters() const -> std::shared_ptr<ConcatenatorParam>;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Trajectories>::SharedPtr subs_trajectories_;

  rclcpp::Publisher<Trajectories>::SharedPtr pub_trajectores_;

  std::unique_ptr<concatenator::ParamListener> listener_;

  std::map<std::string, Trajectories::ConstSharedPtr> buffer_;

  mutable std::mutex mutex_;

  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_selector::trajectory_concatenator

#endif  // NODE_HPP_
