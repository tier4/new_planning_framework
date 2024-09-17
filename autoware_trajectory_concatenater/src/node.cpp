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

namespace autoware::trajectory_selector::trajectory_concatenater
{

TrajectoryConcatenaterNode::TrajectoryConcatenaterNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_concatenater_node", node_options},
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrajectoryConcatenaterNode::publish, this))},
  pub_trajectores_{this->create_publisher<Trajectories>("~/output/trajectories", 1)},
  parameters_{std::make_shared<parameters::ParamListener>(get_node_parameters_interface())}
{
  for (const auto & topic_name : parameters_->get_params().topic_names) {
    std::function<void(const Trajectories::ConstSharedPtr msg)> callback =
      std::bind(&TrajectoryConcatenaterNode::process, this, std::placeholders::_1, topic_name);
    subs_trajectories_.emplace(
      topic_name, this->create_subscription<Trajectories>(topic_name, 1, callback));
  }
}

void TrajectoryConcatenaterNode::process(
  const Trajectories::ConstSharedPtr msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.count(topic_name) == 0) {
    buffer_.emplace(topic_name, msg);
  } else {
    buffer_.at(topic_name) = msg;
  }
}

void TrajectoryConcatenaterNode::publish()
{
  std::vector<Trajectory> trajectories{};
  std::vector<TrajectoryGeneratorInfo> generator_info{};

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.empty()) return;

    for (const auto & [topic_name, msg] : buffer_) {
      trajectories.insert(trajectories.end(), msg->trajectories.begin(), msg->trajectories.end());
      generator_info.insert(
        generator_info.end(), msg->generator_info.begin(), msg->generator_info.end());
    }

    buffer_.clear();
  }

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(trajectories)
      .generator_info(generator_info);

  pub_trajectores_->publish(output);
}

}  // namespace autoware::trajectory_selector::trajectory_concatenater

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_concatenater::TrajectoryConcatenaterNode)
