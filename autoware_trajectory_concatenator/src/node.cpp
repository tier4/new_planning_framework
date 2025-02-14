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

#include "autoware_trajectory_concatenator_param.hpp"
#include "structs.hpp"

#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <chrono>
#include <memory>

namespace autoware::trajectory_selector::trajectory_concatenator
{

TrajectoryConcatenatorNode::TrajectoryConcatenatorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_concatenator_node", node_options},
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrajectoryConcatenatorNode::publish, this))},
  subs_trajectories_{this->create_subscription<Trajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryConcatenatorNode::on_trajectories, this, std::placeholders::_1))},
  pub_trajectores_{this->create_publisher<Trajectories>("~/output/trajectories", 1)},
  listener_{std::make_unique<concatenator::ParamListener>(get_node_parameters_interface())}
{
  debug_processing_time_detail_pub_ =
    create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms/trajectory_concatenator", 1);
  time_keeper_ =
    std::make_shared<autoware::universe_utils::TimeKeeper>(debug_processing_time_detail_pub_);
}

void TrajectoryConcatenatorNode::on_trajectories(const Trajectories::ConstSharedPtr msg)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & generator_info : msg->generator_info) {
    const auto uuid = autoware::universe_utils::toHexString(generator_info.generator_id);

    auto trajectories = msg->trajectories;
    const auto itr = std::remove_if(
      trajectories.begin(), trajectories.end(),
      [&generator_info](const auto & t) { return t.generator_id != generator_info.generator_id; });
    trajectories.erase(itr, trajectories.end());

    const auto pre_combine =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
        .trajectories(trajectories)
        .generator_info({generator_info});

    if (buffer_.count(uuid) == 0) {
      buffer_.emplace(uuid, std::make_shared<Trajectories>(pre_combine));
    } else {
      buffer_.at(uuid) = std::make_shared<Trajectories>(pre_combine);
    }
  }
}

void TrajectoryConcatenatorNode::publish()
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<Trajectory> trajectories{};
  std::vector<TrajectoryGeneratorInfo> generator_info{};

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.empty()) return;

    const auto current_time = this->now();
    const rclcpp::Duration expiration_time(
      std::chrono::nanoseconds(static_cast<int64_t>(parameters()->duration_time * 1e9)));

    for (auto it = buffer_.begin(); it != buffer_.end();) {
      const auto & [uuid, pre_combine] = *it;
      if (!pre_combine->trajectories.empty()) {
        const auto elapsed_time = (current_time - pre_combine->trajectories.begin()->header.stamp);
        if (elapsed_time > expiration_time) {
          it = buffer_.erase(it);
          continue;
        }
      }
      it++;
    }

    for (const auto & [uuid, pre_combine] : buffer_) {
      trajectories.insert(
        trajectories.end(), pre_combine->trajectories.begin(), pre_combine->trajectories.end());
      generator_info.insert(
        generator_info.end(), pre_combine->generator_info.begin(),
        pre_combine->generator_info.end());
    }
  }

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(trajectories)
      .generator_info(generator_info);

  pub_trajectores_->publish(output);
}

auto TrajectoryConcatenatorNode::parameters() const -> std::shared_ptr<ConcatenatorParam>
{
  const auto node_params = listener_->get_params();
  const auto parameters = std::make_shared<ConcatenatorParam>();

  parameters->duration_time = node_params.duration_time;

  return parameters;
}

}  // namespace autoware::trajectory_selector::trajectory_concatenator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_concatenator::TrajectoryConcatenatorNode)
