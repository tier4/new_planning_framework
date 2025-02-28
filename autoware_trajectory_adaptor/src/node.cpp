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

#include <autoware/motion_utils/trajectory/trajectory.hpp>

namespace autoware::trajectory_selector::trajectory_adaptor
{

TrajectoryAdaptorNode::TrajectoryAdaptorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_adaptor_node", node_options},
  sub_trajectories_{this->create_subscription<InputMsgType>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryAdaptorNode::process, this, std::placeholders::_1))},
  pub_trajectory_{this->create_publisher<OutputMsgType>("~/output/trajectory", 1)},
  hazard_signal_publisher_{create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1)}
{
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_adaptor", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);
}

void TrajectoryAdaptorNode::process(const InputMsgType::ConstSharedPtr msg)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (msg->trajectories.empty()) {
    return;
  }

  const auto trajectory_itr = std::max_element(
    msg->trajectories.begin(), msg->trajectories.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });
  if (trajectory_itr == msg->trajectories.end()) {
    return;
  }

  const auto best_generator = [&msg](const auto & uuid) {
    const auto generator_itr = std::find_if(
      msg->generator_info.begin(), msg->generator_info.end(),
      [&uuid](const auto & info) { return info.generator_id == uuid; });
    return generator_itr == msg->generator_info.end() ? "NOT FOUND"
                                                      : generator_itr->generator_name.data;
  };

  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "best generator:" << best_generator(trajectory_itr->generator_id)
                                          << " score:" << trajectory_itr->score);

  const auto traj_points = autoware::motion_utils::removeOverlapPoints(trajectory_itr->points);
  const auto trajectory = autoware_planning_msgs::build<OutputMsgType>()
                            .header(trajectory_itr->header)
                            .points(traj_points);

  pub_trajectory_->publish(trajectory);
  hazard_signal_publisher_->publish(autoware_vehicle_msgs::build<HazardLightsCommand>()
                                      .stamp(trajectory_itr->header.stamp)
                                      .command(0));
}

}  // namespace autoware::trajectory_selector::trajectory_adaptor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_adaptor::TrajectoryAdaptorNode)
