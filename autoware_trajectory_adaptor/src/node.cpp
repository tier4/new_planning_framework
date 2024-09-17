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

#include <autoware/universe_utils/ros/marker_helper.hpp>

namespace autoware::trajectory_selector::trajectory_adaptor
{

TrajectoryAdaptorNode::TrajectoryAdaptorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_adaptor_node", node_options},
  sub_trajectories_{this->create_subscription<InputMsgType>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryAdaptorNode::process, this, std::placeholders::_1))},
  pub_trajectory_{this->create_publisher<OutputMsgType>("~/output/trajectory", 1)},
  pub_marker_{this->create_publisher<MarkerArray>("~/output/markers", 1)}
{
}

void TrajectoryAdaptorNode::process(const InputMsgType::ConstSharedPtr msg)
{
  if (msg->trajectories.empty()) {
    return;
  }

  visualize(msg);

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

  RCLCPP_INFO_STREAM(
    this->get_logger(), "best generator:" << best_generator(trajectory_itr->generator_id)
                                          << " score:" << trajectory_itr->score);

  const auto trajectory = autoware_planning_msgs::build<OutputMsgType>()
                            .header(trajectory_itr->header)
                            .points(trajectory_itr->points);
  pub_trajectory_->publish(trajectory);
}

void TrajectoryAdaptorNode::visualize(const InputMsgType::ConstSharedPtr msg)
{
  using autoware::universe_utils::createDefaultMarker;
  using autoware::universe_utils::createMarkerColor;
  using autoware::universe_utils::createMarkerScale;

  MarkerArray output;

  size_t i = 0L;
  for (const auto & trajectory : msg->trajectories) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "concatenate", i++, Marker::LINE_STRIP,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(0.0, 0.0, 1.0, 0.999));
    for (const auto & point : trajectory.points) {
      marker.points.push_back(point.pose.position);
      marker.colors.push_back(createMarkerColor(0.0, 0.0, 1.0, 0.999));
    }
    output.markers.push_back(marker);
  }

  pub_marker_->publish(output);
}

}  // namespace autoware::trajectory_selector::trajectory_adaptor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_adaptor::TrajectoryAdaptorNode)
