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

#include "utils.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Forward.h>

#include <algorithm>
#include <memory>

namespace autoware::trajectory_selector::valid_trajectory_filter
{

ValidTrajectoryFilterNode::ValidTrajectoryFilterNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"trajectory_ranker_node", node_options}
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ValidTrajectoryFilterNode::map_callback, this, std::placeholders::_1));
}

void ValidTrajectoryFilterNode::process(const Trajectories::ConstSharedPtr msg)
{
  publish(msg);
}

void ValidTrajectoryFilterNode::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void ValidTrajectoryFilterNode::traffic_light_check(const Trajectories::ConstSharedPtr msg)
{
  auto trajectories = msg->trajectories;
  const auto traffic_signal_msg = traffic_signals_subscriber_.take_data();

  if (traffic_signal_msg) {
    traffic_light_id_map_.clear();
    for (const auto & signal : traffic_signal_msg->traffic_light_groups) {
      TrafficSignalStamped traffic_signal;
      traffic_signal.stamp = traffic_signal_msg->stamp;
      traffic_signal.signal = signal;
      traffic_light_id_map_[signal.traffic_light_group_id] = traffic_signal;
    }
  }

  auto itr =
    std::remove_if(trajectories.begin(), trajectories.end(), [this](const auto & trajectory) {
      const lanelet::ConstLanelets lanelets(
        lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());
      const auto lanes = utils::get_lanes_from_trajectory(trajectory.points, lanelets);

      for (const auto & lane : lanes) {
        for (const auto & element : lane.template regulatoryElementsAs<lanelet::TrafficLight>()) {
          if (traffic_light_id_map_.count(element->id()) == 0) continue;

          if (autoware::traffic_light_utils::isTrafficSignalStop(
                lane, traffic_light_id_map_.at(element->id()).signal)) {
            return true;
          }
        }
      }

      return false;
    });
  trajectories.erase(itr);
}

}  // namespace autoware::trajectory_selector::valid_trajectory_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::valid_trajectory_filter::ValidTrajectoryFilterNode)
