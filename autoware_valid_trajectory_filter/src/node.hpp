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

#include "autoware/trajectory_selector_common/interface/node_interface.hpp"

#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <map>
#include <memory>

namespace autoware::trajectory_selector::valid_trajectory_filter
{

struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
};

class ValidTrajectoryFilterNode : public TrajectoryFilterInterface
{
public:
  explicit ValidTrajectoryFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const Trajectories::ConstSharedPtr msg) override;

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  Trajectories::ConstSharedPtr traffic_light_check(const Trajectories::ConstSharedPtr msg);

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    traffic_signals_subscriber_{this, "~/input/traffic_signals"};

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  std::map<int64_t, TrafficSignalStamped> traffic_light_id_map_;
};

}  // namespace autoware::trajectory_selector::valid_trajectory_filter

#endif  // NODE_HPP_
