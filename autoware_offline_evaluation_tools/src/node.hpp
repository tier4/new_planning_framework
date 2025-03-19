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

#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "bag_handler.hpp"
#include "evaluation.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <autoware_internal_debug_msgs/srv/string.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

using autoware_planning_msgs::msg::Trajectory;
using std_srvs::srv::Trigger;

class OfflineEvaluatorNode : public rclcpp::Node
{
public:
  explicit OfflineEvaluatorNode(const rclcpp::NodeOptions & node_options);

private:
  void play(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void rewind(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void next_route(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void weight(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void create_dataset(
    const autoware_internal_debug_msgs::srv::String::Request::SharedPtr request,
    [[maybe_unused]] const autoware_internal_debug_msgs::srv::String::Response::SharedPtr response);

  auto get_route() -> LaneletRoute::ConstSharedPtr;

  void update(const std::shared_ptr<BagData> & bag_data, const double dt) const;

  void analyze(const std::shared_ptr<BagData> & bag_data) const;

  void visualize(const std::shared_ptr<BagEvaluator> & bag_evaluator) const;

  void plot(const std::shared_ptr<BagEvaluator> & bag_evaluator) const;

  auto evaluator_parameters() -> std::shared_ptr<EvaluatorParameters>;

  auto data_augument_parameters() -> std::shared_ptr<DataAugmentParameters>;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;

  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;

  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  rclcpp::Service<Trigger>::SharedPtr srv_play_;

  rclcpp::Service<Trigger>::SharedPtr srv_rewind_;

  rclcpp::Service<Trigger>::SharedPtr srv_route_;

  rclcpp::Service<Trigger>::SharedPtr srv_weight_;

  rclcpp::Service<autoware_internal_debug_msgs::srv::String>::SharedPtr srv_create_dataset_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  mutable std::mutex mutex_;

  mutable rosbag2_cpp::Reader reader_;
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // NODE_HPP_
