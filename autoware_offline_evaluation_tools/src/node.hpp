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
#include "rosbag2_cpp/writer.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
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
  
  ~OfflineEvaluatorNode();

private:
  void play(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void rewind(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void next_route(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  void weight(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  auto get_route() -> LaneletRoute::ConstSharedPtr;

  auto get_route_from_bag() -> LaneletRoute::ConstSharedPtr;

  void update(const std::shared_ptr<BagData> & bag_data, const double dt) const;

  void update_replay_data(const std::shared_ptr<ReplayEvaluationData> & replay_data, const double dt) const;

  void on_live_trajectory(const Trajectory::ConstSharedPtr msg);

  void setup_publishers();

  void setup_evaluation_bag_writer();

  void publish_replay_topics(const std::shared_ptr<ReplayEvaluationData> & replay_data) const;

  auto convert_trajectory_to_points(const Trajectory & trajectory) const -> std::shared_ptr<TrajectoryPoints>;

  void analyze(const std::shared_ptr<BagData> & bag_data) const;

  void visualize(const std::shared_ptr<BagEvaluator> & bag_evaluator) const;

  void plot(const std::shared_ptr<BagEvaluator> & bag_evaluator) const;

  auto evaluator_parameters() -> std::shared_ptr<EvaluatorParameters>;

  auto data_augument_parameters() -> std::shared_ptr<DataAugmentParameters>;

  std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;

  std::map<std::string, bool> publish_enabled_;

  std::map<std::string, bool> publish_once_;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  rclcpp::Subscription<Trajectory>::SharedPtr sub_live_trajectory_;

  rclcpp::Service<Trigger>::SharedPtr srv_play_;

  rclcpp::Service<Trigger>::SharedPtr srv_rewind_;

  rclcpp::Service<Trigger>::SharedPtr srv_route_;

  rclcpp::Service<Trigger>::SharedPtr srv_weight_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  mutable std::mutex mutex_;

  mutable rosbag2_cpp::Reader replay_reader_;

  mutable rosbag2_cpp::Reader route_reader_;

  std::unique_ptr<rosbag2_cpp::Writer> evaluation_bag_writer_;

  std::shared_ptr<ReplayEvaluationData> current_replay_data_;
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // NODE_HPP_
