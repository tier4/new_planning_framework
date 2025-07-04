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
#include <rclcpp/subscription.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

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

class OfflineEvaluatorNode : public rclcpp::Node
{
public:
  explicit OfflineEvaluatorNode(const rclcpp::NodeOptions & node_options);

  ~OfflineEvaluatorNode();

private:
  enum class EvaluationMode { OPEN_LOOP, CLOSED_LOOP };

  void setup_evaluation_bag_writer();
  void run_evaluation();
  rclcpp::Time run_open_loop_evaluation();
  std::pair<rclcpp::Time, rclcpp::Time> run_closed_loop_evaluation();
  void write_map_and_route_markers_to_bag(const rclcpp::Time & reference_time);
  void create_map_markers(visualization_msgs::msg::MarkerArray & marker_array) const;
  void create_route_markers(visualization_msgs::msg::MarkerArray & marker_array) const;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<MarkerArray>::SharedPtr sub_map_marker_;

  std::shared_ptr<RouteHandler> route_handler_;
  MarkerArray::ConstSharedPtr map_marker_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  mutable std::mutex mutex_;

  mutable rosbag2_cpp::Reader bag_reader_;

  std::unique_ptr<rosbag2_cpp::Writer> evaluation_bag_writer_;

  std::string route_topic_name_;
  std::string odometry_topic_name_;
  std::string trajectory_topic_name_;
  std::string objects_topic_name_;
  std::string tf_topic_name_;
  std::string acceleration_topic_name_;
  std::string steering_topic_name_;

  EvaluationMode evaluation_mode_;
  std::string bag_path_;

  rclcpp::TimerBase::SharedPtr map_check_timer_;
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // NODE_HPP_
