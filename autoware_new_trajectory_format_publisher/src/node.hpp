// Copyright 2025 TIER IV, Inc.
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

#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::new_trajectory_format_publisher
{
using autoware_new_planning_msgs::msg::Trajectories;

class NewTrajectoryFormatPublisher : public rclcpp::Node
{
public:
  explicit NewTrajectoryFormatPublisher(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectories>::SharedPtr pub_trajectores_;
  rclcpp::Publisher<Trajectories>::SharedPtr pub_resampled_trajectores_;
  std::shared_ptr<route_handler::RouteHandler> route_handler_;
  TrajectoryPoints centerline_trajectory_;

  void publish();
  TrajectoryPoints generate_centerline_path(
    const std::string & package_name, const std::string & map_name, const int route_start_lane_id,
    const int route_goal_lane_id);
  std::vector<TrajectoryPoints> generate_path();
  std::vector<TrajectoryPoints> generate_resampled_path(
    const std::vector<TrajectoryPoints> & trajectories);
};
}  // namespace autoware::trajectory_selector::new_trajectory_format_publisher

#endif  // NODE_HPP_
