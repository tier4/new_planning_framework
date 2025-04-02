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

#include "node.hpp"

#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/shift.hpp>
#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/time.hpp>

#include <autoware_new_planning_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/detail/int8__builder.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::new_trajectory_format_publisher
{
using namespace std::literals::chrono_literals;
using autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo;
using Trajectory = autoware::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

NewTrajectoryFormatPublisher::NewTrajectoryFormatPublisher(const rclcpp::NodeOptions & node_options)
: Node{"new_trajectory_format_publisher", node_options},
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&NewTrajectoryFormatPublisher::publish, this))},
  pub_trajectores_{this->create_publisher<autoware_new_planning_msgs::msg::Trajectories>(
    "~/output/trajectories", 1)},
  route_handler_{std::make_shared<route_handler::RouteHandler>()}
{
  centerline_trajectory_ =
    generate_centerline_path("autoware_test_utils", "lanelet2_map.osm", 9102, 124);
}

void NewTrajectoryFormatPublisher::publish()
{
  std::vector<autoware_new_planning_msgs::msg::Trajectory> trajectories{};
  std::vector<TrajectoryGeneratorInfo> generator_info{};
  const auto header = std_msgs::build<std_msgs::msg::Header>().stamp(this->now()).frame_id("map");
  const auto generator_name = std_msgs::build<std_msgs::msg::String>().data("test");
  generator_info.push_back(autoware_new_planning_msgs::build<TrajectoryGeneratorInfo>()
                             .generator_id(autoware_utils::generate_uuid())
                             .generator_name(generator_name));

  trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(centerline_trajectory_)
      .score(0.0));

  trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(generate_snake_path())
      .score(0.0));

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(trajectories)
      .generator_info(generator_info);
  pub_trajectores_->publish(output);
}

TrajectoryPoints NewTrajectoryFormatPublisher::generate_centerline_path(
  const std::string & package_name, const std::string & map_name, const int route_start_lane_id,
  const int route_goal_lane_id)
{
  const auto map_path =
    autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_name);
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(map_path, 0.1);
  route_handler_->setMap(map_bin_msg);

  const auto route = autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId(
    route_start_lane_id, route_goal_lane_id, package_name, map_name);
  route_handler_->setRoute(route);
  const auto path = route_handler_->getCenterLinePath(
    route_handler_->getPreferredLanelets(), 0.0, std::numeric_limits<double>::max());

  TrajectoryPoints points;
  for (const auto & point : path.points) {
    points.push_back(autoware_planning_msgs::build<TrajectoryPoint>()
                       .time_from_start(rclcpp::Duration(0, 0))
                       .pose(point.point.pose)
                       .longitudinal_velocity_mps(point.point.longitudinal_velocity_mps)
                       .lateral_velocity_mps(point.point.lateral_velocity_mps)
                       .acceleration_mps2(0.0)
                       .heading_rate_rps(0.0)
                       .front_wheel_angle_rad(0.0)
                       .rear_wheel_angle_rad(0.0));
  }
  return points;
}

TrajectoryPoints NewTrajectoryFormatPublisher::generate_snake_path()
{
  TrajectoryPoints new_points;

  const auto points = centerline_trajectory_;
  const auto trajectory = Trajectory::Builder{}.build(points);

  if (!trajectory.has_value()) return new_points;

  std::vector<trajectory::ShiftInterval> shift_intervals;

  double length = trajectory->length();
  for (size_t i = 0; i < 4; i++) {
    trajectory::ShiftInterval interval;
    interval.start = 0.25 * length * static_cast<double>(i);
    interval.end = 0.25 * length * static_cast<double>(i + 1);
    if (i == 0)
      interval.lateral_offset = 0.5;
    else
      interval.lateral_offset = std::pow(-1.0, i);
    shift_intervals.push_back(interval);
  }

  const auto shifted_point = autoware::trajectory::shift(trajectory.value(), shift_intervals);

  return shifted_point.restore();
}

}  // namespace autoware::trajectory_selector::new_trajectory_format_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::new_trajectory_format_publisher::NewTrajectoryFormatPublisher)
