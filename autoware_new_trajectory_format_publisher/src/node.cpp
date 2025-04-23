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

#include "qnamespace.h"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/shift.hpp>
#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <autoware/trajectory_selector_common/utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/time.hpp>

#include <autoware_new_planning_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <std_msgs/msg/detail/int8__builder.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::new_trajectory_format_publisher
{
using namespace std::literals::chrono_literals;
using autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo;
using Trajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

NewTrajectoryFormatPublisher::NewTrajectoryFormatPublisher(const rclcpp::NodeOptions & node_options)
: Node{"new_trajectory_format_publisher", node_options},
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&NewTrajectoryFormatPublisher::publish, this))},
  pub_trajectores_{this->create_publisher<autoware_new_planning_msgs::msg::Trajectories>(
    "/planning/candidate/trajectories", 1)},
  pub_resampled_trajectores_{this->create_publisher<autoware_new_planning_msgs::msg::Trajectories>(
    "~/output/resampled_trajectories", 1)},
  route_handler_{std::make_shared<route_handler::RouteHandler>()}
{
  // Straight line keep
  const size_t start_lane_id = 9102;
  const size_t end_lane_id = 124;

  // Curved line keep
  // const size_t start_lane_id = 9102;
  // const size_t end_lane_id = 112;

  centerline_trajectory_ =
    generate_centerline_path("autoware_test_utils", "lanelet2_map.osm", start_lane_id, end_lane_id);
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

  const auto generated_trajectories = generate_path();

  trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(generated_trajectories.front())
      .score(0.0));

  trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(generated_trajectories.back())  // Straight lane keep
      .score(0.0));

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(trajectories)
      .generator_info(generator_info);
  pub_trajectores_->publish(output);

  const auto resampled_trajectories = generate_resampled_path(generated_trajectories);

  std::vector<autoware_new_planning_msgs::msg::Trajectory> evaluated_trajectories{};
  evaluated_trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(resampled_trajectories.front())
      .score(0.0));
  evaluated_trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(resampled_trajectories.back())
      .score(0.0));

  const auto resampled_output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(evaluated_trajectories)
      .generator_info(generator_info);

  pub_resampled_trajectores_->publish(resampled_output);
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
  motion_utils::calculate_time_from_start(points, points.front().pose.position);
  return points;
}

std::vector<TrajectoryPoints> NewTrajectoryFormatPublisher::generate_path()
{
  std::vector<TrajectoryPoints> trajectories;

  const auto centerline_trajectory = autoware::experimental::trajectory::Trajectory<
                                       autoware_planning_msgs::msg::TrajectoryPoint>::Builder{}
                                       .build(centerline_trajectory_);

  autoware::experimental::trajectory::ShiftInterval interval{
    0.5 * centerline_trajectory->length(), centerline_trajectory->length(), 0.3};

  const autoware::experimental::trajectory::ShiftParameters shift_parameter{
    40 / 3.6,
    5.0,
  };

  auto offset_trajectory = autoware::experimental::trajectory::shift(
                             centerline_trajectory.value(), interval, shift_parameter)
                             ->trajectory.restore();
  autoware::motion_utils::calculate_time_from_start(
    offset_trajectory, offset_trajectory.begin()->pose.position);

  TrajectoryPoints slow_trajectory;

  for (const auto & point : centerline_trajectory_) {
    slow_trajectory.push_back(autoware_planning_msgs::build<TrajectoryPoint>()
                                .time_from_start(rclcpp::Duration(0, 0))
                                .pose(point.pose)
                                .longitudinal_velocity_mps(point.longitudinal_velocity_mps * 0.01)
                                .lateral_velocity_mps(point.lateral_velocity_mps)
                                .acceleration_mps2(0.0)
                                .heading_rate_rps(0.0)
                                .front_wheel_angle_rad(0.0)
                                .rear_wheel_angle_rad(0.0));
  }
  autoware::motion_utils::calculate_time_from_start(
    slow_trajectory, slow_trajectory.front().pose.position);

  trajectories.push_back(offset_trajectory);
  trajectories.push_back(slow_trajectory);
  return trajectories;
}

std::vector<TrajectoryPoints> NewTrajectoryFormatPublisher::generate_resampled_path(
  const std::vector<TrajectoryPoints> & trajectories)
{
  const auto current_pose = centerline_trajectory_.at(1).pose;
  std::vector<TrajectoryPoints> resampled_trajectories;
  for (const auto & trajectory : trajectories) {
    const auto resampled_points =
      autoware::trajectory_selector::utils::sampling(trajectory, current_pose, 20, 0.5);
    resampled_trajectories.push_back(resampled_points);
  }

  return resampled_trajectories;
}
}  // namespace autoware::trajectory_selector::new_trajectory_format_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::new_trajectory_format_publisher::NewTrajectoryFormatPublisher)
