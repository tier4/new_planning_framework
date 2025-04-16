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
using Trajectory = autoware::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

NewTrajectoryFormatPublisher::NewTrajectoryFormatPublisher(const rclcpp::NodeOptions & node_options)
: Node{"new_trajectory_format_publisher", node_options},
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&NewTrajectoryFormatPublisher::publish, this))},
  pub_trajectores_{this->create_publisher<autoware_new_planning_msgs::msg::Trajectories>(
    "~/output/trajectories", 1)},
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
      .points(generate_snake_path())  // Straight lane keep
      // .points(generate_shifted_path(-1.0))  // Curved lane keep
      .score(0.0));

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(trajectories)
      .generator_info(generator_info);
  pub_trajectores_->publish(output);

  const auto current_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(3714.1552734375).y(73718.0).z(19.339))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.00012232430968265882)
                     .y(-0.0005086549380674299)
                     .z(0.23381954091659465)
                     .w(0.972279871535182));

  const auto current_points =
    autoware::trajectory_selector::utils::sampling(centerline_trajectory_, current_pose, 20, 0.5);

  std::vector<autoware_new_planning_msgs::msg::Trajectory> resampled_trajectories{};
  resampled_trajectories.push_back(
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(header)
      .generator_id(autoware_utils::generate_uuid())
      .points(current_points)
      .score(0.0));

  const auto resampled_output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(resampled_trajectories)
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

  const auto centerline_trajectory =
    autoware::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>::Builder{}.build(
      centerline_trajectory_);

  std::vector<trajectory::ShiftInterval> shift_intervals;
  trajectory::ShiftInterval interval;
  interval.start = 0.5 * centerline_trajectory->length();
  interval.end = centerline_trajectory->length();
  interval.lateral_offset = 0.3;
  shift_intervals.push_back(interval);

  auto offset_trajectory =
    autoware::trajectory::shift(centerline_trajectory.value(), shift_intervals).restore();
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
}

std::vector<TrajectoryPoints> NewTrajectoryFormatPublisher::generate_resampled_path()
{
  TrajectoryPoints new_points;

  const auto points = centerline_trajectory_;
  const auto trajectory = Trajectory::Builder{}.build(points);

  if (!trajectory.has_value()) return new_points;

  std::vector<trajectory::ShiftInterval> shift_intervals;

  for (size_t i = 0; i < 2; i++) {
    trajectory::ShiftInterval interval;
    interval.start = trajectory->length() * 0.5 + 5.0 * static_cast<double>(i);
    interval.end = trajectory->length() * 0.5 + 10.0 * static_cast<double>(i + 1);
    interval.lateral_offset = std::pow(shift_offset, i);
    shift_intervals.push_back(interval);
  }
  const auto shifted_point = autoware::trajectory::shift(trajectory.value(), shift_intervals);
  return shifted_point.restore();
}
}  // namespace autoware::trajectory_selector::new_trajectory_format_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::new_trajectory_format_publisher::NewTrajectoryFormatPublisher)
