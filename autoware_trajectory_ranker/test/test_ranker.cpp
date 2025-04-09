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

#include "../src/node.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware/trajectory_selector_common/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/shift.hpp>
#include <autoware/trajectory_selector_common/structs.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "nav_msgs/msg/detail/odometry__struct.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::trajectory_ranker
{
class TestTrajectoryRanker : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto node_options = rclcpp::NodeOptions{};
    node_options.arguments(std::vector<std::string>{
      "--ros-args", "--params-file",
      ament_index_cpp::get_package_share_directory("autoware_trajectory_ranker") +
        "/config/evaluation.param.yaml",
      "--params-file",
      ament_index_cpp::get_package_share_directory("autoware_test_utils") +
        "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<TrajectoryRankerNode>(node_options);
    sample_num = node_->parameters()->sample_num;
    resolution = node_->parameters()->resolution;
  }

  lanelet::ConstLanelets get_preferred_lanes(
    const std::string & package_name, const std::string & map_name, const int route_start_lane_id,
    const int route_goal_lane_id)
  {
    const auto map_path =
      autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_name);
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(map_path, 0.1);
    node_->route_handler_->setMap(map_bin_msg);

    const auto route = autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId(
      route_start_lane_id, route_goal_lane_id, package_name, map_name);
    node_->route_handler_->setRoute(route);
    return node_->route_handler_->getPreferredLanelets();
  }

  std::shared_ptr<EvaluatorParameters> get_parameters() { return node_->parameters(); }

  TrajectoryPoints create_centerline_path()
  {
    const auto preferred_lanes = node_->route_handler_->getPreferredLanelets();
    const auto path = node_->route_handler_->getCenterLinePath(
      preferred_lanes, 0.0, std::numeric_limits<double>::max());
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
    autoware::motion_utils::calculate_time_from_start(points, points.front().pose.position);
    return points;
  }

  void clear_data() { node_->evaluator_->clear(); }

  void add_data(const std::shared_ptr<CoreData> & core_data) { node_->evaluator_->add(core_data); }

  std::shared_ptr<DataInterface> get_best_trajectory()
  {
    return node_->evaluator_->best(get_parameters());
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<TrajectoryRankerNode> node_;
  size_t sample_num{};
  double resolution{};
};

TEST_F(TestTrajectoryRanker, straight_lane_keep)
{
  const auto preferred_lanes = std::make_shared<lanelet::ConstLanelets>(
    get_preferred_lanes("autoware_test_utils", "lanelet2_map.osm", 9102, 124));
  const auto centerline_points = create_centerline_path();
  std::shared_ptr<PredictedObjects> objects = std::make_shared<PredictedObjects>();
  const auto centerline_trajectory =
    autoware::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>::Builder{}.build(
      centerline_points);

  {
    // Test to check whether traveling distance is working

    const auto prev_pose = centerline_points.at(0).pose;
    const auto current_pose = centerline_points.at(1).pose;
    const auto previous_points =
      std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
        centerline_points, prev_pose, sample_num, resolution));

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

    const auto offset_points =
      std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
        offset_trajectory, current_pose, sample_num, resolution));

    TrajectoryPoints slow_trajectory;

    for (const auto & point : centerline_points) {
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

    const auto slow_points =
      std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
        slow_trajectory, current_pose, sample_num, resolution));

    add_data(std::make_shared<CoreData>(
      offset_points, previous_points, objects, preferred_lanes, "possible"));
    add_data(
      std::make_shared<CoreData>(slow_points, previous_points, objects, preferred_lanes, "bad"));

    const auto best = get_best_trajectory();
    EXPECT_NE(best->tag(), "bad");
  }

  {
    clear_data();
    std::vector<trajectory::ShiftInterval> shift_intervals;
    trajectory::ShiftInterval interval;
    interval.start = 0.0;
    interval.end = centerline_trajectory->length();
    interval.lateral_offset = 3.0;
    shift_intervals.push_back(interval);

    const auto offset_trajectory =
      autoware::trajectory::shift(centerline_trajectory.value(), shift_intervals);

    const auto prev_pose = offset_trajectory.restore().at(0).pose;
    const auto current_pose = offset_trajectory.restore().at(1).pose;

    const auto previous_points =
      std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
        offset_trajectory.restore(), prev_pose, sample_num, resolution));

    const auto offset_points =
      std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
        offset_trajectory.restore(), current_pose, sample_num, resolution));

    shift_intervals.clear();
    interval.start = offset_trajectory.length() * 0.2;
    interval.end = offset_trajectory.length() * 0.5;
    interval.lateral_offset = -3.0;
    shift_intervals.push_back(interval);

    const auto returning_to_center_trajectory =
      autoware::trajectory::shift(offset_trajectory, shift_intervals);

    const auto returning_to_center_points =
      std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
        returning_to_center_trajectory.restore(), current_pose, sample_num, resolution));

    add_data(
      std::make_shared<CoreData>(offset_points, previous_points, objects, preferred_lanes, "bad"));
    add_data(std::make_shared<CoreData>(
      returning_to_center_points, previous_points, objects, preferred_lanes, "possible"));

    const auto best = get_best_trajectory();

    EXPECT_NE(best->tag(), "bad");
  }
}

TEST_F(TestTrajectoryRanker, intersection_lane_keep)
{
  const auto preferred_lanes = std::make_shared<lanelet::ConstLanelets>(
    get_preferred_lanes("autoware_test_utils", "lanelet2_map.osm", 9102, 112));

  const auto prev_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(3711.967529296875).y(73718.0).z(19.339))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.00012232430968265882)
                     .y(-0.0005086549380674299)
                     .z(0.23381954091659465)
                     .w(0.972279871535182));
  const auto current_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(3714.1552734375).y(73718.0).z(19.339))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.00012232430968265882)
                     .y(-0.0005086549380674299)
                     .z(0.23381954091659465)
                     .w(0.972279871535182));

  const auto centerline_points = create_centerline_path();
  const auto previous_points =
    std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
      centerline_points, prev_pose, sample_num, resolution));
  const auto current_points =
    std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
      centerline_points, current_pose, sample_num, resolution));

  const auto centerline_trajectory =
    autoware::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>::Builder{}.build(
      centerline_points);

  std::vector<trajectory::ShiftInterval> shift_intervals;

  double length = centerline_trajectory->length();
  for (size_t i = 0; i < 2; i++) {
    trajectory::ShiftInterval interval;
    interval.start = 0.5 * length + 5.0 * static_cast<double>(i);
    interval.end = 0.5 * length + 5.0 * static_cast<double>(i + 1);
    interval.lateral_offset = std::pow(-1.0, i);
    shift_intervals.push_back(interval);
  }
  const auto shifted_trajectory =
    autoware::trajectory::shift(centerline_trajectory.value(), shift_intervals);
  const auto shifted_point =
    std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
      shifted_trajectory.restore(), current_pose, sample_num, resolution));

  std::shared_ptr<PredictedObjects> objects = std::make_shared<PredictedObjects>();

  add_data(std::make_shared<CoreData>(
    current_points, previous_points, objects, preferred_lanes, "possible"));
  add_data(
    std::make_shared<CoreData>(shifted_point, previous_points, objects, preferred_lanes, "bad"));

  const auto best = get_best_trajectory();

  EXPECT_NE(best->tag(), "bad");
}

}  // namespace autoware::trajectory_selector::trajectory_ranker
