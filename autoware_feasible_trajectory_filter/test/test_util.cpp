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

#include "../src/utils.hpp"

#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <builtin_interfaces/msg/detail/duration__builder.hpp>
#include <builtin_interfaces/msg/detail/duration__struct.hpp>
#include <builtin_interfaces/msg/detail/time__struct.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::trajectory_selector::feasible_trajectory_filter
{
TEST(FeasibleTrajectoryFilterUtilsTest, is_invalid_trajectory)
{
  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 1; i++) {
      const auto time =
        builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec(i).nanosec(0);
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             0.5 * static_cast<double>(i), 10.0, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(0.5)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }
    EXPECT_TRUE(utils::is_invalid_trajectory(points));
  }
  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 4; i++) {
      const auto time =
        builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec(i).nanosec(0);
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             0.5 * static_cast<double>(i), 10.0, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(0.5)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }
    EXPECT_TRUE(utils::is_invalid_trajectory(points));
  }
  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 20; i++) {
      const auto time =
        builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec(i).nanosec(0);
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             0.5 * static_cast<double>(i), 10.0, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(0.5)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }
    EXPECT_FALSE(utils::is_invalid_trajectory(points));
    points.begin()->acceleration_mps2 = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(utils::is_invalid_trajectory(points));

    points.begin()->acceleration_mps2 = 0.0;
    EXPECT_FALSE(utils::is_invalid_trajectory(points));  // Just in case
    points.at(10).time_from_start.sec = 5;
    EXPECT_TRUE(utils::is_invalid_trajectory(points));
  }
}

TEST(FeasibleTrajectoryFilterUtilsTest, is_trajectory_offtrack)
{
  {
    const auto ego_point = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(0.0);

    TrajectoryPoints points;
    for (size_t i = 0; i < 10; i++) {
      const auto time =
        builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec(i).nanosec(0);
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             0.5 * static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(0.5)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }

    EXPECT_FALSE(utils::is_trajectory_offtrack(points, ego_point));
  }
  {
    const auto ego_point = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(0.0);

    TrajectoryPoints points;
    for (size_t i = 0; i < 10; i++) {
      const auto time =
        builtin_interfaces::build<builtin_interfaces::msg::Duration>().sec(i).nanosec(0);
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             0.5 * static_cast<double>(i), 10.0, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(0.5)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }

    EXPECT_TRUE(utils::is_trajectory_offtrack(points, ego_point));
  }
}

TEST(FeasibleTrajectoryFilterUtilsTest, out_of_lane)
{
  const auto map_path =
    autoware::test_utils::get_absolute_path_to_lanelet_map("autoware_test_utils", "2km_test.osm");
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(map_path, 0.1);

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_bin_msg, lanelet_map_ptr);

  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      const auto time = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                          .sec(static_cast<int32_t>(i / 10))
                          .nanosec(static_cast<uint32_t>((i % 10) * 100000000));
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             static_cast<double>(i) * 0.1, 1.75, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(1.0)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }

    EXPECT_FALSE(utils::is_out_of_lane(points, lanelet_map_ptr, 2.0));
    EXPECT_FALSE(utils::is_out_of_lane(points, lanelet_map_ptr, 12.0));
  }
  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      const auto time = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                          .sec(static_cast<int32_t>(i / 10))
                          .nanosec(static_cast<uint32_t>((i % 10) * 100000000));
      const auto point =
        autoware_planning_msgs::build<TrajectoryPoint>()
          .time_from_start(time)
          .pose(autoware::test_utils::createPose(
            static_cast<double>(i) * 0.1, 1.75 + 3.0 * sin(static_cast<double>(i) / 180 * M_PI),
            0.0, 0.0, 0.0, 0.0))
          .longitudinal_velocity_mps(0.5)
          .lateral_velocity_mps(0.0)
          .acceleration_mps2(0.0)
          .heading_rate_rps(0.0)
          .front_wheel_angle_rad(0.0)
          .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }

    EXPECT_FALSE(utils::is_out_of_lane(points, lanelet_map_ptr, 2.0));
    EXPECT_FALSE(utils::is_out_of_lane(points, lanelet_map_ptr, 3.5));
    EXPECT_TRUE(utils::is_out_of_lane(points, lanelet_map_ptr, 3.6));
    EXPECT_TRUE(utils::is_out_of_lane(points, lanelet_map_ptr, 5.0));
  }
}

TEST(FeasibleTrajectoryFilterUtilsTest, has_collision_risk)
{
  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      const auto time = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                          .sec(static_cast<int32_t>(i / 10))
                          .nanosec(static_cast<uint32_t>((i % 10) * 100000000));
      const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                           .time_from_start(time)
                           .pose(autoware::test_utils::createPose(
                             static_cast<double>(i) * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                           .longitudinal_velocity_mps(0.5)
                           .lateral_velocity_mps(0.0)
                           .acceleration_mps2(0.0)
                           .heading_rate_rps(0.0)
                           .front_wheel_angle_rad(0.0)
                           .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }
  }
}
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter
