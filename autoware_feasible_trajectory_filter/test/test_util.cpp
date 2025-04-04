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

#include <cstdint>
#include <vector>

namespace autoware::trajectory_selector::feasible_trajectory_filter
{
TEST(FeasibleTrajectoryFilterUtilsTest, is_trajectory_offtrack)
{
  const auto time = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
  const auto header = std_msgs::build<std_msgs::msg::Header>().stamp(time).frame_id("map");
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

    const auto trajectory =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
        .header(header)
        .generator_id(autoware_utils_uuid::generate_uuid())
        .points(points)
        .score(0.0);

    EXPECT_FALSE(utils::is_trajectory_offtrack(trajectory, ego_point));
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

    const auto trajectory =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
        .header(header)
        .generator_id(autoware_utils_uuid::generate_uuid())
        .points(points)
        .score(0.0);

    EXPECT_TRUE(utils::is_trajectory_offtrack(trajectory, ego_point));
  }
}

TEST(FeasibleTrajectoryFilterUtilsTest, out_of_lane)
{
  const auto map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
    "autoware_test_utils", "lanelet2_map.osm");
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(map_path, 0.1);

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_bin_msg, lanelet_map_ptr);

  const auto time = builtin_interfaces::build<builtin_interfaces::msg::Time>().sec(0).nanosec(0);
  const auto header = std_msgs::build<std_msgs::msg::Header>().stamp(time).frame_id("map");

  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      const auto time = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                          .sec(static_cast<int32_t>(i / 10))
                          .nanosec(static_cast<uint32_t>((i % 10) * 100000000));
      const auto point =
        autoware_planning_msgs::build<TrajectoryPoint>()
          .time_from_start(time)
          .pose(autoware::test_utils::createPose(static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0))
          .longitudinal_velocity_mps(0.5)
          .lateral_velocity_mps(0.0)
          .acceleration_mps2(0.0)
          .heading_rate_rps(0.0)
          .front_wheel_angle_rad(0.0)
          .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }

    const auto trajectory =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
        .header(header)
        .generator_id(autoware_utils_uuid::generate_uuid())
        .points(points)
        .score(0.0);

    EXPECT_TRUE(utils::is_out_of_lane(trajectory, lanelet_map_ptr, 2.0));
    EXPECT_TRUE(utils::is_out_of_lane(trajectory, lanelet_map_ptr, 12.0));
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
          .pose(autoware::test_utils::createPose(static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0))
          .longitudinal_velocity_mps(0.5)
          .lateral_velocity_mps(0.0)
          .acceleration_mps2(0.0)
          .heading_rate_rps(0.0)
          .front_wheel_angle_rad(0.0)
          .rear_wheel_angle_rad(0.0);
      points.push_back(point);
    }

    const auto trajectory =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
        .header(header)
        .generator_id(autoware_utils_uuid::generate_uuid())
        .points(points)
        .score(0.0);

    EXPECT_TRUE(utils::is_out_of_lane(trajectory, lanelet_map_ptr, 2.0));
    EXPECT_TRUE(utils::is_out_of_lane(trajectory, lanelet_map_ptr, 12.0));
  }
}
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter
