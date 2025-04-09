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

#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "autoware/trajectory_selector_common/utils.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/detail/lanelet_route__builder.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>

#include <gtest/gtest.h>

#include <cerrno>
#include <cstddef>
#include <optional>

namespace autoware::trajectory_selector::utils
{

TEST(TrajectorySelectorCommonUtils, find_nearest_timestamp)
{
  using utils::find_nearest_timestamp;

  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      points.push_back(autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(rclcpp::Duration(0, 0))
                         .pose(autoware::test_utils::createPose(
                           static_cast<double>(i) * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0));
    }
    const auto time = find_nearest_timestamp(points, rclcpp::Duration(2, 350000000));
    EXPECT_FALSE(time.has_value());
  }

  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      points.push_back(autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(rclcpp::Duration(i / 10, i % 10 * 100000000))
                         .pose(autoware::test_utils::createPose(
                           static_cast<double>(i) * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0));
    }
    const auto time = find_nearest_timestamp(points, rclcpp::Duration(2, 350000000));
    ASSERT_TRUE(time.has_value());
    EXPECT_EQ(time.value(), 23);
  }
}

TEST(TrajectorySelectorCommonUtils, sampling_with_time)
{
  using utils::sampling_with_time;
  static constexpr double epsilon = 1e-06;

  {
    TrajectoryPoints points;
    std::optional<size_t> start_index(1);

    const auto sampled_points = sampling_with_time(points, 20, 0.5, start_index);
    EXPECT_TRUE(sampled_points.empty());
  }

  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      points.push_back(autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(rclcpp::Duration(i / 10, i % 10 * 100000000))
                         .pose(autoware::test_utils::createPose(
                           static_cast<double>(i) * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0));
    }

    std::optional<size_t> start_index(std::nullopt);
    auto sampled_points = sampling_with_time(points, 20, 0.5, start_index);
    EXPECT_TRUE(sampled_points.empty());

    start_index = 101;
    sampled_points = sampling_with_time(points, 20, 0.5, start_index);
    EXPECT_TRUE(sampled_points.empty());

    start_index = 2;
    sampled_points = sampling_with_time(points, 20, 0.5, start_index);
    ASSERT_EQ(sampled_points.size(), 20);

    for (size_t i = 0; i < 20; i++) {
      EXPECT_NEAR(
        sampled_points.at(i).pose.position.x, 0.1 * start_index.value() + 0.5 * i, epsilon);
    }
  }

  {
    TrajectoryPoints points;
    for (size_t i = 0; i < 100; i++) {
      points.push_back(autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(rclcpp::Duration(0, 0))
                         .pose(autoware::test_utils::createPose(
                           static_cast<double>(i) * 0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0));
    }
    std::optional<size_t> start_index(10);
    const auto sampled_points = sampling_with_time(points, 20, 0.5, start_index);
    ASSERT_EQ(sampled_points.size(), 20);

    for (size_t i = 0; i < 20; i++) {
      EXPECT_NEAR(sampled_points.at(i).pose.position.x, points.back().pose.position.x, epsilon);
    }
  }
}
}  // namespace autoware::trajectory_selector::utils
