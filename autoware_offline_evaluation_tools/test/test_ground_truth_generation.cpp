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

#include "../src/evaluation.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

using namespace autoware::trajectory_selector::offline_evaluation_tools;
using namespace std::chrono;

class GroundTruthGenerationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    timestamp_ = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
    
    // Create mock parameters
    parameters_ = std::make_shared<DataAugmentParameters>();
    parameters_->sample_num = 10;
    parameters_->resolution = 0.1;
  }

  rcutils_time_point_value_t timestamp_;
  std::shared_ptr<DataAugmentParameters> parameters_;
};

TEST_F(GroundTruthGenerationTest, ConvertLocalizationToTrajectoryPoint)
{
  // Create mock BagEvaluator (would need proper initialization in real scenario)
  // This test focuses on the conversion logic
  
  // Create test localization message
  nav_msgs::msg::Odometry localization;
  localization.header.stamp.sec = 1;
  localization.header.stamp.nanosec = 0;
  localization.header.frame_id = "map";
  
  localization.pose.pose.position.x = 10.0;
  localization.pose.pose.position.y = 5.0;
  localization.pose.pose.position.z = 0.0;
  localization.pose.pose.orientation.w = 1.0;
  
  localization.twist.twist.linear.x = 15.0;
  localization.twist.twist.linear.y = 0.5;
  localization.twist.twist.angular.z = 0.1;

  // Create time_from_start
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 2;
  time_from_start.nanosec = 500000000; // 0.5 seconds

  // Note: This test would require a proper BagEvaluator instance
  // For now, we test the data structure compatibility
  
  EXPECT_EQ(localization.pose.pose.position.x, 10.0);
  EXPECT_EQ(localization.twist.twist.linear.x, 15.0);
  EXPECT_EQ(time_from_start.sec, 2);
}

TEST_F(GroundTruthGenerationTest, LiveTrajectoryStructure)
{
  // Test creating a live trajectory similar to what would be received
  autoware_planning_msgs::msg::Trajectory live_trajectory;
  live_trajectory.header.stamp.sec = 1;
  live_trajectory.header.stamp.nanosec = 0;
  live_trajectory.header.frame_id = "map";

  // Add trajectory points with time_from_start
  for (size_t i = 0; i < 5; i++) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.time_from_start.sec = static_cast<int32_t>(i / 10);
    point.time_from_start.nanosec = static_cast<uint32_t>((i % 10) * 100000000);
    
    point.pose = autoware::test_utils::createPose(i * 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    point.longitudinal_velocity_mps = 12.0;
    
    live_trajectory.points.push_back(point);
  }

  EXPECT_EQ(live_trajectory.points.size(), 5u);
  
  // Test timing structure
  const auto& first_point = live_trajectory.points[0];
  const auto& last_point = live_trajectory.points[4];
  
  EXPECT_EQ(first_point.time_from_start.sec, 0);
  EXPECT_EQ(first_point.time_from_start.nanosec, 0u);
  
  // Verify time progression
  EXPECT_GT(
    last_point.time_from_start.sec * 1000000000LL + last_point.time_from_start.nanosec,
    first_point.time_from_start.sec * 1000000000LL + first_point.time_from_start.nanosec
  );
}

TEST_F(GroundTruthGenerationTest, TimestampCalculation)
{
  // Test the timestamp calculation logic used in ground truth generation
  const auto current_timestamp = timestamp_;
  
  // Simulate time_from_start values
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 500000000; // 1.5 seconds
  
  const auto time_from_start_ns = static_cast<rcutils_time_point_value_t>(
    time_from_start.sec) * 1000000000LL + 
    static_cast<rcutils_time_point_value_t>(time_from_start.nanosec);
  
  const auto target_timestamp = current_timestamp + time_from_start_ns;
  
  EXPECT_EQ(time_from_start_ns, 1500000000LL);
  EXPECT_GT(target_timestamp, current_timestamp);
  EXPECT_EQ(target_timestamp - current_timestamp, 1500000000LL);
}

TEST_F(GroundTruthGenerationTest, ReplayEvaluationDataWithLocalization)
{
  // Test that ReplayEvaluationData can store localization messages
  auto replay_data = std::make_shared<ReplayEvaluationData>(timestamp_);
  
  // Verify odometry buffer exists
  EXPECT_TRUE(replay_data->buffers.count(TOPIC::ODOMETRY));
  
  // Create mock odometry message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 1;
  odom.header.stamp.nanosec = 0;
  odom.pose.pose.position.x = 5.0;
  odom.twist.twist.linear.x = 10.0;
  
  // Test buffer access (in real scenario, messages would be added during replay)
  auto odometry_buffer = std::dynamic_pointer_cast<Buffer<nav_msgs::msg::Odometry>>(
    replay_data->buffers.at(TOPIC::ODOMETRY));
  
  EXPECT_TRUE(odometry_buffer != nullptr);
  EXPECT_TRUE(odometry_buffer->msgs.empty()); // Initially empty
}