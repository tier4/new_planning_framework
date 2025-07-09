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

#include "../src/bag_handler.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

using namespace autoware::trajectory_selector::offline_evaluation_tools;
using namespace std::chrono;

class BagHandlerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    timestamp_ = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
  }

  rcutils_time_point_value_t timestamp_;
};

TEST_F(BagHandlerTest, BagDataConstruction)
{
  auto bag_data = std::make_shared<BagData>(timestamp_);

  EXPECT_EQ(bag_data->timestamp, timestamp_);
  EXPECT_EQ(bag_data->buffers.size(), 6u);
  EXPECT_TRUE(bag_data->buffers.count(TOPIC::TF));
  EXPECT_TRUE(bag_data->buffers.count(TOPIC::ODOMETRY));
  EXPECT_TRUE(bag_data->buffers.count(TOPIC::ACCELERATION));
  EXPECT_TRUE(bag_data->buffers.count(TOPIC::TRAJECTORY));
  EXPECT_TRUE(bag_data->buffers.count(TOPIC::OBJECTS));
  EXPECT_TRUE(bag_data->buffers.count(TOPIC::STEERING));
}

TEST_F(BagHandlerTest, ReplayEvaluationDataConstruction)
{
  auto replay_data = std::make_shared<ReplayEvaluationData>(timestamp_);

  EXPECT_EQ(replay_data->timestamp, timestamp_);
  EXPECT_EQ(replay_data->buffers.size(), 6u);
  EXPECT_TRUE(replay_data->live_trajectory_buffer != nullptr);
}

TEST_F(BagHandlerTest, LiveTrajectoryBuffering)
{
  auto replay_data = std::make_shared<ReplayEvaluationData>(timestamp_);

  // Create test trajectory
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp.sec = 1;
  trajectory.header.stamp.nanosec = 0;
  trajectory.header.frame_id = "map";

  // Add test trajectory points
  for (size_t i = 0; i < 5; i++) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = autoware::test_utils::createPose(i * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    point.longitudinal_velocity_mps = 10.0;
    trajectory.points.push_back(point);
  }

  // Test appending live trajectory
  replay_data->append_live_trajectory(trajectory);

  // Test retrieval
  const auto retrieved_trajectory = replay_data->get_live_trajectory(timestamp_);
  EXPECT_TRUE(retrieved_trajectory != nullptr);
  EXPECT_EQ(retrieved_trajectory->points.size(), 5u);
  EXPECT_EQ(retrieved_trajectory->header.frame_id, "map");
}

TEST_F(BagHandlerTest, BufferUpdate)
{
  auto bag_data = std::make_shared<BagData>(timestamp_);

  const auto dt = 1000000000; // 1 second in nanoseconds
  bag_data->update(dt);

  EXPECT_EQ(bag_data->timestamp, timestamp_ + dt);
}

TEST_F(BagHandlerTest, BufferReadiness)
{
  auto bag_data = std::make_shared<BagData>(timestamp_);

  // Initially should not be ready (empty buffers)
  EXPECT_FALSE(bag_data->ready());

  auto replay_data = std::make_shared<ReplayEvaluationData>(timestamp_);

  // Live trajectory buffer should not be ready initially
  EXPECT_FALSE(replay_data->live_trajectory_ready());
}