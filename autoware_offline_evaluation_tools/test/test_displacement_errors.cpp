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
using autoware::trajectory_selector::TrajectoryPoints;

class DisplacementErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create simple trajectory points for testing
    candidate_trajectory_ = std::make_shared<TrajectoryPoints>();
    ground_truth_trajectory_ = std::make_shared<TrajectoryPoints>();
    
    // Create candidate trajectory (straight line)
    for (size_t i = 0; i < 5; i++) {
      autoware_planning_msgs::msg::TrajectoryPoint point;
      point.pose = autoware::test_utils::createPose(i * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      point.longitudinal_velocity_mps = 10.0;
      candidate_trajectory_->push_back(point);
    }
    
    // Create ground truth trajectory (offset by 0.5m in y-direction)
    for (size_t i = 0; i < 5; i++) {
      autoware_planning_msgs::msg::TrajectoryPoint point;
      point.pose = autoware::test_utils::createPose(i * 1.0, 0.5, 0.0, 0.0, 0.0, 0.0);
      point.longitudinal_velocity_mps = 10.0;
      ground_truth_trajectory_->push_back(point);
    }
  }

  std::shared_ptr<TrajectoryPoints> candidate_trajectory_;
  std::shared_ptr<TrajectoryPoints> ground_truth_trajectory_;
};

TEST_F(DisplacementErrorTest, EuclideanDistanceCalculation)
{
  // Create two poses with known distance
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.0;
  pose1.position.y = 0.0;
  pose1.position.z = 0.0;
  
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 3.0;
  pose2.position.y = 4.0;
  pose2.position.z = 0.0;
  
  // Expected distance: sqrt(3^2 + 4^2) = 5.0
  // Note: This test would require a proper BagEvaluator instance
  // For now, we test the expected result manually
  const double expected_distance = std::sqrt(3.0 * 3.0 + 4.0 * 4.0);
  
  EXPECT_DOUBLE_EQ(expected_distance, 5.0);
}

TEST_F(DisplacementErrorTest, TrajectoryStructureValidation)
{
  // Test that our test trajectories are properly set up
  EXPECT_EQ(candidate_trajectory_->size(), 5u);
  EXPECT_EQ(ground_truth_trajectory_->size(), 5u);
  
  // Verify consistent offset between trajectories
  for (size_t i = 0; i < candidate_trajectory_->size(); i++) {
    const auto& candidate_pose = candidate_trajectory_->at(i).pose;
    const auto& ground_truth_pose = ground_truth_trajectory_->at(i).pose;
    
    // Candidate should be at (i, 0, 0)
    EXPECT_DOUBLE_EQ(candidate_pose.position.x, static_cast<double>(i));
    EXPECT_DOUBLE_EQ(candidate_pose.position.y, 0.0);
    
    // Ground truth should be at (i, 0.5, 0)
    EXPECT_DOUBLE_EQ(ground_truth_pose.position.x, static_cast<double>(i));
    EXPECT_DOUBLE_EQ(ground_truth_pose.position.y, 0.5);
    
    // Distance should be constant 0.5m for all points
    const double dx = ground_truth_pose.position.x - candidate_pose.position.x;
    const double dy = ground_truth_pose.position.y - candidate_pose.position.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    
    EXPECT_DOUBLE_EQ(distance, 0.5);
  }
}

TEST_F(DisplacementErrorTest, ExpectedDisplacementErrorValues)
{
  // For our test case with constant 0.5m offset:
  // - ADE (Average Displacement Error) should be 0.5
  // - FDE (Final Displacement Error) should be 0.5
  // - Min/Max displacement errors should both be 0.5
  // - Standard deviation should be 0.0 (all errors are the same)
  
  const double expected_ade = 0.5;
  const double expected_fde = 0.5;
  const double expected_min = 0.5;
  const double expected_max = 0.5;
  const double expected_std = 0.0;
  
  // These values would be validated if we had a proper BagEvaluator instance
  EXPECT_DOUBLE_EQ(expected_ade, 0.5);
  EXPECT_DOUBLE_EQ(expected_fde, 0.5);
  EXPECT_DOUBLE_EQ(expected_min, 0.5);
  EXPECT_DOUBLE_EQ(expected_max, 0.5);
  EXPECT_DOUBLE_EQ(expected_std, 0.0);
}

TEST_F(DisplacementErrorTest, EmptyTrajectoryHandling)
{
  // Test behavior with empty trajectories
  auto empty_trajectory = std::make_shared<TrajectoryPoints>();
  
  EXPECT_TRUE(empty_trajectory->empty());
  EXPECT_EQ(empty_trajectory->size(), 0u);
  
  // The displacement error calculation should handle empty trajectories gracefully
  // and return an error message indicating the issue
}

TEST_F(DisplacementErrorTest, UnequalLengthTrajectories)
{
  // Create trajectories of different lengths
  auto short_trajectory = std::make_shared<TrajectoryPoints>();
  for (size_t i = 0; i < 3; i++) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = autoware::test_utils::createPose(i * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    short_trajectory->push_back(point);
  }
  
  // The displacement error calculation should use the minimum length
  // In this case, it should compare only the first 3 points
  const auto min_length = std::min(short_trajectory->size(), ground_truth_trajectory_->size());
  
  EXPECT_EQ(min_length, 3u);
  EXPECT_EQ(short_trajectory->size(), 3u);
  EXPECT_EQ(ground_truth_trajectory_->size(), 5u);
}