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

#include "../src/utils.cpp"  // NOLINT
#include "autoware/universe_utils/math/unit_conversion.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::trajectory_selector::trajectory_metrics
{

using autoware::universe_utils::createPoint;
using autoware::universe_utils::createQuaternionFromRPY;
using autoware::universe_utils::createVector3;
using autoware::universe_utils::deg2rad;

TEST(MetricsUtilsTest, get_velocity_in_world_coordinate)
{
  {
    const auto pose = geometry_msgs::build<Pose>()
                        .position(createPoint(1.0, 1.0, 0.0))
                        .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(90)));
    const auto velocity = createVector3(1.0, 0.0, 0.0);

    const auto output = utils::internal::get_velocity_in_world_coordinate(pose, velocity);
    EXPECT_NEAR(output.getX(), 0.0, 1e-6);
    EXPECT_NEAR(output.getY(), 1.0, 1e-6);
    EXPECT_NEAR(output.getZ(), 0.0, 1e-6);
  }

  {
    const auto pose = geometry_msgs::build<Pose>()
                        .position(createPoint(1.0, 2.0, 0.0))
                        .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(60)));
    const auto velocity = createVector3(0.866025403784438647, 0.5, 0.0);

    const auto output = utils::internal::get_velocity_in_world_coordinate(pose, velocity);
    EXPECT_NEAR(output.getX(), 0.0, 1e-6);
    EXPECT_NEAR(output.getY(), 1.0, 1e-6);
    EXPECT_NEAR(output.getZ(), 0.0, 1e-6);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto pose = geometry_msgs::build<Pose>()
                        .position(createPoint(1.0, 1.0, 0.0))
                        .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(90)));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(pose)
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto output = utils::internal::get_velocity_in_world_coordinate(point);
    EXPECT_NEAR(output.getX(), 0.0, 1e-6);
    EXPECT_NEAR(output.getY(), 1.0, 1e-6);
    EXPECT_NEAR(output.getZ(), 0.0, 1e-6);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto pose = geometry_msgs::build<Pose>()
                        .position(createPoint(1.0, 2.0, 0.0))
                        .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(60)));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(pose)
                         .longitudinal_velocity_mps(0.866025403784438647)
                         .lateral_velocity_mps(0.5)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto output = utils::internal::get_velocity_in_world_coordinate(point);
    EXPECT_NEAR(output.getX(), 0.0, 1e-6);
    EXPECT_NEAR(output.getY(), 1.0, 1e-6);
    EXPECT_NEAR(output.getZ(), 0.0, 1e-6);
  }
}

TEST(MetricsUtilsTest, time_to_collision)
{
  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto ego_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(0.0, 0.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, 0.0));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(ego_pose)
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto points = std::make_shared<TrajectoryPoints>();
    points->push_back(point);

    const auto objects = std::make_shared<PredictedObjects>();

    const auto output = utils::time_to_collision(points, objects, 0L);
    EXPECT_EQ(output, 10000.0);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto ego_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(0.0, 0.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, 0.0));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(ego_pose)
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto points = std::make_shared<TrajectoryPoints>();
    points->push_back(point);

    const auto obj_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(1.0, 1.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(-90)));

    const auto time_step = builtin_interfaces::build<Duration>().sec(0.5).nanosec(0.0);
    const auto predicted_path = autoware_perception_msgs::build<PredictedPath>()
                                  .path({obj_pose})
                                  .time_step(time_step)
                                  .confidence(1.0);

    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose = obj_pose;
    object.kinematics.initial_twist_with_covariance.twist.linear = createVector3(1.0, 0.0, 0.0);
    object.kinematics.predicted_paths.push_back(predicted_path);

    const auto objects = std::make_shared<PredictedObjects>();
    objects->objects.push_back(object);

    const auto output = utils::time_to_collision(points, objects, 0L);
    EXPECT_DOUBLE_EQ(output, 1.0);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto ego_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(2.0, 1.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(60)));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(ego_pose)
                         .longitudinal_velocity_mps(0.866025403784438647)
                         .lateral_velocity_mps(0.5)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto points = std::make_shared<TrajectoryPoints>();
    points->push_back(point);

    const auto obj_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(1.0, 2.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(-90)));

    const auto time_step = builtin_interfaces::build<Duration>().sec(0.5).nanosec(0.0);
    const auto predicted_path = autoware_perception_msgs::build<PredictedPath>()
                                  .path({obj_pose})
                                  .time_step(time_step)
                                  .confidence(1.0);

    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose = obj_pose;
    object.kinematics.initial_twist_with_covariance.twist.linear = createVector3(1.0, 0.0, 0.0);
    object.kinematics.predicted_paths.push_back(predicted_path);

    const auto objects = std::make_shared<PredictedObjects>();
    objects->objects.push_back(object);

    const auto output = utils::time_to_collision(points, objects, 0L);
    EXPECT_NEAR(output, 1.0, 1e-6);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto ego_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(2.0, 1.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(90)));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(ego_pose)
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto points = std::make_shared<TrajectoryPoints>();
    points->push_back(point);

    const auto obj_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(1.0, 1.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(-90)));

    const auto time_step = builtin_interfaces::build<Duration>().sec(0.5).nanosec(0.0);
    const auto predicted_path = autoware_perception_msgs::build<PredictedPath>()
                                  .path({obj_pose})
                                  .time_step(time_step)
                                  .confidence(1.0);

    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose = obj_pose;
    object.kinematics.initial_twist_with_covariance.twist.linear = createVector3(1.0, 0.0, 0.0);
    object.kinematics.predicted_paths.push_back(predicted_path);

    const auto objects = std::make_shared<PredictedObjects>();
    objects->objects.push_back(object);

    const auto output = utils::time_to_collision(points, objects, 0L);
    EXPECT_DOUBLE_EQ(output, 10000.0);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto ego_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(0.0, 0.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, 0.0));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(ego_pose)
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto points = std::make_shared<TrajectoryPoints>();
    points->push_back(point);

    const auto obj_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(0.0, 1.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(225)));

    const auto time_step = builtin_interfaces::build<Duration>().sec(0.5).nanosec(0.0);
    const auto predicted_path = autoware_perception_msgs::build<PredictedPath>()
                                  .path({obj_pose})
                                  .time_step(time_step)
                                  .confidence(1.0);

    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose = obj_pose;
    object.kinematics.initial_twist_with_covariance.twist.linear =
      createVector3(1.41421356237309505, 0.0, 0.0);
    object.kinematics.predicted_paths.push_back(predicted_path);

    const auto objects = std::make_shared<PredictedObjects>();
    objects->objects.push_back(object);

    const auto output = utils::time_to_collision(points, objects, 0L);
    EXPECT_DOUBLE_EQ(output, 1.0);
  }

  {
    const auto duration = builtin_interfaces::build<Duration>().sec(0.0).nanosec(0.0);
    const auto ego_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(0.0, 0.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, 0.0));
    const auto point = autoware_planning_msgs::build<TrajectoryPoint>()
                         .time_from_start(duration)
                         .pose(ego_pose)
                         .longitudinal_velocity_mps(1.0)
                         .lateral_velocity_mps(0.0)
                         .acceleration_mps2(0.0)
                         .heading_rate_rps(0.0)
                         .front_wheel_angle_rad(0.0)
                         .rear_wheel_angle_rad(0.0);

    const auto points = std::make_shared<TrajectoryPoints>();
    points->push_back(point);

    const auto obj_pose = geometry_msgs::build<Pose>()
                            .position(createPoint(0.0, 1.0, 0.0))
                            .orientation(createQuaternionFromRPY(0.0, 0.0, deg2rad(135)));

    const auto time_step = builtin_interfaces::build<Duration>().sec(0.5).nanosec(0.0);
    const auto predicted_path = autoware_perception_msgs::build<PredictedPath>()
                                  .path({obj_pose})
                                  .time_step(time_step)
                                  .confidence(1.0);

    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose = obj_pose;
    object.kinematics.initial_twist_with_covariance.twist.linear =
      createVector3(1.41421356237309505, 0.0, 0.0);
    object.kinematics.predicted_paths.push_back(predicted_path);

    const auto objects = std::make_shared<PredictedObjects>();
    objects->objects.push_back(object);

    const auto output = utils::time_to_collision(points, objects, 0L);
    EXPECT_DOUBLE_EQ(output, 10000.0);
  }
}

}  // namespace autoware::trajectory_selector::trajectory_metrics
