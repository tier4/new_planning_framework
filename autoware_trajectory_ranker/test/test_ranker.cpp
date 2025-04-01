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
#include <autoware/trajectory_selector_common/structs.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_planning_msgs/msg/detail/lanelet_primitive__builder.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>

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
    return points;
  }

  void add_data(const std::shared_ptr<CoreData> & core_data) { node_->evaluator_->add(core_data); }

  void call_evaluator()
  {
    const auto parameters = node_->parameters();
    node_->evaluator_->best(parameters);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<TrajectoryRankerNode> node_;
  size_t sample_num;
  double resolution;
};

TEST_F(TestTrajectoryRanker, straight_line)
{
  const auto preferred_lanes = std::make_shared<lanelet::ConstLanelets>(
    get_preferred_lanes("autoware_test_utils", "2km_test.osm", 9618, 4715));
  const auto prev_pose = autoware::test_utils::createPose(-700.0, 1.45, 0.0, 0.0, 0.0, 0.0);
  const auto current_pose = autoware::test_utils::createPose(-695.0, 1.45, 0.0, 0.0, 0.0, 0.0);

  const auto previous_points =
    std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
      create_centerline_path(), prev_pose, sample_num, resolution));
  const auto current_points =
    std::make_shared<TrajectoryPoints>(autoware::trajectory_selector::utils::sampling(
      create_centerline_path(), current_pose, sample_num, resolution));

  std::shared_ptr<PredictedObjects> objects = std::make_shared<PredictedObjects>();

  add_data(
    std::make_shared<CoreData>(current_points, previous_points, objects, preferred_lanes, "best"));
}
}  // namespace autoware::trajectory_selector::trajectory_ranker
