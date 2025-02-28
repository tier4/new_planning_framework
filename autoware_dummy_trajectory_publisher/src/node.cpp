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

#include "node.hpp"

#include "autoware/offline_evaluation_tools/utils.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

namespace autoware::trajectory_selector::dummy_trajectory_publisher
{

DummyTrajectoryPublisherNode::DummyTrajectoryPublisherNode(const rclcpp::NodeOptions & node_options)
: Node{"dummy_trajectory_publisher_node", node_options},
  sub_trajectory_{this->create_subscription<InputMsgType>(
    "~/input/trajectory", 1,
    std::bind(&DummyTrajectoryPublisherNode::process, this, std::placeholders::_1))},
  pub_trajectories_{this->create_publisher<OutputMsgType>("~/output/trajectories", 1)},
  listener_{std::make_shared<augment::ParamListener>(get_node_parameters_interface())},
  vehicle_info_{std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())},
  generator_name_{"frenet_planner"},
  generator_uuid_{autoware_utils::generate_uuid()}
{
}

void DummyTrajectoryPublisherNode::process(const InputMsgType::ConstSharedPtr msg)
{
  const auto odometry_ptr = sub_odometry_.take_data();
  if (odometry_ptr == nullptr) {
    return;
  }

  const auto acceleration_ptr = sub_acceleration_.take_data();
  if (acceleration_ptr == nullptr) {
    return;
  }

  const auto & frenet_trajectories = offline_evaluation_tools::utils::augment(
    *msg, odometry_ptr->pose.pose, odometry_ptr->twist.twist.linear.x,
    acceleration_ptr->accel.accel.linear.x, vehicle_info_, parameters());

  std::vector<autoware_new_planning_msgs::msg::Trajectory> new_trajectories;
  for (const auto & frenet_trajectory : frenet_trajectories) {
    const auto new_trajectory =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
        .header(msg->header)
        .generator_id(generator_uuid_)
        .points(frenet_trajectory)
        .score(0.0);
    new_trajectories.push_back(new_trajectory);
  }

  const auto generator_name = std_msgs::build<std_msgs::msg::String>().data(generator_name_);

  const auto generator_info =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo>()
      .generator_id(generator_uuid_)
      .generator_name(generator_name);

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(new_trajectories)
      .generator_info({generator_info});

  pub_trajectories_->publish(output);
}

auto DummyTrajectoryPublisherNode::parameters() const
  -> std::shared_ptr<offline_evaluation_tools::DataAugmentParameters>
{
  const auto node_params = listener_->get_params();

  const auto parameters = std::make_shared<offline_evaluation_tools::DataAugmentParameters>();

  parameters->sample_num = node_params.sample_num;
  parameters->resolution = node_params.resolution;
  parameters->target_state.lat_positions = node_params.target_state.lateral_positions;
  parameters->target_state.lat_velocities = node_params.target_state.lateral_velocities;
  parameters->target_state.lat_accelerations = node_params.target_state.lateral_accelerations;
  parameters->target_state.lon_positions = node_params.target_state.longitudinal_positions;
  parameters->target_state.lon_velocities = node_params.target_state.longitudinal_velocities;
  parameters->target_state.lon_accelerations = node_params.target_state.longitudinal_accelerations;

  return parameters;
}

}  // namespace autoware::trajectory_selector::dummy_trajectory_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::dummy_trajectory_publisher::DummyTrajectoryPublisherNode)
