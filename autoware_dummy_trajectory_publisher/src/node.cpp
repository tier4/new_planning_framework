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

#include "autoware_frenet_planner/frenet_planner.hpp"
#include "autoware_path_sampler/prepare_inputs.hpp"
#include "autoware_path_sampler/utils/trajectory_utils.hpp"

#include <autoware/universe_utils/ros/uuid_helper.hpp>

namespace autoware::trajectory_selector::dummy_trajectory_publisher
{

namespace
{
auto convertToTrajectoryPoints(
  const autoware::sampler_common::Trajectory & trajectory,
  const vehicle_info_utils::VehicleInfo & vehicle_info, const double z) -> TrajectoryPoints
{
  std::vector<TrajectoryPoint> traj_points;
  for (auto i = 0UL; i < trajectory.points.size(); ++i) {
    TrajectoryPoint p;
    p.pose.position.x = trajectory.points[i].x();
    p.pose.position.y = trajectory.points[i].y();
    p.pose.position.z = z;
    auto quat = tf2::Quaternion();
    quat.setRPY(0.0, 0.0, trajectory.yaws[i]);
    p.pose.orientation.w = quat.w();
    p.pose.orientation.x = quat.x();
    p.pose.orientation.y = quat.y();
    p.pose.orientation.z = quat.z();
    p.longitudinal_velocity_mps = trajectory.longitudinal_velocities.at(i);
    p.lateral_velocity_mps = trajectory.lateral_velocities.at(i);
    p.front_wheel_angle_rad = vehicle_info.wheel_base_m * trajectory.curvatures.at(i);
    traj_points.push_back(p);
  }
  return traj_points;
}

auto prepareSamplingParameters(
  const autoware::sampler_common::Configuration & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline,
  [[maybe_unused]] const double trajectory_length,
  const parameters::Params & parameters) -> autoware::frenet_planner::SamplingParameters
{
  autoware::frenet_planner::SamplingParameters sampling_parameters;

  // calculate target lateral positions
  sampling_parameters.resolution = 0.5;
  const auto max_s = path_spline.lastS();
  autoware::frenet_planner::SamplingParameter p;
  p.target_duration = 10.0;
  for (const auto lon_acceleration : parameters.frenet.target_state.longitudinal_accelerations) {
    p.target_state.longitudinal_acceleration = lon_acceleration;
    p.target_state.longitudinal_velocity =
      initial_state.velocity + lon_acceleration * p.target_duration;
    p.target_state.position.s = std::min(
      max_s, path_spline.frenet(initial_state.pose).s +
               std::max(
                 0.0, initial_state.velocity * p.target_duration +
                        0.5 * lon_acceleration * std::pow(p.target_duration, 2.0) - base_length));
    for (const auto lat_position : parameters.frenet.target_state.lateral_positions) {
      p.target_state.position.d = lat_position;
      for (const auto lat_velocity : parameters.frenet.target_state.lateral_velocities) {
        p.target_state.lateral_velocity = lat_velocity;
        for (const auto lat_acceleration : parameters.frenet.target_state.lateral_accelerations) {
          p.target_state.lateral_acceleration = lat_acceleration;
          sampling_parameters.parameters.push_back(p);
        }
      }
    }
    if (p.target_state.position.s == max_s) break;
  }
  return sampling_parameters;
}

auto sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const double v_ego, const double a_ego,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const parameters::Params & parameters) -> std::vector<TrajectoryPoints>
{
  const auto reference_trajectory = autoware::path_sampler::preparePathSpline(points, true);

  autoware::sampler_common::Configuration current_state;
  current_state.pose = {p_ego.position.x, p_ego.position.y};
  current_state.heading = tf2::getYaw(p_ego.orientation);
  current_state.velocity = v_ego;

  current_state.frenet = reference_trajectory.frenet(current_state.pose);
  // current_state.pose = reference_trajectory.cartesian(current_state.frenet.s);
  current_state.heading = reference_trajectory.yaw(current_state.frenet.s);
  current_state.curvature = reference_trajectory.curvature(current_state.frenet.s);

  const auto trajectory_length = autoware::motion_utils::calcArcLength(points);
  const auto sampling_parameters = prepareSamplingParameters(
    current_state, 0.0, reference_trajectory, trajectory_length, parameters);

  autoware::frenet_planner::FrenetState initial_frenet_state;
  initial_frenet_state.position = reference_trajectory.frenet(current_state.pose);
  initial_frenet_state.longitudinal_velocity = v_ego;
  initial_frenet_state.longitudinal_acceleration = a_ego;
  const auto s = initial_frenet_state.position.s;
  const auto d = initial_frenet_state.position.d;
  // Calculate Velocity and acceleration parametrized over arc length
  // From appendix I of Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet
  // Frame
  const auto frenet_yaw = current_state.heading - reference_trajectory.yaw(s);
  const auto path_curvature = reference_trajectory.curvature(s);
  const auto delta_s = 0.001;
  initial_frenet_state.lateral_velocity = (1 - path_curvature * d) * std::tan(frenet_yaw);
  const auto path_curvature_deriv =
    (reference_trajectory.curvature(s + delta_s) - path_curvature) / delta_s;
  const auto cos_yaw = std::cos(frenet_yaw);
  if (cos_yaw == 0.0) {
    initial_frenet_state.lateral_acceleration = 0.0;
  } else {
    initial_frenet_state.lateral_acceleration =
      -(path_curvature_deriv * d + path_curvature * initial_frenet_state.lateral_velocity) *
        std::tan(frenet_yaw) +
      ((1 - path_curvature * d) / (cos_yaw * cos_yaw)) *
        (current_state.curvature * ((1 - path_curvature * d) / cos_yaw) - path_curvature);
  }

  const auto sampling_frenet_trajectories = autoware::frenet_planner::generateTrajectories(
    reference_trajectory, initial_frenet_state, sampling_parameters);

  std::vector<TrajectoryPoints> output;
  output.reserve(sampling_frenet_trajectories.size());

  for (const auto & trajectory : sampling_frenet_trajectories) {
    output.push_back(convertToTrajectoryPoints(
      trajectory.resampleTimeFromZero(parameters.time_resolution), vehicle_info, p_ego.position.z));
  }

  return output;
}
}  // namespace

DummyTrajectoryPublisherNode::DummyTrajectoryPublisherNode(const rclcpp::NodeOptions & node_options)
: Node{"dummy_trajectory_publisher_node", node_options},
  sub_trajectory_{this->create_subscription<InputMsgType>(
    "~/input/trajectory", 1,
    std::bind(&DummyTrajectoryPublisherNode::process, this, std::placeholders::_1))},
  pub_trajectories_{this->create_publisher<OutputMsgType>("~/output/trajectories", 1)},
  parameters_{std::make_shared<parameters::ParamListener>(get_node_parameters_interface())},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  generator_name_{"frenet_planner"},
  generator_uuid_{autoware::universe_utils::generateUUID()}
{
}

void DummyTrajectoryPublisherNode::process(const InputMsgType::ConstSharedPtr msg)
{
  const auto odometry_ptr = sub_odometry_.takeData();
  if (odometry_ptr == nullptr) {
    return;
  }

  const auto acceleration_ptr = sub_acceleration_.takeData();
  if (acceleration_ptr == nullptr) {
    return;
  }

  const auto & frenet_trajectories = sampling(
    msg->points, odometry_ptr->pose.pose, odometry_ptr->twist.twist.linear.x,
    acceleration_ptr->accel.accel.linear.x, vehicle_info_, parameters_->get_params());

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

}  // namespace autoware::trajectory_selector::dummy_trajectory_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::dummy_trajectory_publisher::DummyTrajectoryPublisherNode)
