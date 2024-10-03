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

#include "utils.hpp"

#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_frenet_planner/frenet_planner.hpp"
#include "autoware_path_sampler/prepare_inputs.hpp"
#include "autoware_path_sampler/utils/trajectory_utils.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools::utils
{

auto convertToTrajectoryPoints(
  const autoware::sampler_common::Trajectory & trajectory,
  const std::shared_ptr<VehicleInfo> & vehicle_info, const double z) -> std::vector<TrajectoryPoint>
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
    p.acceleration_mps2 = trajectory.longitudinal_accelerations.at(i);
    p.front_wheel_angle_rad = vehicle_info->wheel_base_m * trajectory.curvatures.at(i);
    traj_points.push_back(p);
  }
  return traj_points;
}

auto prepareSamplingParameters(
  const autoware::sampler_common::Configuration & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline,
  [[maybe_unused]] const double trajectory_length,
  const std::shared_ptr<TargetStateParameters> & parameters)
  -> autoware::frenet_planner::SamplingParameters
{
  autoware::frenet_planner::SamplingParameters sampling_parameters;

  // calculate target lateral positions
  sampling_parameters.resolution = 0.5;
  // const auto max_s = std::max(path_spline.lastS(), 5.0);
  // const auto max_s = path_spline.lastS();
  autoware::frenet_planner::SamplingParameter p;
  p.target_duration = 10.0;
  for (const auto lon_acceleration : parameters->lon_accelerations) {
    p.target_state.longitudinal_acceleration = lon_acceleration;
    p.target_state.longitudinal_velocity =
      initial_state.velocity + lon_acceleration * p.target_duration;
    // p.target_state.position.s = std::min(
    //   max_s, path_spline.frenet(initial_state.pose).s +
    //            std::max(
    //              0.0, initial_state.velocity * p.target_duration +
    //                     0.5 * lon_acceleration * std::pow(p.target_duration, 2.0) -
    //                     base_length));
    p.target_state.position.s =
      path_spline.frenet(initial_state.pose).s + initial_state.velocity * p.target_duration +
      0.5 * lon_acceleration * std::pow(p.target_duration, 2.0) - base_length;
    for (const auto lat_position : parameters->lat_positions) {
      p.target_state.position.d = lat_position;
      for (const auto lat_velocity : parameters->lat_velocities) {
        p.target_state.lateral_velocity = lat_velocity;
        for (const auto lat_acceleration : parameters->lat_accelerations) {
          p.target_state.lateral_acceleration = lat_acceleration;
          sampling_parameters.parameters.push_back(p);
        }
      }
    }
    // if (p.target_state.position.s == max_s) break;
  }
  return sampling_parameters;
}

auto augment(
  const Trajectory & trajectory, const Pose & p_ego, const double v_ego, const double a_ego,
  const std::shared_ptr<VehicleInfo> & vehicle_info,
  const std::shared_ptr<DataAugmentParameters> & parameters)
  -> std::vector<std::vector<TrajectoryPoint>>
{
  const auto reference_trajectory =
    autoware::path_sampler::preparePathSpline(trajectory.points, true);

  autoware::sampler_common::Configuration current_state;
  current_state.pose = {p_ego.position.x, p_ego.position.y};
  current_state.heading = tf2::getYaw(p_ego.orientation);
  current_state.velocity = v_ego;

  current_state.frenet = reference_trajectory.frenet(current_state.pose);
  // current_state.pose = reference_trajectory.cartesian(current_state.frenet.s);
  current_state.heading = reference_trajectory.yaw(current_state.frenet.s);
  current_state.curvature = reference_trajectory.curvature(current_state.frenet.s);

  const auto trajectory_length = autoware::motion_utils::calcArcLength(trajectory.points);
  const auto sampling_parameters = prepareSamplingParameters(
    current_state, 0.0, reference_trajectory, trajectory_length,
    std::make_shared<TargetStateParameters>(parameters->target_state));

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

  std::vector<std::vector<TrajectoryPoint>> output;
  output.reserve(sampling_frenet_trajectories.size());

  for (const auto & trajectory : sampling_frenet_trajectories) {
    output.push_back(convertToTrajectoryPoints(
      trajectory.resampleTimeFromZero(parameters->resolution), vehicle_info, p_ego.position.z));
  }

  return output;
}
}  // namespace autoware::trajectory_selector::offline_evaluation_tools::utils
