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

#include "utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <vector>

namespace autoware::trajectory_selector::feasible_trajectory_filter
{

FeasibleTrajectoryFilterNode::FeasibleTrajectoryFilterNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"feasible_trajectory_filter_node", node_options},
  listener_{std::make_unique<feasible::ParamListener>(get_node_parameters_interface())}
{
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  pub_velocity_limit_ = create_publisher<VelocityLimit>("~/output/current_velocity_limit_mps", 1);

  // publish default max velocity
  VelocityLimit max_vel_msg{};
  max_vel_msg.stamp = this->now();
  max_vel_msg.max_velocity = listener_->get_params().max_velocity;
  pub_velocity_limit_->publish(max_vel_msg);

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&FeasibleTrajectoryFilterNode::map_callback, this, std::placeholders::_1));
}

void FeasibleTrajectoryFilterNode::process(const Trajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  publish(check_feasibility(msg));
}

void FeasibleTrajectoryFilterNode::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
}

Trajectories::ConstSharedPtr FeasibleTrajectoryFilterNode::check_feasibility(
  const Trajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto odometry_ptr = std::const_pointer_cast<Odometry>(sub_odometry_.take_data());
  if (odometry_ptr == nullptr) {
    return std::make_shared<Trajectories>();
  }

  const auto objects_ptr = std::const_pointer_cast<PredictedObjects>(sub_objects_.take_data());

  auto trajectories = msg->trajectories;

  const auto remove_invalid_trajectories = [&trajectories](auto predicate) {
    auto itr = std::remove_if(trajectories.begin(), trajectories.end(), predicate);
    trajectories.erase(itr, trajectories.end());
  };

  remove_invalid_trajectories(
    [](const auto & trajectory) { return utils::is_invalid_trajectory(trajectory.points); });

  remove_invalid_trajectories([odometry_ptr](const auto & trajectory) {
    return utils::is_trajectory_offtrack(trajectory.points, odometry_ptr->pose.pose.position);
  });

  if (listener_->get_params().out_of_lane.enable) {
    remove_invalid_trajectories([this](const auto & trajectory) {
      return utils::is_out_of_lane(
        trajectory.points, lanelet_map_ptr_, listener_->get_params().out_of_lane.time);
    });
  }

  if (listener_->get_params().collision.enable && objects_ptr) {
    remove_invalid_trajectories([this, objects_ptr](const auto & trajectory) {
      return utils::has_collision_risk(
        trajectory.points, *objects_ptr, listener_->get_params().collision.time);
    });
  }

  const auto new_trajectories = autoware_new_planning_msgs::build<Trajectories>()
                                  .trajectories(trajectories)
                                  .generator_info(msg->generator_info);

  return std::make_shared<Trajectories>(new_trajectories);
}
}  // namespace autoware::trajectory_selector::feasible_trajectory_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::feasible_trajectory_filter::FeasibleTrajectoryFilterNode)
