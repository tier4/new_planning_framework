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

#include "autoware/interpolation/linear_interpolation.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_new_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <boost/geometry/algorithms/detail/within/interface.hpp>

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
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&FeasibleTrajectoryFilterNode::map_callback, this, std::placeholders::_1));
}

void FeasibleTrajectoryFilterNode::process(const Trajectories::ConstSharedPtr msg)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  publish(check_feasibility(msg));
}

void FeasibleTrajectoryFilterNode::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

Trajectories::ConstSharedPtr FeasibleTrajectoryFilterNode::check_feasibility(
  const Trajectories::ConstSharedPtr msg)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto odometry_ptr = std::const_pointer_cast<Odometry>(sub_odometry_.take_data());
  if (odometry_ptr == nullptr) {
    return std::make_shared<Trajectories>();
  }

  auto trajectories = msg->trajectories;

  const auto remove_invalid_trajectories = [&trajectories](auto predicate) {
    auto itr = std::remove_if(trajectories.begin(), trajectories.end(), predicate);
    trajectories.erase(itr, trajectories.end());
  };

  remove_invalid_trajectories([](const auto & trajectory) { return trajectory.points.size() < 2; });

  remove_invalid_trajectories([this, odometry_ptr](const auto & trajectory) {
    return is_trajectory_offtrack(trajectory, odometry_ptr->pose.pose.position);
  });

  if (listener_->get_params().out_of_lane.enable) {
    remove_invalid_trajectories(
      [this](const auto & trajectory) { return out_of_lane(trajectory); });
  }

  const auto new_trajectories = autoware_new_planning_msgs::build<Trajectories>()
                                  .trajectories(trajectories)
                                  .generator_info(msg->generator_info);

  return std::make_shared<Trajectories>(new_trajectories);
}

bool FeasibleTrajectoryFilterNode::is_trajectory_offtrack(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Point & ego_position)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  static constexpr double epsilon = 5.0;

  const auto idx = autoware::motion_utils::findNearestIndex(trajectory.points, ego_position);
  const auto target_position = trajectory.points.at(idx).pose.position;

  return autoware_utils::calc_squared_distance2d(ego_position, target_position) > epsilon;
}

bool FeasibleTrajectoryFilterNode::out_of_lane(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr_) {
    return false;
  }

  std::vector<double> timestamps;
  for (const auto & point : trajectory.points) {
    timestamps.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  if (!interpolation::isIncreasing(timestamps))
    return false;  // Todo (go-sakayori): consider how to deal with trajectory without
                   // time_from_start

  for (const auto & point : trajectory.points) {
    if (
      rclcpp::Duration(point.time_from_start).seconds() > listener_->get_params().out_of_lane.time)
      break;
    const auto nearest_lanelet = lanelet::geometry::findWithin2d(
      lanelet_map_ptr_->laneletLayer,
      lanelet::BasicPoint2d(point.pose.position.x, point.pose.position.y),
      1.0);  // Todo (go-sakayori): remove hard code value
    if (nearest_lanelet.empty()) return true;
  }
  return false;
}

}  // namespace autoware::trajectory_selector::feasible_trajectory_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::feasible_trajectory_filter::FeasibleTrajectoryFilterNode)
