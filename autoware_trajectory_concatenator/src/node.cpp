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

#include "autoware/trajectory_selector_common/utils.hpp"
#include "structs.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <builtin_interfaces/msg/detail/duration__builder.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include "autoware_new_planning_msgs/msg/trajectory.hpp"
#include "autoware_new_planning_msgs/msg/trajectory_generator_info.hpp"
#include <autoware_new_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::trajectory_concatenator
{

TrajectoryConcatenatorNode::TrajectoryConcatenatorNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_concatenator_node", node_options},
  timer_{rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrajectoryConcatenatorNode::publish, this))},
  subs_trajectories_{this->create_subscription<Trajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryConcatenatorNode::on_trajectories, this, std::placeholders::_1))},
  sub_selected_trajectory_{this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/selected_trajectory", 1,
    std::bind(&TrajectoryConcatenatorNode::on_selected_trajectory, this, std::placeholders::_1))},
  pub_trajectores_{this->create_publisher<Trajectories>("~/output/trajectories", 1)},
  listener_{std::make_unique<concatenator::ParamListener>(get_node_parameters_interface())}
{
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_concatenator", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
}

void TrajectoryConcatenatorNode::on_trajectories(const Trajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & generator_info : msg->generator_info) {
    const auto uuid = autoware_utils_uuid::to_hex_string(generator_info.generator_id);

    auto trajectories = msg->trajectories;
    const auto itr = std::remove_if(
      trajectories.begin(), trajectories.end(),
      [&generator_info](const auto & t) { return t.generator_id != generator_info.generator_id; });
    trajectories.erase(itr, trajectories.end());

    const auto pre_combine =
      autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
        .trajectories(trajectories)
        .generator_info({generator_info});

    if (buffer_.count(uuid) == 0) {
      buffer_.emplace(uuid, std::make_shared<Trajectories>(pre_combine));
    } else {
      buffer_.at(uuid) = std::make_shared<Trajectories>(pre_combine);
    }
  }
}

void TrajectoryConcatenatorNode::on_selected_trajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!parameters()->use_feedback) return;

  const auto odometry_ptr = std::const_pointer_cast<Odometry>(sub_odometry_.take_data());
  if (odometry_ptr == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const auto uuid = autoware_utils_uuid::generate_default_uuid();
  const auto hex_uuid = autoware_utils_uuid::to_hex_string(uuid);
  const auto ego_seg_idx =
    autoware::motion_utils::findNearestIndex(msg->points, odometry_ptr->pose.pose, 10.0, M_PI_2);
  if (!ego_seg_idx.has_value()) {
    return;
  }

  const auto end = msg->points.back().time_from_start;
  const auto start = msg->points.at(ego_seg_idx.value()).time_from_start;

  auto trajectory_time_duration =
    (rclcpp::Duration(end.sec, end.nanosec) - rclcpp::Duration(start.sec, start.nanosec)).seconds();

  TrajectoryPoints trajectory_points(msg->points.begin() + ego_seg_idx.value(), msg->points.end());

  while (trajectory_time_duration < parameters()->min_end_time) {
    trajectory_points.push_back(autoware::trajectory_selector::utils::calc_extended_point(
      trajectory_points.back(), parameters()->extension_interval));
    trajectory_time_duration += parameters()->extension_interval;
  }

  const auto new_trajectory =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectory>()
      .header(
        std_msgs::build<Header>()
          .stamp(this->now())
          .frame_id(msg->header.frame_id))  // To-do(go-sakayori): consider if msg-> header could be
                                            // used directly. Seems to exceed expiration_time
      .generator_id(uuid)
      .points(trajectory_points)
      .score(0.0);

  const auto generator_info =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo>()
      .generator_id(uuid)
      .generator_name(std_msgs::msg::String().set__data("SelectedTrajectory"));

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories({new_trajectory})
      .generator_info({generator_info});

  if (buffer_.count(hex_uuid) == 0) {
    buffer_.emplace(hex_uuid, std::make_shared<Trajectories>(output));
  } else {
    buffer_.at(hex_uuid) = std::make_shared<Trajectories>(output);
  }
}

void TrajectoryConcatenatorNode::publish()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<Trajectory> trajectories{};
  std::vector<TrajectoryGeneratorInfo> generator_info{};

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.empty()) return;

    const auto current_time = this->now();
    const rclcpp::Duration expiration_time(
      std::chrono::nanoseconds(static_cast<int64_t>(parameters()->duration_time * 1e9)));

    for (auto it = buffer_.begin(); it != buffer_.end();) {
      const auto & [uuid, pre_combine] = *it;
      if (!pre_combine->trajectories.empty()) {
        const auto elapsed_time = (current_time - pre_combine->trajectories.begin()->header.stamp);
        if (elapsed_time > expiration_time) {
          it = buffer_.erase(it);
          continue;
        }
      }
      it++;
    }

    for (const auto & [uuid, pre_combine] : buffer_) {
      trajectories.insert(
        trajectories.end(), pre_combine->trajectories.begin(), pre_combine->trajectories.end());
      generator_info.insert(
        generator_info.end(), pre_combine->generator_info.begin(),
        pre_combine->generator_info.end());
    }
  }

  const auto output =
    autoware_new_planning_msgs::build<autoware_new_planning_msgs::msg::Trajectories>()
      .trajectories(trajectories)
      .generator_info(generator_info);

  pub_trajectores_->publish(output);
}

auto TrajectoryConcatenatorNode::parameters() const -> std::shared_ptr<ConcatenatorParam>
{
  const auto node_params = listener_->get_params();
  const auto parameters = std::make_shared<ConcatenatorParam>();

  parameters->duration_time = node_params.duration_time;
  parameters->use_feedback = node_params.selected_trajectory.use;
  parameters->min_end_time = node_params.selected_trajectory.endpoint_time_min;
  parameters->extension_interval = node_params.selected_trajectory.extension_interval;

  return parameters;
}

}  // namespace autoware::trajectory_selector::trajectory_concatenator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_concatenator::TrajectoryConcatenatorNode)
