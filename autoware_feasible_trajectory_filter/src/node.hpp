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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/trajectory_selector_common/interface/node_interface.hpp"
#include "autoware_feasible_trajectory_filter_param.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>

#include <memory>

namespace autoware::trajectory_selector::feasible_trajectory_filter
{

class FeasibleTrajectoryFilterNode : public TrajectoryFilterInterface
{
public:
  explicit FeasibleTrajectoryFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const Trajectories::ConstSharedPtr msg) override;

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  Trajectories::ConstSharedPtr check_feasibility(const Trajectories::ConstSharedPtr msg);

  std::unique_ptr<feasible::ParamListener> listener_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
};

}  // namespace autoware::trajectory_selector::feasible_trajectory_filter

#endif  // NODE_HPP_
