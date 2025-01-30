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

#include "autoware/trajectory_selector_common/evaluation.hpp"
#include "autoware/trajectory_selector_common/interface/node_interface.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware_trajectory_ranker_param.hpp"

#include "autoware_new_planning_msgs/msg/trajectory.hpp"
#include "autoware_new_planning_msgs/msg/evaluation_info.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_selector::trajectory_ranker
{

using autoware_new_planning_msgs::msg::Trajectory;
using autoware_new_planning_msgs::msg::EvaluationInfo;

class TrajectoryRankerNode : public TrajectoryFilterInterface
{
public:
  explicit TrajectoryRankerNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const Trajectories::ConstSharedPtr msg) override;

  auto score(const Trajectories::ConstSharedPtr msg) -> Trajectories::ConstSharedPtr;

  auto parameters() const -> std::shared_ptr<EvaluatorParameters>;

  autoware::universe_utils::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};

  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<EvaluationInfo>::SharedPtr pub_score_debug_;

  std::shared_ptr<Evaluator> evaluator_;

  std::unique_ptr<evaluation::ParamListener> listener_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<TrajectoryPoints> previous_points_;
};

}  // namespace autoware::trajectory_selector::trajectory_ranker

#endif  // NODE_HPP_
