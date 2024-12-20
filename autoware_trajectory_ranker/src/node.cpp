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

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>

#include <algorithm>
#include <limits>
#include <string>

namespace autoware::trajectory_selector::trajectory_ranker
{

auto generator_name(const UUID & uuid, const std::vector<TrajectoryGeneratorInfo> & generator_info)
  -> std::string
{
  const auto generator_itr = std::find_if(
    generator_info.begin(), generator_info.end(),
    [&uuid](const auto & info) { return info.generator_id == uuid; });
  return generator_itr == generator_info.end() ? "NOT FOUND" : generator_itr->generator_name.data;
}

TrajectoryRankerNode::TrajectoryRankerNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"trajectory_ranker_node", node_options},
  pub_marker_{this->create_publisher<MarkerArray>("~/output/markers", 1)},
  listener_{std::make_unique<evaluation::ParamListener>(get_node_parameters_interface())},
  route_handler_{std::make_shared<RouteHandler>()},
  previous_points_{nullptr}
{
  const auto vehicle_info = std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo());

  evaluator_ = std::make_shared<Evaluator>(route_handler_, vehicle_info);

  const auto metrics = listener_->get_params().metrics;
  for (size_t i = 0; i < metrics.size(); i++) {
    evaluator_->load_metric(metrics.at(i), i, listener_->get_params().resolution);
  }

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletRoute::ConstSharedPtr msg) { route_handler_->setRoute(*msg); });
}

void TrajectoryRankerNode::process(const Trajectories::ConstSharedPtr msg)
{
  publish(score(msg));
}

auto TrajectoryRankerNode::score(const Trajectories::ConstSharedPtr msg)
  -> Trajectories::ConstSharedPtr
{
  if (!route_handler_->isHandlerReady()) {
    return msg;
  }

  const auto odometry_ptr = std::const_pointer_cast<Odometry>(sub_odometry_.takeData());
  if (odometry_ptr == nullptr) {
    return msg;
  }

  const auto objects_ptr = std::const_pointer_cast<PredictedObjects>(sub_objects_.takeData());
  if (objects_ptr == nullptr) {
    return msg;
  }
  const auto preferred_lanes =
    std::make_shared<lanelet::ConstLanelets>(route_handler_->getPreferredLanelets());

  std::vector<Trajectory> trajectories;
  trajectories.reserve(msg->trajectories.size());

  evaluator_->clear();

  const auto params = parameters();

  for (const auto & t : msg->trajectories) {
    // TODO(satoshi-ota): remove this lambda.
    const auto points = [&t, &msg, &odometry_ptr, &params]() {
      return generator_name(t.generator_id, msg->generator_info) == "frenet_planner"
               ? t.points
               : utils::sampling(
                   t.points, odometry_ptr->pose.pose, params->sample_num, params->resolution);
    }();

    const auto core_data = std::make_shared<CoreData>(
      std::make_shared<TrajectoryPoints>(t.points), std::make_shared<TrajectoryPoints>(points),
      previous_points_, objects_ptr, odometry_ptr, preferred_lanes, t.header, t.generator_id);

    evaluator_->add(core_data);
  }

  const auto best_data = evaluator_->best(params);
  previous_points_ = best_data == nullptr ? nullptr : best_data->points();

  evaluator_->show();

  pub_marker_->publish(*evaluator_->marker());

  for (const auto & result : evaluator_->results()) {
    const auto scored_trajectory = autoware_new_planning_msgs::build<Trajectory>()
                                     .header(result->header())
                                     .generator_id(result->uuid())
                                     .points(*result->original())
                                     .score(result->total());
    trajectories.push_back(scored_trajectory);
  }

  const auto new_trajectories = autoware_new_planning_msgs::build<Trajectories>()
                                  .trajectories(trajectories)
                                  .generator_info(msg->generator_info);

  return std::make_shared<Trajectories>(new_trajectories);
}

auto TrajectoryRankerNode::parameters() const -> std::shared_ptr<EvaluatorParameters>
{
  const auto node_params = listener_->get_params();

  const auto parameters =
    std::make_shared<EvaluatorParameters>(node_params.metrics.size(), node_params.sample_num);

  parameters->resolution = node_params.resolution;
  parameters->score_weight = node_params.score_weight;
  parameters->time_decay_weight.at(0) = node_params.time_decay_weight.s0;
  parameters->time_decay_weight.at(1) = node_params.time_decay_weight.s1;
  parameters->time_decay_weight.at(2) = node_params.time_decay_weight.s2;
  parameters->time_decay_weight.at(3) = node_params.time_decay_weight.s3;
  parameters->time_decay_weight.at(4) = node_params.time_decay_weight.s4;
  parameters->time_decay_weight.at(5) = node_params.time_decay_weight.s5;

  return parameters;
}

}  // namespace autoware::trajectory_selector::trajectory_ranker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_ranker::TrajectoryRankerNode)
