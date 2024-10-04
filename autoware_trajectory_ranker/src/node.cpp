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

namespace autoware::trajectory_selector::trajectory_ranker
{

TrajectoryRankerNode::TrajectoryRankerNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"trajectory_ranker_node", node_options},
  pub_marker_{this->create_publisher<MarkerArray>("~/output/markers", 1)},
  listener_{std::make_unique<evaluation::ParamListener>(get_node_parameters_interface())},
  route_handler_{std::make_shared<RouteHandler>()},
  previous_points_{nullptr}
{
  const auto vehicle_info = std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo());

  evaluator_ = std::make_shared<trajectory_evaluator::Evaluator>(route_handler_, vehicle_info);

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

  for (const auto & t : msg->trajectories) {
    const auto points = utils::sampling(t.points, odometry_ptr->pose.pose, 20, 0.5);

    const auto core_data = std::make_shared<CoreData>(
      std::make_shared<TrajectoryPoints>(t.points), std::make_shared<TrajectoryPoints>(points),
      objects_ptr, odometry_ptr, preferred_lanes, t.header, t.generator_id);

    evaluator_->add(core_data);
  }

  evaluator_->setup(previous_points_);

  const auto best_data = evaluator_->best(parameters());
  previous_points_ = best_data == nullptr ? nullptr : best_data->points();

  evaluator_->show();

  visualize(evaluator_, best_data);

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

  const auto parameters = std::make_shared<EvaluatorParameters>(node_params.sample_num);

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

void TrajectoryRankerNode::visualize(
  const std::shared_ptr<trajectory_evaluator::Evaluator> evaluator,
  const std::shared_ptr<trajectory_evaluator::DataInterface> & best_data) const
{
  using autoware::universe_utils::createDefaultMarker;
  using autoware::universe_utils::createMarkerColor;
  using autoware::universe_utils::createMarkerScale;

  MarkerArray msg;

  if (best_data != nullptr) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "best_score", 0L, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & point : *best_data->points()) {
      marker.points.push_back(point.pose.position);
    }
    msg.markers.push_back(marker);
  }

  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < evaluator->results().size(); ++i) {
    const auto result = evaluator->results().at(i);

    if (result == nullptr) continue;

    {
      std::stringstream ss;
      ss << magic_enum::enum_name(SCORE::LATERAL_COMFORTABILITY);
      const auto score = result->score(SCORE::LATERAL_COMFORTABILITY);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), ss.str(), i));
    }
    {
      std::stringstream ss;
      ss << magic_enum::enum_name(SCORE::LONGITUDINAL_COMFORTABILITY);
      const auto score = result->score(SCORE::LONGITUDINAL_COMFORTABILITY);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), ss.str(), i));
    }
    {
      std::stringstream ss;
      ss << magic_enum::enum_name(SCORE::EFFICIENCY);
      const auto score = result->score(SCORE::EFFICIENCY);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), ss.str(), i));
    }
    {
      std::stringstream ss;
      ss << magic_enum::enum_name(SCORE::SAFETY);
      const auto score = result->score(SCORE::SAFETY);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), ss.str(), i));
    }
    {
      std::stringstream ss;
      ss << magic_enum::enum_name(SCORE::ACHIEVABILITY);
      const auto score = result->score(SCORE::ACHIEVABILITY);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), ss.str(), i));
    }
    {
      std::stringstream ss;
      ss << magic_enum::enum_name(SCORE::CONSISTENCY);
      const auto score = result->score(SCORE::CONSISTENCY);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), ss.str(), i));
    }
    {
      min = std::min(min, result->total());
      max = std::max(max, result->total());
    }
  }

  for (size_t i = 0; i < evaluator->results().size(); ++i) {
    const auto result = evaluator->results().at(i);

    if (result == nullptr) continue;

    if (std::abs(max - min) < std::numeric_limits<double>::epsilon()) {
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->points(), 1.0, result->feasible(), "TOTAL", i));
    } else {
      // convert score to 0.0~1.0 value
      const auto score = (result->total() - min) / (max - min);
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), "TOTAL", i));
    }
  }

  pub_marker_->publish(msg);
}

}  // namespace autoware::trajectory_selector::trajectory_ranker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_ranker::TrajectoryRankerNode)
