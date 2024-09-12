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

namespace autoware::trajectory_selector::trajectory_ranker
{

TrajectoryRankerNode::TrajectoryRankerNode(const rclcpp::NodeOptions & node_options)
: TrajectoryFilterInterface{"trajectory_ranker_node", node_options},
  parameters_{std::make_shared<parameters::ParamListener>(get_node_parameters_interface())},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()}
{
}

void TrajectoryRankerNode::process(const Trajectories::ConstSharedPtr msg)
{
  publish(score(msg));
}

auto TrajectoryRankerNode::score(const Trajectories::ConstSharedPtr msg)
  -> Trajectories::ConstSharedPtr
{
  std::vector<double> lateral_accel_values;
  std::vector<double> minimum_ttc_values;
  std::vector<double> longitudinal_jerk_values;
  std::vector<double> travel_distance_values;

  const auto param = parameters_->get_params();

  const auto odometry_ptr = sub_odometry_.takeData();
  if (odometry_ptr == nullptr) {
    return msg;
  }

  const auto objects = [this]() {
    const auto ptr = sub_objects_.takeData();
    return ptr == nullptr ? PredictedObjects{} : *ptr;
  }();

  const auto resample_num = static_cast<size_t>(param.resample_num);

  std::vector<Trajectory> trajectories;
  trajectories.reserve(msg->trajectories.size());

  for (const auto & t : msg->trajectories) {
    const auto points =
      resampling(t.points, odometry_ptr->pose.pose, param.resample_num, param.time_resolution);
    for (size_t i = 0; i < resample_num - 1; i++) {
      lateral_accel_values.push_back(lateral_accel(vehicle_info_, points, i));
      longitudinal_jerk_values.push_back(longitudinal_jerk(points, i));
      minimum_ttc_values.push_back(minimum_ttc(objects, points, i));
      travel_distance_values.push_back(travel_distance(points, i));
    }

    {
      lateral_accel_values.push_back(lateral_accel(vehicle_info_, points, resample_num - 1));
      longitudinal_jerk_values.push_back(0.0);
      minimum_ttc_values.push_back(minimum_ttc(objects, points, resample_num - 1));
      travel_distance_values.push_back(travel_distance(points, resample_num - 1));
    }

    const auto score =
      param.weight.w0 * lateral_comfortability(lateral_accel_values, resample_num) +
      param.weight.w1 * longitudinal_comfortability(longitudinal_jerk_values, resample_num) +
      param.weight.w2 * efficiency(travel_distance_values, resample_num) +
      param.weight.w3 * safety(minimum_ttc_values, resample_num);
    const auto scored_trajectory = autoware_new_planning_msgs::build<Trajectory>()
                                     .header(t.header)
                                     .generator_id(t.generator_id)
                                     .points(t.points)
                                     .score(score);
    trajectories.push_back(scored_trajectory);
  }

  const auto new_trajectories = autoware_new_planning_msgs::build<Trajectories>()
                                  .trajectories(trajectories)
                                  .generator_info(msg->generator_info);

  return std::make_shared<Trajectories>(new_trajectories);
}

}  // namespace autoware::trajectory_selector::trajectory_ranker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::trajectory_ranker::TrajectoryRankerNode)
