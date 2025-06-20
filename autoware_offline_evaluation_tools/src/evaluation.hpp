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

#ifndef EVALUATION_HPP_
#define EVALUATION_HPP_

#include "autoware/offline_evaluation_tools/data_structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"
#include "bag_handler.hpp"

#include <autoware/trajectory_selector_common/evaluation.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

using autoware_planning_msgs::msg::Trajectory;

class BagEvaluator : public Evaluator
{
public:
  BagEvaluator(
    const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<DataAugmentParameters> & parameters);

  void setup(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<TrajectoryPoints> & previous_points);

  auto loss(const std::shared_ptr<EvaluatorParameters> & parameters)
    -> std::pair<double, std::shared_ptr<TrajectoryPoints>>;

  auto tf() const -> std::shared_ptr<TFMessage> { return tf_; };

  auto objects() const -> std::shared_ptr<PredictedObjects> { return objects_; }

  auto marker() const -> std::shared_ptr<MarkerArray>;

private:
  auto preferred_lanes(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<RouteHandler> & route_handler)
    const -> std::shared_ptr<lanelet::ConstLanelets>;

  auto objects(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<DataAugmentParameters> & parameters) const
    -> std::shared_ptr<PredictedObjects>;

  auto ground_truth(
    const std::shared_ptr<BagData> & bag_data,
    const std::shared_ptr<DataAugmentParameters> & parameters) const
    -> std::shared_ptr<TrajectoryPoints>;

  auto ground_truth_from_live_trajectory(
    const std::shared_ptr<ReplayEvaluationData> & replay_data,
    const Trajectory & live_trajectory) const
    -> std::shared_ptr<TrajectoryPoints>;

  auto get_localization_at_time(
    const std::shared_ptr<ReplayEvaluationData> & replay_data,
    const rcutils_time_point_value_t target_timestamp) const
    -> std::shared_ptr<Odometry>;

  auto interpolate_localization(
    const std::shared_ptr<Odometry> & odom1,
    const std::shared_ptr<Odometry> & odom2,
    const double ratio) const
    -> std::shared_ptr<Odometry>;

  auto convert_localization_to_trajectory_point(
    const Odometry & localization,
    const builtin_interfaces::msg::Duration & time_from_start) const
    -> autoware_planning_msgs::msg::TrajectoryPoint;

  std::pair<double, double> calculate_displacement_errors(
    const std::shared_ptr<TrajectoryPoints> & candidate_trajectory,
    const std::shared_ptr<TrajectoryPoints> & ground_truth_trajectory) const;

  double calculate_euclidean_distance(
    const geometry_msgs::msg::Pose & pose1,
    const geometry_msgs::msg::Pose & pose2) const;

  double calculate_minimum_ttc(
    const std::shared_ptr<TrajectoryPoints> & trajectory,
    const std::shared_ptr<PredictedObjects> & objects) const;

  std::pair<bool, double> check_speed_limit_violations(
    const std::shared_ptr<TrajectoryPoints> & trajectory,
    const double speed_limit_mps = 13.89) const; // Default 50 km/h

  std::pair<bool, double> check_lane_keeping_violations(
    const std::shared_ptr<TrajectoryPoints> & trajectory,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes) const;

  std::tuple<double, double, double> calculate_comfort_metrics(
    const std::shared_ptr<TrajectoryPoints> & trajectory) const;

  void evaluate_trajectory_safety(
    const std::shared_ptr<TrajectoryPoints> & trajectory,
    const std::shared_ptr<PredictedObjects> & objects,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes) const;

  void evaluate_and_save_trajectory_safety(
    const std::shared_ptr<TrajectoryPoints> & trajectory,
    const std::shared_ptr<PredictedObjects> & objects,
    const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes,
    const std::shared_ptr<TrajectoryPoints> & ground_truth,
    rosbag2_cpp::Writer & bag_writer) const;

  void save_evaluation_results_to_bag(
    const double ttc, const std::pair<double, double> & displacement_errors,
    const std::pair<bool, double> & speed_check, const std::pair<bool, double> & lane_check,
    const std::tuple<double, double, double> & comfort_metrics,
    rosbag2_cpp::Writer & bag_writer) const;


  auto augment_data(
    const std::shared_ptr<BagData> & bag_data, const std::shared_ptr<VehicleInfo> & vehicle_info,
    const std::shared_ptr<DataAugmentParameters> & parameters) const
    -> std::vector<std::shared_ptr<TrajectoryPoints>>;

  std::shared_ptr<DataAugmentParameters> parameters_;

  std::shared_ptr<TFMessage> tf_;

  std::shared_ptr<Odometry> odometry_;

  std::shared_ptr<SteeringReport> steering_;

  std::shared_ptr<PredictedObjects> objects_;

  std::shared_ptr<lanelet::ConstLanelets> preferred_lanes_;
};
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // EVALUATION_HPP_
