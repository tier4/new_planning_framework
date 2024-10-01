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

#include "autoware/trajectory_evaluator/evaluation.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory_evaluator/data_structs.hpp"
#include "autoware/trajectory_evaluator/utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/LineString.h>

namespace autoware::trajectory_selector::trajectory_evaluator
{

DataInterface::DataInterface(
  const std::shared_ptr<CoreData> & core_data, const std::shared_ptr<RouteHandler> & route_handler,
  const std::shared_ptr<lanelet::ConstLanelets> & preferred_lanes,
  const std::shared_ptr<VehicleInfo> & vehicle_info)
: core_data_{core_data},
  route_handler_{route_handler},
  preferred_lanes_{preferred_lanes},
  vehicle_info_{vehicle_info},
  metrics_(static_cast<size_t>(METRIC::SIZE), std::vector<double>(core_data->points->size(), 0.0)),
  scores_(static_cast<size_t>(SCORE::SIZE), 0.0)
{
  evaluate();
}

void DataInterface::evaluate()
{
  for (size_t i = 0; i < core_data_->points->size(); i++) {
    metrics_.at(static_cast<size_t>(METRIC::LATERAL_ACCEL)).at(i) = lateral_accel(i);
    metrics_.at(static_cast<size_t>(METRIC::LONGITUDINAL_JERK)).at(i) = longitudinal_jerk(i);
    metrics_.at(static_cast<size_t>(METRIC::MINIMUM_TTC)).at(i) = minimum_ttc(i);
    metrics_.at(static_cast<size_t>(METRIC::TRAVEL_DISTANCE)).at(i) = travel_distance(i);
    metrics_.at(static_cast<size_t>(METRIC::LATERAL_DEVIATION)).at(i) = lateral_deviation(i);
    metrics_.at(static_cast<size_t>(METRIC::TRAJECTORY_DEVIATION)).at(i) = trajectory_deviation(i);
  }
}

void DataInterface::compress(const std::vector<std::vector<double>> & weight)
{
  scores_.at(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY)) =
    compress(weight, METRIC::LATERAL_ACCEL);
  scores_.at(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY)) =
    compress(weight, METRIC::LONGITUDINAL_JERK);
  scores_.at(static_cast<size_t>(SCORE::EFFICIENCY)) = compress(weight, METRIC::TRAVEL_DISTANCE);
  scores_.at(static_cast<size_t>(SCORE::SAFETY)) = compress(weight, METRIC::MINIMUM_TTC);
  scores_.at(static_cast<size_t>(SCORE::ACHIEVABILITY)) =
    compress(weight, METRIC::LATERAL_DEVIATION);
  scores_.at(static_cast<size_t>(SCORE::CONSISTENCY)) =
    compress(weight, METRIC::TRAJECTORY_DEVIATION);
}

double DataInterface::lateral_accel(const size_t idx) const
{
  const auto radius =
    vehicle_info_->wheel_base_m / std::tan(core_data_->points->at(idx).front_wheel_angle_rad);
  const auto speed = core_data_->points->at(idx).longitudinal_velocity_mps;
  return std::abs(speed * speed / radius);
}

double DataInterface::longitudinal_jerk(const size_t idx) const
{
  if (idx + 2 > core_data_->points->size()) return 0.0;

  const auto jerk = (core_data_->points->at(idx + 1).acceleration_mps2 -
                     core_data_->points->at(idx).acceleration_mps2) /
                    0.5;
  return std::abs(jerk);
}

double DataInterface::minimum_ttc(const size_t idx) const
{
  // TODO(satoshi-ota): linear interpolation
  return utils::time_to_collision(core_data_->points, core_data_->objects, idx);
}

double DataInterface::travel_distance(const size_t idx) const
{
  return autoware::motion_utils::calcSignedArcLength(*core_data_->points, 0L, idx);
}

double DataInterface::lateral_deviation(const size_t idx) const
{
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(
    *preferred_lanes_, autoware::universe_utils::getPose(core_data_->points->at(idx)));
  return std::abs(arc_coordinates.distance);
}

double DataInterface::trajectory_deviation(const size_t idx) const
{
  if (core_data_->previous_points == nullptr) return 0.0;

  if (idx + 1 > core_data_->previous_points->size()) return 0.0;

  const auto & p1 = autoware::universe_utils::getPose(core_data_->points->at(idx));
  const auto & p2 = autoware::universe_utils::getPose(core_data_->previous_points->at(idx));
  return autoware::universe_utils::calcSquaredDistance2d(p1, p2);
}

bool DataInterface::feasible() const
{
  const auto condition = [](const auto & p) { return p.longitudinal_velocity_mps >= 0.0; };
  return std::all_of(core_data_->points->begin(), core_data_->points->end(), condition);
}

void DataInterface::normalize(
  const double min, const double max, const size_t score_type, const bool flip)
{
  scores_.at(score_type) = flip ? (max - scores_.at(score_type)) / (max - min)
                                : (scores_.at(score_type) - min) / (max - min);
}

auto DataInterface::compress(
  const std::vector<std::vector<double>> & weight, const METRIC & metric_type) const -> double
{
  const auto & w = weight.at(static_cast<size_t>(metric_type));
  const auto & metric = metrics_.at(static_cast<size_t>(metric_type));
  return std::inner_product(w.begin(), w.end(), metric.begin(), 0.0);
}

auto DataInterface::score(const SCORE & score_type) const -> double
{
  return scores_.at(static_cast<size_t>(score_type));
}

void DataInterface::weighting(const std::vector<double> & weight)
{
  total_ = std::inner_product(weight.begin(), weight.end(), scores_.begin(), 0.0);
}

void Evaluator::normalize()
{
  const auto range = [this](const size_t idx) {
    const auto min_itr = std::min_element(
      results_.begin(), results_.end(),
      [&idx](const auto & a, const auto & b) { return a->scores().at(idx) < b->scores().at(idx); });
    const auto max_itr = std::max_element(
      results_.begin(), results_.end(),
      [&idx](const auto & a, const auto & b) { return a->scores().at(idx) < b->scores().at(idx); });

    const auto & min = (*min_itr)->scores().at(idx);
    const auto & max = (*max_itr)->scores().at(idx);

    return std::make_pair(min, max);
  };

  const auto [s0_min, s0_max] = range(static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY));
  const auto [s1_min, s1_max] = range(static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY));
  const auto [s2_min, s2_max] = range(static_cast<size_t>(SCORE::EFFICIENCY));
  const auto [s3_min, s3_max] = range(static_cast<size_t>(SCORE::SAFETY));
  const auto [s4_min, s4_max] = range(static_cast<size_t>(SCORE::ACHIEVABILITY));
  const auto [s5_min, s5_max] = range(static_cast<size_t>(SCORE::CONSISTENCY));

  for (auto & data : results_) {
    data->normalize(s0_min, s0_max, static_cast<size_t>(SCORE::LATERAL_COMFORTABILITY), true);
    data->normalize(s1_min, s1_max, static_cast<size_t>(SCORE::LONGITUDINAL_COMFORTABILITY), true);
    data->normalize(s2_min, s2_max, static_cast<size_t>(SCORE::EFFICIENCY));
    data->normalize(s3_min, s3_max, static_cast<size_t>(SCORE::SAFETY));
    data->normalize(s4_min, s4_max, static_cast<size_t>(SCORE::ACHIEVABILITY), true);
    data->normalize(s5_min, s5_max, static_cast<size_t>(SCORE::CONSISTENCY), true);
  }
}

void Evaluator::pruning()
{
  const auto itr =
    std::remove_if(results_.begin(), results_.end(), [](const auto & d) { return !d->feasible(); });

  results_.erase(itr, results_.end());
}

void Evaluator::compress(const std::vector<std::vector<double>> & weight)
{
  std::for_each(
    results_.begin(), results_.end(), [&weight](auto & data) { data->compress(weight); });
}

void Evaluator::weighting(const std::vector<double> & weight)
{
  std::for_each(
    results_.begin(), results_.end(), [&weight](auto & data) { data->weighting(weight); });

  std::sort(results_.begin(), results_.end(), [](const auto & a, const auto & b) {
    return a->total() > b->total();
  });
}

auto Evaluator::get(const std::string & tag) const -> std::shared_ptr<DataInterface>
{
  const auto itr = std::find_if(
    results_.begin(), results_.end(), [&tag](const auto & data) { return data->tag() == tag; });

  return itr != results_.end() ? *itr : nullptr;
}

void Evaluator::add(const std::shared_ptr<CoreData> & core_data)
{
  const auto preferred_lanes = route_handler_->getPreferredLanelets();

  lanelet::ConstLanelet nearest{};
  if (!route_handler_->getClosestPreferredLaneletWithinRoute(
        core_data->odometry->pose.pose, &nearest)) {
    RCLCPP_ERROR(rclcpp::get_logger(__func__), "couldn't find nearest preferred lane.");
  }

  const auto itr = std::find_if(
    preferred_lanes.begin(), preferred_lanes.end(),
    [&nearest](const auto & lanelet) { return lanelet.id() == nearest.id(); });

  double length = 0.0;

  std::shared_ptr<lanelet::ConstLanelets> preferred_lanes_nearby{};
  std::for_each(
    itr, preferred_lanes.end(), [&length, &preferred_lanes_nearby](const auto & lanelet) {
      length +=
        static_cast<double>(boost::geometry::length(lanelet.centerline().basicLineString()));
      constexpr double threshold = 150.0;
      if (length < threshold) preferred_lanes_nearby->push_back(lanelet);
    });

  const auto ptr = std::make_shared<DataInterface>(
    core_data, route_handler_, preferred_lanes_nearby, vehicle_info_);
  results_.push_back(ptr);
}

auto Evaluator::best(
  const std::shared_ptr<EvaluatorParameters> & parameters,
  const std::string & exclude) -> std::shared_ptr<DataInterface>
{
  pruning();

  compress(parameters->time_decay_weight);

  normalize();

  weighting(parameters->score_weight);

  return best(exclude);
}

auto Evaluator::best(const std::string & exclude) const -> std::shared_ptr<DataInterface>
{
  if (results_.empty()) return nullptr;

  const auto itr = std::find_if(results_.begin(), results_.end(), [&exclude](const auto & result) {
    return result->tag() != exclude && result->feasible();
  });
  if (results_.end() == itr) return nullptr;

  return *itr;
}

auto Evaluator::statistics(const SCORE & score_type) const -> std::pair<double, double>
{
  double ave = 0.0;
  double dev = 0.0;

  const auto update = [](const double ave, const double dev, const double value, const size_t i) {
    const auto new_ave = (i * ave + value) / (i + 1);
    const auto new_dev =
      (i * (ave * ave + dev * dev) + value * value) / (i + 1) - new_ave * new_ave;
    return std::make_pair(new_ave, new_dev);
  };

  for (size_t i = 0; i < results_.size(); i++) {
    std::tie(ave, dev) = update(ave, dev, results_.at(i)->score(score_type), i);
  }

  return std::make_pair(ave, dev);
}

void Evaluator::show() const
{
  const auto best_data = best();

  if (best_data == nullptr) {
    return;
  }

  const auto s0 = statistics(SCORE::LATERAL_COMFORTABILITY);
  const auto s1 = statistics(SCORE::LONGITUDINAL_COMFORTABILITY);
  const auto s2 = statistics(SCORE::EFFICIENCY);
  const auto s3 = statistics(SCORE::SAFETY);
  const auto s4 = statistics(SCORE::ACHIEVABILITY);
  const auto s5 = statistics(SCORE::CONSISTENCY);

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << "\n";
  // clang-format off
  ss << " tag               :" << best_data->tag()                                      << "\n";
  ss << " lat comfortability:" << best_data->score(SCORE::LATERAL_COMFORTABILITY)       << " mean:" << s0.first << " std:" << std::sqrt(s0.second) << "\n"; // NOLINT
  ss << " lon comfortability:" << best_data->score(SCORE::LONGITUDINAL_COMFORTABILITY)  << " mean:" << s1.first << " std:" << std::sqrt(s1.second) << "\n"; // NOLINT
  ss << " efficiency        :" << best_data->score(SCORE::EFFICIENCY)                   << " mean:" << s2.first << " std:" << std::sqrt(s2.second) << "\n"; // NOLINT
  ss << " safety            :" << best_data->score(SCORE::SAFETY)                       << " mean:" << s3.first << " std:" << std::sqrt(s3.second) << "\n"; // NOLINT
  ss << " achievability     :" << best_data->score(SCORE::ACHIEVABILITY)                << " mean:" << s4.first << " std:" << std::sqrt(s4.second) << "\n"; // NOLINT
  ss << " consistency       :" << best_data->score(SCORE::CONSISTENCY)                  << " mean:" << s5.first << " std:" << std::sqrt(s5.second) << "\n"; // NOLINT
  ss << " total             :" << best_data->total();
  // clang-format on
  RCLCPP_INFO_STREAM(rclcpp::get_logger(__func__), ss.str());
}
}  // namespace autoware::trajectory_selector::trajectory_evaluator
