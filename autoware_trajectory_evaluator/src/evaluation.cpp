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
#include "autoware/trajectory_selector_common/data_structs.hpp"
#include "autoware/trajectory_selector_common/utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

namespace autoware::trajectory_selector::trajectory_evaluator
{
void Evaluator::loadMetricPlugin(const std::string & name)
{
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);

    // Check if the plugin is already registered.
    for (const auto & running_plugin : metric_ptrs_) {
      if (plugin->name() == running_plugin->name()) {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger(__func__), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // register
    metric_ptrs_.push_back(plugin);
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(__func__), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(__func__), "The scene plugin '" << name << "' is not available.");
  }
}

void Evaluator::normalize()
{
  if (results_.size() < 2) return;

  const auto range = [this](const auto & score_type) {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    for (const auto & result : results_) {
      min = std::min(min, result->score(score_type));
      max = std::max(max, result->score(score_type));
    }
    return std::make_pair(min, max);
  };

  const auto [s0_min, s0_max] = range(SCORE::LATERAL_COMFORTABILITY);
  const auto [s1_min, s1_max] = range(SCORE::LONGITUDINAL_COMFORTABILITY);
  const auto [s2_min, s2_max] = range(SCORE::EFFICIENCY);
  const auto [s3_min, s3_max] = range(SCORE::SAFETY);
  const auto [s4_min, s4_max] = range(SCORE::ACHIEVABILITY);
  const auto [s5_min, s5_max] = range(SCORE::CONSISTENCY);

  for (auto & data : results_) {
    data->normalize(s0_min, s0_max, SCORE::LATERAL_COMFORTABILITY, true);
    data->normalize(s1_min, s1_max, SCORE::LONGITUDINAL_COMFORTABILITY, true);
    data->normalize(s2_min, s2_max, SCORE::EFFICIENCY);
    data->normalize(s3_min, s3_max, SCORE::SAFETY);
    data->normalize(s4_min, s4_max, SCORE::ACHIEVABILITY, true);
    data->normalize(s5_min, s5_max, SCORE::CONSISTENCY, true);
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
  const auto ptr = std::make_shared<DataInterface>(core_data, route_handler_, vehicle_info_);
  results_.push_back(ptr);
}

void Evaluator::setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  std::for_each(results_.begin(), results_.end(), [&previous_points](const auto & result) {
    result->setup(previous_points);
  });
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
  ss << " size              :" << results_.size()                                       << "\n";
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
