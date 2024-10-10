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

#include "autoware/trajectory_selector_common/evaluation.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory_selector_common/utils.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

namespace autoware::trajectory_selector
{
void Evaluator::loadMetricPlugin(const std::string & name, const size_t index)
{
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);

    plugin->init(vehicle_info_);
    plugin->set_index(index);

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

void Evaluator::evaluate()
{
  for (const auto & result : results_) {
    for (const auto & metric_ptr : metric_ptrs_) {
      metric_ptr->evaluate(result);
    }
  }
}

void Evaluator::normalize()
{
  if (results_.size() < 2) return;

  const auto range = [this](const size_t index) {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    for (const auto & result : results_) {
      min = std::min(min, result->score(index));
      max = std::max(max, result->score(index));
    }
    return std::make_pair(min, max);
  };

  for (const auto & metric_ptr : metric_ptrs_) {
    const auto [min, max] = range(metric_ptr->index());
    for (auto & data : results_) {
      data->normalize(min, max, metric_ptr->index(), metric_ptr->is_deviation());
    }
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
  const auto ptr = std::make_shared<DataInterface>(core_data, metric_ptrs_.size());
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

  evaluate();

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

auto Evaluator::statistics(const size_t metric_index) const -> std::pair<double, double>
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
    std::tie(ave, dev) = update(ave, dev, results_.at(i)->score(metric_index), i);
  }

  return std::make_pair(ave, dev);
}

void Evaluator::show() const
{
  const auto best_data = best();

  if (best_data == nullptr) {
    return;
  }

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << "\n";
  ss << "size:" << results_.size() << "\n";
  ss << "tag:" << best_data->tag() << "\n";
  for (const auto & metric_ptr : metric_ptrs_) {
    const auto [mean, dev] = statistics(metric_ptr->index());
    ss << metric_ptr->name() << ":" << " mean:" << mean << " std:" << std::sqrt(dev) << "\n";
  }
  ss << "total:" << best_data->total();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(__func__), ss.str());
}

auto Evaluator::marker() const -> std::shared_ptr<MarkerArray>
{
  using autoware::universe_utils::createDefaultMarker;
  using autoware::universe_utils::createMarkerColor;
  using autoware::universe_utils::createMarkerScale;

  MarkerArray msg;

  const auto best_data = best();
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
  for (size_t i = 0; i < results().size(); ++i) {
    const auto result = results().at(i);

    if (result == nullptr) continue;

    for (const auto & metric_ptr : metric_ptrs_) {
      const auto score = result->score(metric_ptr->index());
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->original(), score, result->feasible(), metric_ptr->name(), i));
    }

    {
      min = std::min(min, result->total());
      max = std::max(max, result->total());
    }
  }

  for (size_t i = 0; i < results().size(); ++i) {
    const auto result = results().at(i);

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

  return std::make_shared<MarkerArray>(msg);
}
}  // namespace autoware::trajectory_selector
