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

#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include "autoware_new_planning_msgs/msg/evaluation_info.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_selector
{
void Evaluator::load_metric(
  const std::string & name, const size_t index, const double time_resolution)
{
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);

    plugin->init(vehicle_info_, time_resolution);
    plugin->set_index(index);

    // Check if the plugin is already registered.
    for (const auto & running_plugin : plugins_) {
      if (plugin->name() == running_plugin->name()) {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger(__func__), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // register
    plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(__func__), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(__func__), "The scene plugin '" << name << "' is not available.");
  }
}

void Evaluator::unload_metric(const std::string & name)
{
  auto it = std::remove_if(
    plugins_.begin(), plugins_.end(),
    [&](const std::shared_ptr<MetricInterface> plugin) { return plugin->name() == name; });

  if (it == plugins_.end()) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(__func__),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    plugins_.erase(it, plugins_.end());
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(__func__), "The scene plugin '" << name << "' is unloaded.");
  }
}

void Evaluator::evaluate(const std::vector<double> & max_value)
{
  for (const auto & result : results_) {
    for (const auto & plugin : plugins_) {
      plugin->evaluate(result, max_value.at(plugin->index()));
    }
  }
}

void Evaluator::normalize()
{
  if (results_.empty()) return;

  if (results_.size() < 2) {
    const auto data = results_.front();
    for (const auto & plugin : plugins_) {
      data->normalize(0.0, data->score(plugin->index()), plugin->index());
    }
    return;
  }

  const auto range = [this](const size_t index) {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    for (const auto & result : results_) {
      min = std::min(min, result->score(index));
      max = std::max(max, result->score(index));
    }
    return std::make_pair(min, max);
  };

  for (const auto & plugin : plugins_) {
    const auto [min, max] = range(plugin->index());
    for (auto & data : results_) {
      data->normalize(min, max, plugin->index(), plugin->is_deviation());
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
  const auto ptr = std::make_shared<DataInterface>(core_data, plugins_.size());
  results_.push_back(ptr);
}

auto Evaluator::best(
  const std::shared_ptr<EvaluatorParameters> & parameters,
  const std::string & exclude) -> std::shared_ptr<DataInterface>
{
  pruning();

  evaluate(parameters->metrics_max_value);

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
  for (const auto & plugin : plugins_) {
    const auto [mean, dev] = statistics(plugin->index());
    ss << plugin->name() << ":" << " mean:" << mean << " std:" << std::sqrt(dev) << "\n";
  }
  ss << "total:" << best_data->total();
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(__func__), ss.str());
}

auto Evaluator::marker() const -> std::shared_ptr<MarkerArray>
{
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;

  MarkerArray msg;

  const auto best_data = best();
  if (best_data != nullptr) {
    Marker marker = create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "best_score", 0L, Marker::LINE_STRIP,
      create_marker_scale(0.2, 0.0, 0.0), create_marker_color(1.0, 1.0, 1.0, 0.999));
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

    for (const auto & plugin : plugins_) {
      const auto score = result->score(plugin->index());
      msg.markers.push_back(trajectory_selector::utils::to_marker(
        result->points(), score, result->feasible(), plugin->name(), i));
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
        result->points(), score, result->feasible(), "TOTAL", i));
    }
  }

  autoware_utils::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "preferred_lanes", route_handler_->getPreferredLanelets(),
      create_marker_color(0.16, 1.0, 0.69, 0.2)),
    &msg);

  return std::make_shared<MarkerArray>(msg);
}

auto Evaluator::score_debug(const std::shared_ptr<EvaluatorParameters> & parameters) const
  -> std::shared_ptr<TrajectoriesDebug>
{
  TrajectoriesDebug msg;
  msg.header.frame_id = "map";
  msg.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();

  for (size_t i = 0; i < results_.size(); i++) {
    autoware_new_planning_msgs::msg::EvaluationInfo eval_info;

    const auto result = results_.at(i);
    if (result == nullptr) continue;
    eval_info.generator_info.generator_name.data = result->tag();
    eval_info.generator_info.generator_id = result->uuid();
    eval_info.score = result->total();

    for (const auto & plugin : plugins_) {
      eval_info.metrics.push_back(plugin->name());
      eval_info.metrics_value.push_back(result->score(plugin->index()));
      eval_info.weights.push_back(parameters->score_weight.at(plugin->index()));
    }
    msg.scores.push_back(eval_info);
  }

  return std::make_shared<TrajectoriesDebug>(msg);
}
}  // namespace autoware::trajectory_selector
