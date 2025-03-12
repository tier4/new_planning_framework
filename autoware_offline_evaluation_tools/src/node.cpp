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
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "structs.hpp"

#include <autoware/trajectory_selector_common/type_alias.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <cstddef>
#include <fstream>
#include <memory>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
using autoware_utils::create_marker_color;
using autoware_utils::get_or_declare_parameter;
using autoware_utils::Polygon2d;

OfflineEvaluatorNode::OfflineEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("offline_evaluator_node", node_options),
  route_handler_{std::make_shared<RouteHandler>()},
  vehicle_info_{std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())},
  index_(0)
{
  pub_marker_ = create_publisher<MarkerArray>("~/output/markers", 1);

  pub_objects_ = create_publisher<PredictedObjects>(TOPIC::OBJECTS, rclcpp::QoS(1));

  pub_tf_ = create_publisher<TFMessage>(TOPIC::TF, rclcpp::QoS(1));

  pub_trajectory_ = create_publisher<Trajectory>(TOPIC::TRAJECTORY, rclcpp::QoS(1));

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  srv_play_ = this->create_service<Trigger>(
    "play",
    std::bind(&OfflineEvaluatorNode::play, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_rewind_ = this->create_service<Trigger>(
    "rewind",
    std::bind(&OfflineEvaluatorNode::rewind, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_route_ = this->create_service<Trigger>(
    "next_route",
    std::bind(
      &OfflineEvaluatorNode::next_route, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_weight_ = this->create_service<Trigger>(
    "weight_grid_search",
    std::bind(&OfflineEvaluatorNode::weight, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  srv_create_dataset_ = this->create_service<Trigger>(
    "create_dataset",
    std::bind(
      &OfflineEvaluatorNode::create_dataset, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  reader_.open(get_or_declare_parameter<std::string>(*this, "bag_path"));
}

auto OfflineEvaluatorNode::evaluator_parameters() -> std::shared_ptr<EvaluatorParameters>
{
  const auto metrics = get_or_declare_parameter<std::vector<std::string>>(*this, "metrics");
  const auto sample_num = get_or_declare_parameter<int>(*this, "sample_num");
  const auto parameters = std::make_shared<EvaluatorParameters>(metrics.size(), sample_num);
  parameters->score_weight = get_or_declare_parameter<std::vector<double>>(*this, "score_weight");
  parameters->time_decay_weight.at(0) =
    get_or_declare_parameter<std::vector<double>>(*this, "time_decay_weight.s0");
  parameters->time_decay_weight.at(1) =
    get_or_declare_parameter<std::vector<double>>(*this, "time_decay_weight.s1");
  parameters->time_decay_weight.at(2) =
    get_or_declare_parameter<std::vector<double>>(*this, "time_decay_weight.s2");
  parameters->time_decay_weight.at(3) =
    get_or_declare_parameter<std::vector<double>>(*this, "time_decay_weight.s3");
  parameters->time_decay_weight.at(4) =
    get_or_declare_parameter<std::vector<double>>(*this, "time_decay_weight.s4");
  parameters->time_decay_weight.at(5) =
    get_or_declare_parameter<std::vector<double>>(*this, "time_decay_weight.s5");

  return parameters;
}

auto OfflineEvaluatorNode::data_augument_parameters() -> std::shared_ptr<DataAugmentParameters>
{
  const auto parameters = std::make_shared<DataAugmentParameters>();

  parameters->sample_num = get_or_declare_parameter<int>(*this, "sample_num");
  parameters->resolution = get_or_declare_parameter<double>(*this, "resolution");
  parameters->target_state.lat_positions =
    get_or_declare_parameter<std::vector<double>>(*this, "target_state.lateral_positions");
  parameters->target_state.lat_velocities =
    get_or_declare_parameter<std::vector<double>>(*this, "target_state.lateral_velocities");
  parameters->target_state.lat_accelerations =
    get_or_declare_parameter<std::vector<double>>(*this, "target_state.lateral_accelerations");
  parameters->target_state.lon_positions =
    get_or_declare_parameter<std::vector<double>>(*this, "target_state.longitudinal_positions");
  parameters->target_state.lon_velocities =
    get_or_declare_parameter<std::vector<double>>(*this, "target_state.longitudinal_velocities");
  parameters->target_state.lon_accelerations =
    get_or_declare_parameter<std::vector<double>>(*this, "target_state.longitudinal_accelerations");

  return parameters;
}

auto OfflineEvaluatorNode::get_route() -> LaneletRoute::ConstSharedPtr
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back("/planning/mission_planning/route");
  reader_.set_filter(filter);

  if (!reader_.has_next()) {
    throw std::domain_error("not found route msg.");
  }

  rclcpp::Serialization<LaneletRoute> serializer;

  const auto deserialized_message = std::make_shared<LaneletRoute>();
  while (reader_.has_next()) {
    const auto next_data = reader_.read_next();
    if (next_data->topic_name == TOPIC::ROUTE) {
      rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      break;
    }
  }

  return deserialized_message;
}

void OfflineEvaluatorNode::update(const std::shared_ptr<BagData> & bag_data, const double dt)
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(TOPIC::TF);
  filter.topics.emplace_back(TOPIC::ODOMETRY);
  filter.topics.emplace_back(TOPIC::ACCELERATION);
  filter.topics.emplace_back(TOPIC::OBJECTS);
  filter.topics.emplace_back(TOPIC::STEERING);
  filter.topics.emplace_back(TOPIC::TRAJECTORY);
  reader_.set_filter(filter);

  bag_data->update(dt * 1e9);

  while (reader_.has_next()) {
    const auto next_data = reader_.read_next();
    rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);

    if (bag_data->ready()) {
      break;
    }

    if (next_data->topic_name == TOPIC::TF) {
      rclcpp::Serialization<TFMessage> serializer;
      const auto deserialized_message = std::make_shared<TFMessage>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers.at(TOPIC::TF))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::ODOMETRY) {
      rclcpp::Serialization<Odometry> serializer;
      const auto deserialized_message = std::make_shared<Odometry>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers.at(TOPIC::ODOMETRY))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::ACCELERATION) {
      rclcpp::Serialization<AccelWithCovarianceStamped> serializer;
      const auto deserialized_message = std::make_shared<AccelWithCovarianceStamped>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
        bag_data->buffers.at(TOPIC::ACCELERATION))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::OBJECTS) {
      rclcpp::Serialization<PredictedObjects> serializer;
      const auto deserialized_message = std::make_shared<PredictedObjects>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers.at(TOPIC::OBJECTS))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::STEERING) {
      rclcpp::Serialization<SteeringReport> serializer;
      const auto deserialized_message = std::make_shared<SteeringReport>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers.at(TOPIC::STEERING))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::TRAJECTORY) {
      index_++;
      rclcpp::Serialization<Trajectory> serializer;
      const auto deserialized_message = std::make_shared<Trajectory>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
        ->append(*deserialized_message);
    }
  }
}

void OfflineEvaluatorNode::play(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  const auto time_step = get_or_declare_parameter<double>(*this, "play.time_step");

  RCLCPP_INFO(get_logger(), "rosbag play now...");

  std::shared_ptr<TrajectoryPoints> previous_points{nullptr};

  const auto bag_evaluator =
    std::make_shared<BagEvaluator>(route_handler_, vehicle_info_, data_augument_parameters());

  const auto metrics = get_or_declare_parameter<std::vector<std::string>>(*this, "metrics");
  for (size_t i = 0; i < metrics.size(); i++) {
    bag_evaluator->load_metric(metrics.at(i), i, data_augument_parameters()->resolution);
  }

  const auto parameters = evaluator_parameters();

  while (reader_.has_next() && rclcpp::ok()) {
    update(bag_data, time_step);

    bag_evaluator->setup(bag_data, previous_points);

    const auto best_data = bag_evaluator->best(parameters);

    previous_points = best_data == nullptr ? nullptr : best_data->points();

    pub_tf_->publish(*bag_evaluator->tf());

    pub_objects_->publish(*bag_evaluator->objects());

    pub_marker_->publish(*bag_evaluator->marker());

    pub_trajectory_->publish(*bag_evaluator->trajectory());

    bag_evaluator->show();

    bag_evaluator->clear();
    rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<long>(time_step * 1e9)));
  }

  res->success = true;

  RCLCPP_INFO(get_logger(), "finish.");
}

void OfflineEvaluatorNode::rewind(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  reader_.seek(0);

  res->success = true;

  RCLCPP_INFO(get_logger(), "rewind rosbag.");
}

void OfflineEvaluatorNode::next_route(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  route_handler_->setRoute(*get_route());

  MarkerArray msg;

  autoware_utils::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "preferred_lanes", route_handler_->getPreferredLanelets(),
      create_marker_color(0.16, 1.0, 0.69, 0.2)),
    &msg);

  pub_marker_->publish(msg);

  res->success = true;

  RCLCPP_INFO(get_logger(), "update route.");
}

void OfflineEvaluatorNode::weight(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "start weight grid seach.");

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  stop_watch.tic("total_time");

  reader_.seek(0);
  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  std::vector<Result> weight_grid;

  const auto resolution =
    autoware_utils::get_or_declare_parameter<double>(*this, "grid_seach.grid_step");
  const auto min = autoware_utils::get_or_declare_parameter<double>(*this, "grid_seach.min");
  const auto max = autoware_utils::get_or_declare_parameter<double>(*this, "grid_seach.max");
  for (double w0 = min; w0 < max + 0.1 * resolution; w0 += resolution) {
    for (double w1 = min; w1 < max + 0.1 * resolution; w1 += resolution) {
      for (double w2 = min; w2 < max + 0.1 * resolution; w2 += resolution) {
        for (double w3 = min; w3 < max + 0.1 * resolution; w3 += resolution) {
          for (double w4 = min; w4 < max + 0.1 * resolution; w4 += resolution) {
            for (double w5 = min; w5 < max + 0.1 * resolution; w5 += resolution) {
              weight_grid.emplace_back(w0, w1, w2, w3, w4, w5);
            }
          }
        }
      }
    }
  }

  const auto show_best_result = [this, &weight_grid]() {
    auto sort_by_loss = weight_grid;
    std::sort(sort_by_loss.begin(), sort_by_loss.end(), [](const auto & a, const auto & b) {
      return a.loss < b.loss;
    });

    const auto best = sort_by_loss.front();

    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < best.weight.size(); i++) {
      ss << " [w" << i << "]:" << best.weight.at(i);
    }
    ss << " [loss]:" << best.loss << std::endl;
    RCLCPP_INFO_STREAM(get_logger(), ss.str());
  };

  const auto time_step =
    autoware_utils::get_or_declare_parameter<double>(*this, "grid_seach.time_step");

  const auto bag_evaluator =
    std::make_shared<BagEvaluator>(route_handler_, vehicle_info_, data_augument_parameters());

  const auto metrics = get_or_declare_parameter<std::vector<std::string>>(*this, "metrics");
  for (size_t i = 0; i < metrics.size(); i++) {
    bag_evaluator->load_metric(metrics.at(i), i, data_augument_parameters()->resolution);
  }

  // start grid search
  while (reader_.has_next() && rclcpp::ok()) {
    stop_watch.tic("one_step");
    update(bag_data, time_step);

    if (!bag_data->ready()) break;

    std::mutex g_mutex;
    std::mutex e_mutex;

    const auto update = [&bag_data, &bag_evaluator, &metrics, &weight_grid, &g_mutex,
                         &e_mutex](const auto idx) {
      // TODO(satoshi-ota): remove hard code param
      const auto selector_parameters = std::make_shared<EvaluatorParameters>(6, 20);

      double loss = 0.0;

      std::shared_ptr<TrajectoryPoints> previous_points;
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (idx + 1 > weight_grid.size()) return;
        selector_parameters->score_weight = weight_grid.at(idx).weight;
        selector_parameters->time_decay_weight = std::vector<std::vector<double>>(
          metrics.size(), {1.0, 0.8, 0.64, 0.51, 0.41, 0.33, 0.26, 0.21, 0.17, 0.13});
        previous_points = weight_grid.at(idx).previous_points;
      }

      std::shared_ptr<TrajectoryPoints> selected_points;
      {
        std::lock_guard<std::mutex> lock(e_mutex);
        bag_evaluator->setup(bag_data, previous_points);
        std::tie(loss, selected_points) = bag_evaluator->loss(selector_parameters);
        bag_evaluator->clear();
      }

      {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (idx < weight_grid.size()) {
          weight_grid.at(idx).loss += loss;
          weight_grid.at(idx).previous_points = selected_points;
        }
      }
    };

    // TODO(satoshi-ota): use multithread
    // size_t i = 0;
    // while (rclcpp::ok()) {
    //   std::vector<std::thread> threads;
    //   for (size_t thread_id = 0; thread_id < thread_num; thread_id++) {
    //     threads.emplace_back(update, i + thread_id);
    //   }
    //   for (auto & t : threads) t.join();
    //   if (i + 1 >= weight_grid.size()) break;
    //   i += thread_num;
    // }

    for (size_t i = 0; i < weight_grid.size(); i++) {
      update(i);
    }

    show_best_result();

    RCLCPP_INFO_STREAM(
      get_logger(), "it took " << stop_watch.toc("one_step") << "[ms] to search grid for "
                               << time_step << "[s] bag.");
  }

  res->success = true;

  RCLCPP_INFO_STREAM(
    get_logger(),
    "finish weight grid search. processing time:" << stop_watch.toc("total_time") << "[ms]");
}

void OfflineEvaluatorNode::create_dataset(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "Starting dataset creation...");
  route_handler_->setRoute(*get_route());

  reader_.seek(0);
  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(reader_.get_metadata().starting_time.time_since_epoch()).count());

  const auto bag_evaluator =
    std::make_shared<BagEvaluator>(route_handler_, vehicle_info_, data_augument_parameters());

  auto bag_path = get_or_declare_parameter<std::string>(*this, "bag_path");
  std::string output_file = bag_path;
  if (output_file.find_last_of('.') != std::string::npos) {
    output_file = output_file.substr(0, output_file.find_last_of('.'));
  }
  output_file += ".csv";
  std::ofstream ofs(output_file);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s for writing.", output_file.c_str());
    res->success = false;
    return;
  }

  const auto metrics = get_or_declare_parameter<std::vector<std::string>>(*this, "metrics");
  for (size_t i = 0; i < metrics.size(); i++) {
    bag_evaluator->load_metric(metrics.at(i), i, data_augument_parameters()->resolution);
  }

  ofs << "Tag,Timestamp,";
  for (size_t i = 0; i < data_augument_parameters()->sample_num; ++i) {
    ofs << "x" << i << ",y" << i << ",yaw" << i << ",";
    for (const auto & metric : metrics) {
      ofs << metric.c_str() << ",";
    }
  }
  ofs << std::endl;

  std::shared_ptr<TrajectoryPoints> previous_points{nullptr};

  while (reader_.has_next() && rclcpp::ok()) {
    const bool write = previous_points != nullptr;
    update(bag_data, 0.1);  // to-do(go-sakayori): fix hard code
    if (!bag_data->ready()) break;

    bag_evaluator->setup(bag_data, previous_points, false);
    const auto results = bag_evaluator->calc_metric_values(metrics.size(), previous_points);

    if (results.size() != 2) {
      RCLCPP_INFO(get_logger(), "Data creation failed");
      res->success = false;
      return;
    }

    if (write) {
      for (const auto & result : results) {
        ofs << result.tag << "," << result.time << ",";
        for (const auto & point : result.points) {
          ofs << point.point.pose.position.x << "," << point.point.pose.position.y << ","
              << point.point.pose.orientation.z << ",";
          for (const auto & metric : point.metrics) {
            ofs << metric << ",";
          }
        }
        ofs << std::endl;
      }
    }
    bag_evaluator->clear();
  }

  res->success = true;
  RCLCPP_INFO(get_logger(), "Finish dataset creation.");
}
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode)
