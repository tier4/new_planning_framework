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

#include <rmw/rmw.h>
#include <rosbag2_storage/topic_metadata.hpp>

#include <filesystem>
#include <fstream>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
using autoware_utils::get_or_declare_parameter;

// Helper function to get parameter with default value
template<typename T>
T get_parameter_or_default(rclcpp::Node& node, const std::string& name, const T& default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, default_value);
}

OfflineEvaluatorNode::OfflineEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("offline_evaluator_node", node_options),
  route_handler_{std::make_shared<RouteHandler>()},
  vehicle_info_{std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())}
{
  setup_evaluation_bag_writer();

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  srv_route_ = this->create_service<Trigger>(
    "next_route",
    std::bind(
      &OfflineEvaluatorNode::next_route, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  // Open bag file
  const auto bag_path = get_or_declare_parameter<std::string>(*this, "bag_path");
  try {
    bag_reader_.open(bag_path);
    RCLCPP_INFO(get_logger(), "Successfully opened bag file: %s", bag_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to open bag file: %s", e.what());
    throw;
  }
  
  // Initialize topic names from parameters with defaults
  route_topic_name_ = get_parameter_or_default<std::string>(*this, "route_topic", "/planning/mission_planning/route");
  odometry_topic_name_ = get_parameter_or_default<std::string>(*this, "odometry_topic", "/localization/kinematic_state");
  trajectory_topic_name_ = get_parameter_or_default<std::string>(*this, "trajectory_topic", "/planning/scenario_planning/trajectory");
  objects_topic_name_ = get_parameter_or_default<std::string>(*this, "objects_topic", "/perception/object_recognition/objects");
  tf_topic_name_ = get_parameter_or_default<std::string>(*this, "tf_topic", "/tf");
  acceleration_topic_name_ = get_parameter_or_default<std::string>(*this, "acceleration_topic", "/localization/acceleration");
  steering_topic_name_ = get_parameter_or_default<std::string>(*this, "steering_topic", "/vehicle/status/steering_status");
  
  // Update global TOPIC constants with configured values
  TOPIC::ROUTE = route_topic_name_;
  TOPIC::ODOMETRY = odometry_topic_name_;
  TOPIC::TRAJECTORY = trajectory_topic_name_;
  TOPIC::OBJECTS = objects_topic_name_;
  TOPIC::TF = tf_topic_name_;
  TOPIC::ACCELERATION = acceleration_topic_name_;
  TOPIC::STEERING = steering_topic_name_;
}

OfflineEvaluatorNode::~OfflineEvaluatorNode()
{
  if (evaluation_bag_writer_) {
    try {
      evaluation_bag_writer_->close();
      RCLCPP_INFO(get_logger(), "Evaluation bag writer closed successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error closing evaluation bag writer: %s", e.what());
    }
  }
}


void OfflineEvaluatorNode::setup_evaluation_bag_writer()
{
  try {
    evaluation_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    
    // Get output bag path from parameters with default
    const auto output_bag_path = get_parameter_or_default<std::string>(
      *this, "evaluation_output_bag_path", "/tmp/trajectory_evaluation_results.bag");
    
    // Ensure directory exists
    const auto output_dir = std::filesystem::path(output_bag_path).parent_path();
    if (!output_dir.empty() && !std::filesystem::exists(output_dir)) {
      std::filesystem::create_directories(output_dir);
      RCLCPP_INFO(get_logger(), "Created output directory: %s", output_dir.string().c_str());
    }
    
    // Validate write permissions
    if (!output_dir.empty()) {
      const auto test_file = output_dir / ".write_test";
      std::ofstream test_stream(test_file);
      if (!test_stream.is_open()) {
        throw std::runtime_error("No write permission to output directory: " + output_dir.string());
      }
      test_stream.close();
      std::filesystem::remove(test_file);
    }
    
    // Setup bag writer
    const rosbag2_storage::StorageOptions storage_options{
      output_bag_path,
      "sqlite3"
    };
    
    const rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(),
      rmw_get_serialization_format()
    };
    
    evaluation_bag_writer_->open(storage_options, converter_options);
    
    // Create topics for evaluation results
    const std::vector<std::pair<std::string, std::string>> topics = {
      {"/evaluation/ttc", "std_msgs/msg/Float64"},
      {"/evaluation/displacement_errors", "geometry_msgs/msg/Point"},
      {"/evaluation/speed_violations", "geometry_msgs/msg/Point"},
      {"/evaluation/lane_violations", "geometry_msgs/msg/Point"},
      {"/evaluation/comfort_metrics", "geometry_msgs/msg/Point"},
      {"/evaluation/status", "diagnostic_msgs/msg/DiagnosticStatus"}
    };
    
    for (const auto& [topic_name, topic_type] : topics) {
      const auto topic_info = rosbag2_storage::TopicMetadata{
        topic_name,
        topic_type,
        rmw_get_serialization_format(),
        ""
      };
      evaluation_bag_writer_->create_topic(topic_info);
    }
    
    RCLCPP_INFO(get_logger(), "Evaluation bag writer initialized: %s", output_bag_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to setup evaluation bag writer: %s", e.what());
    evaluation_bag_writer_ = nullptr;
  }
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
  // Reset reader to beginning to find route
  const auto starting_time = bag_reader_.get_metadata().starting_time;
  const auto starting_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    starting_time.time_since_epoch()).count();
  bag_reader_.seek(starting_time_ns);
  
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(route_topic_name_);
  bag_reader_.set_filter(filter);

  if (!bag_reader_.has_next()) {
    // Provide more helpful error message
    std::stringstream ss;
    ss << "Route topic '" << route_topic_name_ << "' not found in bag file.\n";
    ss << "Available topics in bag:\n";
    const auto topics = bag_reader_.get_all_topics_and_types();
    for (const auto& topic : topics) {
      ss << "  - " << topic.name << " (" << topic.type << ")\n";
    }
    throw std::domain_error(ss.str());
  }

  rclcpp::Serialization<LaneletRoute> serializer;

  const auto deserialized_message = std::make_shared<LaneletRoute>();
  bool route_found = false;
  int messages_checked = 0;
  const int max_messages_to_check = 1000; // Prevent infinite loop
  
  while (bag_reader_.has_next() && messages_checked < max_messages_to_check) {
    const auto next_data = bag_reader_.read_next();
    messages_checked++;
    
    if (next_data->topic_name == route_topic_name_) {
      try {
        rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);
        serializer.deserialize_message(&serialized_msg, deserialized_message.get());
        route_found = true;
        break;
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize route message: %s", e.what());
        continue;
      }
    }
  }

  if (!route_found) {
    throw std::domain_error("Route message not found in first " + std::to_string(max_messages_to_check) + " messages of bag.");
  }
  
  if (deserialized_message->segments.empty()) {
    throw std::domain_error("Route message found but contains no segments.");
  }

  RCLCPP_INFO(get_logger(), "Successfully loaded route with %zu segments", deserialized_message->segments.size());
  return deserialized_message;
}


// Removed unused on_live_trajectory function

void OfflineEvaluatorNode::update_replay_data(const std::shared_ptr<ReplayEvaluationData> & replay_data, const double dt) const
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(TOPIC::TF);
  filter.topics.emplace_back(TOPIC::ODOMETRY);
  filter.topics.emplace_back(TOPIC::ACCELERATION);
  filter.topics.emplace_back(TOPIC::OBJECTS);
  filter.topics.emplace_back(TOPIC::STEERING);
  bag_reader_.set_filter(filter);

  replay_data->update(dt * 1e9);

  while (bag_reader_.has_next()) {
    const auto next_data = bag_reader_.read_next();
    rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);

    if (replay_data->ready()) {
      break;
    }

    if (next_data->topic_name == TOPIC::TF) {
      rclcpp::Serialization<TFMessage> serializer;
      const auto deserialized_message = std::make_shared<TFMessage>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<TFMessage>>(replay_data->buffers.at(TOPIC::TF))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::ODOMETRY) {
      rclcpp::Serialization<Odometry> serializer;
      const auto deserialized_message = std::make_shared<Odometry>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<Odometry>>(replay_data->buffers.at(TOPIC::ODOMETRY))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::ACCELERATION) {
      rclcpp::Serialization<AccelWithCovarianceStamped> serializer;
      const auto deserialized_message = std::make_shared<AccelWithCovarianceStamped>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
        replay_data->buffers.at(TOPIC::ACCELERATION))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::OBJECTS) {
      rclcpp::Serialization<PredictedObjects> serializer;
      const auto deserialized_message = std::make_shared<PredictedObjects>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<PredictedObjects>>(replay_data->buffers.at(TOPIC::OBJECTS))
        ->append(*deserialized_message);
    }

    if (next_data->topic_name == TOPIC::STEERING) {
      rclcpp::Serialization<SteeringReport> serializer;
      const auto deserialized_message = std::make_shared<SteeringReport>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<SteeringReport>>(replay_data->buffers.at(TOPIC::STEERING))
        ->append(*deserialized_message);
    }
  }
}


auto OfflineEvaluatorNode::convert_trajectory_to_points(const Trajectory & trajectory) const 
  -> std::shared_ptr<TrajectoryPoints>
{
  auto trajectory_points = std::make_shared<TrajectoryPoints>();
  trajectory_points->reserve(trajectory.points.size());
  
  for (const auto & point : trajectory.points) {
    trajectory_points->push_back(point);
  }
  
  return trajectory_points;
}

void OfflineEvaluatorNode::update(const std::shared_ptr<BagData> & bag_data, const double dt) const
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(TOPIC::TF);
  filter.topics.emplace_back(TOPIC::ODOMETRY);
  filter.topics.emplace_back(TOPIC::ACCELERATION);
  filter.topics.emplace_back(TOPIC::OBJECTS);
  filter.topics.emplace_back(TOPIC::STEERING);
  filter.topics.emplace_back(TOPIC::TRAJECTORY);
  bag_reader_.set_filter(filter);

  bag_data->update(dt * 1e9);

  while (bag_reader_.has_next()) {
    const auto next_data = bag_reader_.read_next();
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
      rclcpp::Serialization<Trajectory> serializer;
      const auto deserialized_message = std::make_shared<Trajectory>();
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers.at(TOPIC::TRAJECTORY))
        ->append(*deserialized_message);
    }
  }
}

// Removed unused play function

void OfflineEvaluatorNode::next_route(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    const auto route = get_route();
    route_handler_->setRoute(*route);

    res->success = true;

    RCLCPP_INFO(get_logger(), "update route from bag.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to update route: %s", e.what());
    res->success = false;
  }
}

// Removed unused validation functions
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode)
