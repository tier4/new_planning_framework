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

#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <rmw/rmw.h>
#include <rosbag2_storage/topic_metadata.hpp>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
using autoware_utils::create_marker_color;
using autoware_utils::get_or_declare_parameter;
using autoware_utils::Polygon2d;

OfflineEvaluatorNode::OfflineEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("offline_evaluator_node", node_options),
  route_handler_{std::make_shared<RouteHandler>()},
  vehicle_info_{std::make_shared<VehicleInfo>(
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())}
{
  setup_publishers();

  setup_evaluation_bag_writer();

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr msg) { route_handler_->setMap(*msg); });

  sub_live_trajectory_ = create_subscription<Trajectory>(
    "~/input/live_trajectory", rclcpp::QoS(10),
    std::bind(&OfflineEvaluatorNode::on_live_trajectory, this, std::placeholders::_1));

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

  replay_reader_.open(get_or_declare_parameter<std::string>(*this, "replay_bag_path"));
  route_reader_.open(get_or_declare_parameter<std::string>(*this, "route_bag_path"));
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

void OfflineEvaluatorNode::setup_publishers()
{
  // Read publisher configuration from parameters
  const auto topic_configs = {
    std::make_tuple("route", TOPIC::ROUTE),
    std::make_tuple("tf", TOPIC::TF),
    std::make_tuple("objects", TOPIC::OBJECTS),
    std::make_tuple("odometry", TOPIC::ODOMETRY),
    std::make_tuple("acceleration", TOPIC::ACCELERATION),
    std::make_tuple("steering", TOPIC::STEERING),
    std::make_tuple("markers", "~/output/markers")
  };

  for (const auto& [config_name, default_topic] : topic_configs) {
    const auto param_prefix = "publish_topics." + config_name;
    
    try {
      const auto enabled = get_or_declare_parameter<bool>(*this, param_prefix + ".enabled");
      const auto publish_once = get_or_declare_parameter<bool>(*this, param_prefix + ".publish_once");
      const auto topic = get_or_declare_parameter<std::string>(*this, param_prefix + ".topic");
      const auto qos = get_or_declare_parameter<int>(*this, param_prefix + ".qos");

      publish_enabled_[config_name] = enabled;
      publish_once_[config_name] = publish_once;

      if (enabled) {
        if (config_name == "route") {
          publishers_[config_name] = create_publisher<LaneletRoute>(topic, rclcpp::QoS(qos));
        } else if (config_name == "tf") {
          publishers_[config_name] = create_publisher<TFMessage>(topic, rclcpp::QoS(qos));
        } else if (config_name == "objects") {
          publishers_[config_name] = create_publisher<PredictedObjects>(topic, rclcpp::QoS(qos));
        } else if (config_name == "odometry") {
          publishers_[config_name] = create_publisher<Odometry>(topic, rclcpp::QoS(qos));
        } else if (config_name == "acceleration") {
          publishers_[config_name] = create_publisher<AccelWithCovarianceStamped>(topic, rclcpp::QoS(qos));
        } else if (config_name == "steering") {
          publishers_[config_name] = create_publisher<SteeringReport>(topic, rclcpp::QoS(qos));
        } else if (config_name == "markers") {
          publishers_[config_name] = create_publisher<MarkerArray>(topic, rclcpp::QoS(qos));
        }

        RCLCPP_INFO(get_logger(), "Created publisher for %s: %s (QoS: %d, publish_once: %s)", 
                    config_name.c_str(), topic.c_str(), qos, publish_once ? "true" : "false");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to setup publisher for %s: %s", config_name.c_str(), e.what());
    }
  }
}

void OfflineEvaluatorNode::setup_evaluation_bag_writer()
{
  try {
    evaluation_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    
    // Get output bag path from parameters
    const auto output_bag_path = get_or_declare_parameter<std::string>(
      *this, "evaluation_output_bag_path");
    
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
    
    // Create topic for displacement errors
    const auto topic_info = rosbag2_storage::TopicMetadata{
      "/trajectory_evaluation/displacement_errors",
      "autoware_new_planning_msgs/msg/TrajectoryDisplacementError",
      rmw_get_serialization_format(),
      ""
    };
    
    evaluation_bag_writer_->create_topic(topic_info);
    
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
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back("/planning/mission_planning/route");
  replay_reader_.set_filter(filter);

  if (!replay_reader_.has_next()) {
    throw std::domain_error("not found route msg.");
  }

  rclcpp::Serialization<LaneletRoute> serializer;

  const auto deserialized_message = std::make_shared<LaneletRoute>();
  while (replay_reader_.has_next()) {
    const auto next_data = replay_reader_.read_next();
    if (next_data->topic_name == TOPIC::ROUTE) {
      rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      break;
    }
  }

  return deserialized_message;
}

auto OfflineEvaluatorNode::get_route_from_bag() -> LaneletRoute::ConstSharedPtr
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back("/planning/mission_planning/route");
  route_reader_.set_filter(filter);

  if (!route_reader_.has_next()) {
    throw std::domain_error("not found route msg in route bag.");
  }

  rclcpp::Serialization<LaneletRoute> serializer;

  const auto deserialized_message = std::make_shared<LaneletRoute>();
  while (route_reader_.has_next()) {
    const auto next_data = route_reader_.read_next();
    if (next_data->topic_name == TOPIC::ROUTE) {
      rclcpp::SerializedMessage serialized_msg(*next_data->serialized_data);
      serializer.deserialize_message(&serialized_msg, deserialized_message.get());
      break;
    }
  }

  return deserialized_message;
}

void OfflineEvaluatorNode::on_live_trajectory(const Trajectory::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_replay_data_) {
    current_replay_data_->append_live_trajectory(*msg);
  }
}

void OfflineEvaluatorNode::update_replay_data(const std::shared_ptr<ReplayEvaluationData> & replay_data, const double dt) const
{
  rosbag2_storage::StorageFilter filter;
  filter.topics.emplace_back(TOPIC::TF);
  filter.topics.emplace_back(TOPIC::ODOMETRY);
  filter.topics.emplace_back(TOPIC::ACCELERATION);
  filter.topics.emplace_back(TOPIC::OBJECTS);
  filter.topics.emplace_back(TOPIC::STEERING);
  replay_reader_.set_filter(filter);

  replay_data->update(dt * 1e9);

  while (replay_reader_.has_next()) {
    const auto next_data = replay_reader_.read_next();
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

void OfflineEvaluatorNode::publish_replay_topics(const std::shared_ptr<ReplayEvaluationData> & replay_data) const
{
  // Publish topics based on configuration (excluding publish_once topics)
  
  if (publish_enabled_.count("tf") && publish_enabled_.at("tf") && !publish_once_.at("tf")) {
    const auto tf_msg = std::dynamic_pointer_cast<Buffer<TFMessage>>(
      replay_data->buffers.at(TOPIC::TF))->get(replay_data->timestamp);
    if (tf_msg && publishers_.count("tf")) {
      std::static_pointer_cast<rclcpp::Publisher<TFMessage>>(publishers_.at("tf"))->publish(*tf_msg);
    }
  }

  if (publish_enabled_.count("objects") && publish_enabled_.at("objects") && !publish_once_.at("objects")) {
    const auto objects_msg = std::dynamic_pointer_cast<Buffer<PredictedObjects>>(
      replay_data->buffers.at(TOPIC::OBJECTS))->get(replay_data->timestamp);
    if (objects_msg && publishers_.count("objects")) {
      std::static_pointer_cast<rclcpp::Publisher<PredictedObjects>>(publishers_.at("objects"))->publish(*objects_msg);
    }
  }

  if (publish_enabled_.count("odometry") && publish_enabled_.at("odometry") && !publish_once_.at("odometry")) {
    const auto odometry_msg = std::dynamic_pointer_cast<Buffer<Odometry>>(
      replay_data->buffers.at(TOPIC::ODOMETRY))->get(replay_data->timestamp);
    if (odometry_msg && publishers_.count("odometry")) {
      std::static_pointer_cast<rclcpp::Publisher<Odometry>>(publishers_.at("odometry"))->publish(*odometry_msg);
    }
  }

  if (publish_enabled_.count("acceleration") && publish_enabled_.at("acceleration") && !publish_once_.at("acceleration")) {
    const auto accel_msg = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
      replay_data->buffers.at(TOPIC::ACCELERATION))->get(replay_data->timestamp);
    if (accel_msg && publishers_.count("acceleration")) {
      std::static_pointer_cast<rclcpp::Publisher<AccelWithCovarianceStamped>>(publishers_.at("acceleration"))->publish(*accel_msg);
    }
  }

  if (publish_enabled_.count("steering") && publish_enabled_.at("steering") && !publish_once_.at("steering")) {
    const auto steering_msg = std::dynamic_pointer_cast<Buffer<SteeringReport>>(
      replay_data->buffers.at(TOPIC::STEERING))->get(replay_data->timestamp);
    if (steering_msg && publishers_.count("steering")) {
      std::static_pointer_cast<rclcpp::Publisher<SteeringReport>>(publishers_.at("steering"))->publish(*steering_msg);
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
  replay_reader_.set_filter(filter);

  bag_data->update(dt * 1e9);

  while (replay_reader_.has_next()) {
    const auto next_data = replay_reader_.read_next();
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

void OfflineEvaluatorNode::play(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Initialize replay data with ReplayEvaluationData instead of BagData
  current_replay_data_ = std::make_shared<ReplayEvaluationData>(
    duration_cast<nanoseconds>(replay_reader_.get_metadata().starting_time.time_since_epoch()).count());

  // First, read and publish route from route bag - this must happen before replay starts
  if (publish_enabled_.count("route") && publish_enabled_.at("route")) {
    try {
      const auto route = get_route_from_bag();
      route_handler_->setRoute(*route);
      if (publishers_.count("route")) {
        std::static_pointer_cast<rclcpp::Publisher<LaneletRoute>>(publishers_.at("route"))->publish(*route);
        RCLCPP_INFO(get_logger(), "Published route from route bag, waiting for processing...");
        
        // Wait a bit for route to be processed by other nodes
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to read route: %s", e.what());
      res->success = false;
      return;
    }
  }

  const auto time_step = get_or_declare_parameter<double>(*this, "play.time_step");

  RCLCPP_INFO(get_logger(), "Starting rosbag replay now...");

  std::shared_ptr<TrajectoryPoints> previous_points{nullptr};

  const auto bag_evaluator =
    std::make_shared<BagEvaluator>(route_handler_, vehicle_info_, data_augument_parameters());

  const auto metrics = get_or_declare_parameter<std::vector<std::string>>(*this, "metrics");
  for (size_t i = 0; i < metrics.size(); i++) {
    bag_evaluator->load_metric(metrics.at(i), i, data_augument_parameters()->resolution);
  }

  const auto parameters = evaluator_parameters();

  while (replay_reader_.has_next() && rclcpp::ok()) {
    update_replay_data(current_replay_data_, time_step);

    // Cast ReplayEvaluationData to BagData for compatibility with existing evaluator
    auto bag_data = std::static_pointer_cast<BagData>(current_replay_data_);
    bag_evaluator->setup(bag_data, previous_points);

    const auto best_data = bag_evaluator->best(parameters);

    previous_points = best_data == nullptr ? nullptr : best_data->points();

    // Publish all configured topics
    publish_replay_topics(current_replay_data_);

    // Publish markers if enabled
    if (publish_enabled_.count("markers") && publish_enabled_.at("markers") && publishers_.count("markers")) {
      std::static_pointer_cast<rclcpp::Publisher<MarkerArray>>(publishers_.at("markers"))->publish(*bag_evaluator->marker());
    }

    bag_evaluator->show();

    bag_evaluator->clear();

    // Check if we have live trajectory data for evaluation
    if (current_replay_data_->live_trajectory_ready()) {
      const auto live_trajectory = current_replay_data_->get_live_trajectory(current_replay_data_->timestamp);
      if (live_trajectory) {
        RCLCPP_INFO(get_logger(), "Evaluating live trajectory with %zu points", live_trajectory->points.size());
        
        // Generate ground truth trajectory from localization data
        try {
          const auto ground_truth_trajectory = bag_evaluator->ground_truth_from_live_trajectory(
            current_replay_data_, *live_trajectory);
          
          if (ground_truth_trajectory && !ground_truth_trajectory->empty()) {
            RCLCPP_INFO(get_logger(), "Generated ground truth trajectory with %zu points", 
                       ground_truth_trajectory->size());
            
            // Convert live trajectory to TrajectoryPoints format
            const auto candidate_trajectory = convert_trajectory_to_points(*live_trajectory);
            
            // Calculate displacement errors (ADE, FDE, etc.)
            const auto displacement_errors = bag_evaluator->calculate_displacement_errors(
              candidate_trajectory, ground_truth_trajectory);
            
            // Log evaluation results
            RCLCPP_INFO(get_logger(), 
              "Displacement errors - ADE: %.3f, FDE: %.3f, Max: %.3f, Min: %.3f",
              displacement_errors.average_displacement_error,
              displacement_errors.final_displacement_error,
              displacement_errors.max_displacement_error,
              displacement_errors.min_displacement_error);
            
            // Store results in evaluation bag
            if (evaluation_bag_writer_) {
              try {
                // Serialize the message
                rclcpp::Serialization<autoware_new_planning_msgs::msg::TrajectoryDisplacementError> serializer;
                rclcpp::SerializedMessage serialized_msg;
                serializer.serialize_message(&displacement_errors, &serialized_msg);
                
                // Write to bag with current timestamp
                evaluation_bag_writer_->write(
                  serialized_msg,
                  "/trajectory_evaluation/displacement_errors",
                  rclcpp::Clock().now()
                );
                
                RCLCPP_DEBUG(get_logger(), "Stored displacement errors in evaluation bag");
              } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to store displacement errors: %s", e.what());
              }
            }
          } else {
            RCLCPP_WARN(get_logger(), "Failed to generate ground truth trajectory");
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Error generating ground truth: %s", e.what());
        }
      }
    }
  }

  res->success = true;
  current_replay_data_ = nullptr;

  RCLCPP_INFO(get_logger(), "finish.");
}

void OfflineEvaluatorNode::rewind(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  replay_reader_.seek(0);
  route_reader_.seek(0);

  res->success = true;

  RCLCPP_INFO(get_logger(), "rewind rosbags.");
}

void OfflineEvaluatorNode::next_route(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    const auto route = get_route_from_bag();
    route_handler_->setRoute(*route);
    pub_route_->publish(*route);

    MarkerArray msg;

    autoware_utils::append_marker_array(
      lanelet::visualization::laneletsAsTriangleMarkerArray(
        "preferred_lanes", route_handler_->getPreferredLanelets(),
        create_marker_color(0.16, 1.0, 0.69, 0.2)),
      &msg);

    pub_marker_->publish(msg);

    res->success = true;

    RCLCPP_INFO(get_logger(), "update route from route bag.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to update route: %s", e.what());
    res->success = false;
  }
}

void OfflineEvaluatorNode::weight(
  [[maybe_unused]] const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "start weight grid seach.");

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  stop_watch.tic("total_time");

  replay_reader_.seek(0);
  const auto bag_data = std::make_shared<BagData>(
    duration_cast<nanoseconds>(replay_reader_.get_metadata().starting_time.time_since_epoch()).count());

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
  while (replay_reader_.has_next() && rclcpp::ok()) {
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
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode)
