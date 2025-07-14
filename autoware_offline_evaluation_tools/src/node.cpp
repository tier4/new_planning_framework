// Copyright 2025 TIER IV, Inc.
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

#include "closed_loop_evaluator.hpp"
#include "open_loop_evaluator.hpp"

#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <fstream>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <rmw/rmw.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace autoware::trajectory_selector::offline_evaluation_tools
{
using autoware_utils::create_marker_color;
using autoware_utils_rclcpp::get_or_declare_parameter;

// Helper function to get parameter with default value
template <typename T>
T get_parameter_or_default(rclcpp::Node & node, const std::string & name, const T & default_value)
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

  sub_map_marker_ = create_subscription<MarkerArray>(
    "~/input/lanelet2_map_marker", rclcpp::QoS{1}.transient_local(),
    [this](const MarkerArray::ConstSharedPtr msg) { map_marker_ = msg; });

  // Open bag file
  bag_path_ = get_or_declare_parameter<std::string>(*this, "bag_path");
  try {
    bag_reader_.open(bag_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to open bag file: %s", e.what());
    throw;
  }

  // Initialize topic names from parameters with defaults
  route_topic_name_ =
    get_parameter_or_default<std::string>(*this, "route_topic", "/planning/mission_planning/route");
  odometry_topic_name_ =
    get_parameter_or_default<std::string>(*this, "odometry_topic", "/localization/kinematic_state");
  trajectory_topic_name_ = get_parameter_or_default<std::string>(
    *this, "trajectory_topic", "/planning/scenario_planning/lane_driving/trajectory");
  objects_topic_name_ = get_parameter_or_default<std::string>(
    *this, "objects_topic", "/perception/object_recognition/objects");
  tf_topic_name_ = get_parameter_or_default<std::string>(*this, "tf_topic", "/tf");
  acceleration_topic_name_ = get_parameter_or_default<std::string>(
    *this, "acceleration_topic", "/localization/acceleration");
  steering_topic_name_ = get_parameter_or_default<std::string>(
    *this, "steering_topic", "/vehicle/status/steering_status");

  // Update global TOPIC constants with configured values
  TOPIC::ROUTE = route_topic_name_;
  TOPIC::ODOMETRY = odometry_topic_name_;
  TOPIC::TRAJECTORY = trajectory_topic_name_;
  TOPIC::OBJECTS = objects_topic_name_;
  TOPIC::TF = tf_topic_name_;
  TOPIC::ACCELERATION = acceleration_topic_name_;
  TOPIC::STEERING = steering_topic_name_;

  // Read evaluation mode
  const auto mode_str =
    get_parameter_or_default<std::string>(*this, "evaluation.mode", "closed_loop");
  if (mode_str == "open_loop") {
    evaluation_mode_ = EvaluationMode::OPEN_LOOP;
  } else if (mode_str == "closed_loop") {
    evaluation_mode_ = EvaluationMode::CLOSED_LOOP;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid evaluation mode: %s. Using CLOSED_LOOP.", mode_str.c_str());
    evaluation_mode_ = EvaluationMode::CLOSED_LOOP;
  }

  // Create a timer to check map readiness without blocking
  map_check_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    if (route_handler_->isMapMsgReady() && map_marker_) {
      map_check_timer_->cancel();  // Stop checking
      run_evaluation();
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for map to be ready...");
    }
  });
}

OfflineEvaluatorNode::~OfflineEvaluatorNode()
{
  if (evaluation_bag_writer_) {
    try {
      evaluation_bag_writer_->close();
      RCLCPP_INFO(get_logger(), "Evaluation bag writer closed successfully");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error closing evaluation bag writer: %s", e.what());
    }
  }
}

void OfflineEvaluatorNode::setup_evaluation_bag_writer()
{
  try {
    evaluation_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Get output bag path from parameters with default
    auto output_bag_path = get_parameter_or_default<std::string>(
      *this, "evaluation_output_bag_path", "~/trajectory_evaluation_results.bag");

    // Expand home directory if needed
    if (output_bag_path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        output_bag_path = std::string(home) + output_bag_path.substr(1);
      }
    }

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

    // Setup bag writer with MCAP format
    const rosbag2_storage::StorageOptions storage_options{output_bag_path, "mcap"};

    const rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    evaluation_bag_writer_->open(storage_options, converter_options);

    // Create topics for evaluation results
    const std::vector<std::pair<std::string, std::string>> topics = {
      {"/evaluation/lateral_error", "std_msgs/msg/Float64"},
      {"/evaluation/ttc", "std_msgs/msg/Float64"},
      {"/evaluation/acceleration_metrics", "geometry_msgs/msg/PointStamped"},
      {"/evaluation/oscillation_metrics", "geometry_msgs/msg/PointStamped"},
      {"/evaluation/yaw_rate", "std_msgs/msg/Float64"},
      {"/evaluation/map_markers", "visualization_msgs/msg/MarkerArray"},
      {"/evaluation/route_markers", "visualization_msgs/msg/MarkerArray"},
      {"/evaluation/localization", "nav_msgs/msg/Odometry"},
      {"/evaluation/objects", "autoware_perception_msgs/msg/PredictedObjects"},
      {"/tf_static", "tf2_msgs/msg/TFMessage"},
      {"/tf", "tf2_msgs/msg/TFMessage"}};

    for (const auto & [topic_name, topic_type] : topics) {
      const auto topic_info =
        rosbag2_storage::TopicMetadata{topic_name, topic_type, rmw_get_serialization_format(), ""};
      evaluation_bag_writer_->create_topic(topic_info);
    }

    RCLCPP_INFO(get_logger(), "Evaluation bag writer initialized: %s", output_bag_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to setup evaluation bag writer: %s", e.what());
    evaluation_bag_writer_ = nullptr;
  }
}

void OfflineEvaluatorNode::run_evaluation()
{
  RCLCPP_INFO(get_logger(), "Starting evaluation...");

  // Storage for the last route message and tf messages
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr last_route_msg = nullptr;
  tf2_msgs::msg::TFMessage tf_static_msg;
  std::vector<std::pair<tf2_msgs::msg::TFMessage, rclcpp::Time>> tf_messages;

  // Quick scan for route and tf messages
  while (bag_reader_.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader_.read_next();
    const auto & topic_name = serialized_message->topic_name;

    if (topic_name == route_topic_name_ && !last_route_msg) {
      try {
        autoware_planning_msgs::msg::LaneletRoute msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::LaneletRoute> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        last_route_msg = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(msg);
        RCLCPP_INFO(get_logger(), "Found route message in bag");
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize route message: %s", e.what());
      }
    } else if (topic_name == "/tf_static") {
      try {
        tf2_msgs::msg::TFMessage msg;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        // Accumulate all tf_static transforms
        tf_static_msg.transforms.insert(
          tf_static_msg.transforms.end(), msg.transforms.begin(), msg.transforms.end());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize tf_static message: %s", e.what());
      }
    } else if (topic_name == tf_topic_name_) {
      try {
        tf2_msgs::msg::TFMessage msg;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        // Store tf messages with their timestamps
        rclcpp::Time msg_time(serialized_message->time_stamp);
        tf_messages.emplace_back(msg, msg_time);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to deserialize tf message: %s", e.what());
      }
    }
  }

  if (!last_route_msg) {
    RCLCPP_ERROR(get_logger(), "No route message found in bag");
    // return;
  }
  else {
    route_handler_->setRoute(*last_route_msg);
  }

  // Seek back to the beginning of the bag for mode-specific evaluation
  bag_reader_.seek(0);

  // Store reference times for map/route markers
  rclcpp::Time start_time = now();
  rclcpp::Time end_time = now();

  switch (evaluation_mode_) {
    case EvaluationMode::OPEN_LOOP: {
      auto times = run_open_loop_evaluation();
      start_time = times.first;
      end_time = times.second;
      break;
    }
    case EvaluationMode::CLOSED_LOOP: {
      auto times = run_closed_loop_evaluation();
      start_time = times.first;
      end_time = times.second;
      break;
    }
  }


  // Write tf_static at the beginning if available
  if (!tf_static_msg.transforms.empty() && evaluation_bag_writer_) {
    evaluation_bag_writer_->write(tf_static_msg, "/tf_static", start_time);
    RCLCPP_INFO(
      get_logger(), "Wrote %zu tf_static transforms to evaluation bag",
      tf_static_msg.transforms.size());
  }

  // Write map and route markers at the beginning and end of the rosbag
  write_map_and_route_markers_to_bag(start_time);
  
  // Also write markers at the end time to ensure they're visible throughout the bag
  if (end_time > start_time) {
    write_map_and_route_markers_to_bag(end_time);
  }

  RCLCPP_INFO(get_logger(), "Evaluation complete");
}

std::pair<rclcpp::Time, rclcpp::Time> OfflineEvaluatorNode::run_open_loop_evaluation()
{
  RCLCPP_INFO(get_logger(), "Running open-loop evaluation for trajectory analysis");
  RCLCPP_INFO(get_logger(), "Looking for trajectories on topic: %s", trajectory_topic_name_.c_str());
  
  // Reopen the bag to read from the beginning
  bag_reader_.close();
  bag_reader_.open(bag_path_);
  
  // Create bag data handler
  const auto buffer_duration_sec =
    get_parameter_or_default<double>(*this, "buffer_duration_sec", 20.0);
  const size_t max_buffer_messages =
    static_cast<size_t>(get_parameter_or_default<int64_t>(*this, "max_buffer_messages", 10000));
  
  auto bag_data = std::make_shared<BagData>(buffer_duration_sec, max_buffer_messages);
  
  // Find the time range of the bag
  rclcpp::Time bag_start_time = rclcpp::Time(std::numeric_limits<int64_t>::max());
  rclcpp::Time bag_end_time = rclcpp::Time(0);
  
  // Counters for debugging
  size_t odometry_count = 0;
  size_t trajectory_count = 0;
  size_t objects_count = 0;
  size_t tf_count = 0;
  size_t tf_static_count = 0;
  
  // First pass: scan for time range and collect all data
  while (bag_reader_.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader_.read_next();
    rclcpp::Time msg_time(serialized_message->time_stamp);
    
    if (msg_time < bag_start_time) bag_start_time = msg_time;
    if (msg_time > bag_end_time) bag_end_time = msg_time;
    
    const auto & topic_name = serialized_message->topic_name;
    
    try {
      // Process kinematic state messages
      if (topic_name == odometry_topic_name_) {
        Odometry msg;
        rclcpp::Serialization<Odometry> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<Odometry>>(
          bag_data->buffers[TOPIC::ODOMETRY]);
        if (buffer) {
          buffer->append(msg);
        }
        
        odometry_count++;
        
        // Write to evaluation bag
        if (evaluation_bag_writer_) {
          evaluation_bag_writer_->write(msg, "/evaluation/localization", msg_time);
        }
      }
      // Process trajectory messages
      else if (topic_name == trajectory_topic_name_) {
        Trajectory msg;
        rclcpp::Serialization<Trajectory> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<Trajectory>>(
          bag_data->buffers[TOPIC::TRAJECTORY]);
        if (buffer) {
          buffer->append(msg);
        }
        
        trajectory_count++;
        
        // Write to evaluation bag
        if (evaluation_bag_writer_) {
          evaluation_bag_writer_->write(msg, trajectory_topic_name_, msg_time);
        }
      }
      // Process objects for visualization/analysis
      else if (topic_name == objects_topic_name_) {
        PredictedObjects msg;
        rclcpp::Serialization<PredictedObjects> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<PredictedObjects>>(
          bag_data->buffers[TOPIC::OBJECTS]);
        if (buffer) {
          buffer->append(msg);
        }
        
        objects_count++;
        
        // Write to evaluation bag
        if (evaluation_bag_writer_) {
          evaluation_bag_writer_->write(msg, "/perception/object_recognition/objects", msg_time);
        }
      }
      // Process TF messages
      else if (topic_name == tf_topic_name_) {
        TFMessage msg;
        rclcpp::Serialization<TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers[TOPIC::TF]);
        if (buffer) {
          buffer->append(msg);
        }
        
        tf_count++;
        
        // Write to evaluation bag
        if (evaluation_bag_writer_) {
          evaluation_bag_writer_->write(msg, "/tf", msg_time);
        }
      }
      // Process TF static messages
      else if (topic_name == "/tf_static") {
        TFMessage msg;
        rclcpp::Serialization<TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        tf_static_count++;
        
        // Write to evaluation bag
        if (evaluation_bag_writer_) {
          evaluation_bag_writer_->write(msg, "/tf_static", msg_time);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Failed to deserialize message on topic %s: %s", 
        topic_name.c_str(), e.what());
    }
  }
  
  // Log message counts
  RCLCPP_INFO(get_logger(), "Message counts - Odometry: %zu, Trajectory: %zu, Objects: %zu, TF: %zu, TF_static: %zu",
    odometry_count, trajectory_count, objects_count, tf_count, tf_static_count);
  
  // Get all data points with synchronized localization and trajectory data
  const auto evaluation_interval_ms =
    get_parameter_or_default<double>(*this, "evaluation_interval_ms", 100.0);
  
  // Collect synchronized data for evaluation
  std::vector<std::shared_ptr<SynchronizedData>> synchronized_data_list;
  const double sync_tolerance_ms =
    get_parameter_or_default<double>(*this, "sync_tolerance_ms", 50.0);
  
  // Get all kinematic states at regular intervals
  auto kinematic_states = bag_data->get_kinematic_states_at_interval(evaluation_interval_ms);
  
  if (kinematic_states.empty()) {
    RCLCPP_ERROR(get_logger(), "No kinematic states found in the rosbag");
    return {bag_start_time, bag_end_time};
  }
  
  // For each kinematic state, try to get synchronized data
  for (const auto & kin_state : kinematic_states) {
    const auto timestamp = rclcpp::Time(kin_state->header.stamp).nanoseconds();
    auto sync_data = bag_data->get_synchronized_data_at_time(timestamp, sync_tolerance_ms);
    if (sync_data) {
      synchronized_data_list.push_back(sync_data);
    }
  }
  
  // Sort by timestamp
  std::sort(synchronized_data_list.begin(), synchronized_data_list.end(),
    [](const auto & a, const auto & b) { return a->timestamp < b->timestamp; });
  
  // Run open-loop evaluation
  if (!synchronized_data_list.empty()) {
    OpenLoopEvaluator evaluator(get_logger(), route_handler_);
    evaluator.evaluate(synchronized_data_list, evaluation_bag_writer_.get());
    
    // Get and save evaluation results
    auto summary_json = evaluator.get_summary_as_json();
    auto detailed_json = evaluator.get_detailed_results_as_json();
    
    // Write results to file
    const auto output_dir = get_parameter_or_default<std::string>(*this, "output_dir", ".");
    const auto json_path = output_dir + "/open_loop_evaluation_results.json";
    
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
      json_file << detailed_json.dump(2);
      json_file.close();
      RCLCPP_INFO(get_logger(), "Saved evaluation results to: %s", json_path.c_str());
    }
    
    // Log summary
    RCLCPP_INFO(get_logger(), "Open-loop evaluation summary:");
    RCLCPP_INFO(get_logger(), "  Total trajectories: %d", 
      static_cast<int>(summary_json["total_trajectories"]));
    RCLCPP_INFO(get_logger(), "  Valid trajectories: %d", 
      static_cast<int>(summary_json["valid_trajectories"]));
    if (summary_json.contains("ade") && summary_json["ade"].contains("mean")) {
      RCLCPP_INFO(get_logger(), "  Mean ADE: %.3f m", 
        static_cast<double>(summary_json["ade"]["mean"]));
      RCLCPP_INFO(get_logger(), "  Mean FDE: %.3f m", 
        static_cast<double>(summary_json["fde"]["mean"]));
    }
  }
  
  RCLCPP_INFO(get_logger(), "Open-loop evaluation complete");
  
  // Return the actual time range of the evaluation
  return {bag_start_time, bag_end_time};
}

std::pair<rclcpp::Time, rclcpp::Time> OfflineEvaluatorNode::run_closed_loop_evaluation()
{
  RCLCPP_INFO(get_logger(), "Running closed-loop evaluation for autonomous driving data");

  // Create bag data handler
  const auto buffer_duration_sec =
    get_parameter_or_default<double>(*this, "buffer_duration_sec", 20.0);
  const size_t max_buffer_messages =
    static_cast<size_t>(get_parameter_or_default<int64_t>(*this, "max_buffer_messages", 10000));

  auto bag_data = std::make_shared<BagData>(0, buffer_duration_sec, max_buffer_messages);

  // Read all messages from bag into buffers
  RCLCPP_INFO(get_logger(), "Loading rosbag data into buffers...");

  while (bag_reader_.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader_.read_next();
    const auto & topic_name = serialized_message->topic_name;

    try {
      if (topic_name == odometry_topic_name_) {
        Odometry msg;
        rclcpp::Serialization<Odometry> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);

        auto buffer =
          std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers[TOPIC::ODOMETRY]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == trajectory_topic_name_) {
        Trajectory msg;
        rclcpp::Serialization<Trajectory> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);

        auto buffer =
          std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers[TOPIC::TRAJECTORY]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == acceleration_topic_name_) {
        AccelWithCovarianceStamped msg;
        rclcpp::Serialization<AccelWithCovarianceStamped> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);

        auto buffer = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(
          bag_data->buffers[TOPIC::ACCELERATION]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == steering_topic_name_) {
        SteeringReport msg;
        rclcpp::Serialization<SteeringReport> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);

        auto buffer =
          std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers[TOPIC::STEERING]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == objects_topic_name_) {
        PredictedObjects msg;
        rclcpp::Serialization<PredictedObjects> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);

        auto buffer =
          std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers[TOPIC::OBJECTS]);
        if (buffer) {
          buffer->append(msg);
        }
        
        // Also write objects to evaluation bag
        if (evaluation_bag_writer_) {
          rclcpp::Time msg_time(serialized_message->time_stamp);
          evaluation_bag_writer_->write(msg, "/evaluation/objects", msg_time);
        }
      } else if (topic_name == tf_topic_name_) {
        TFMessage msg;
        rclcpp::Serialization<TFMessage> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);

        auto buffer = std::dynamic_pointer_cast<Buffer<TFMessage>>(bag_data->buffers[TOPIC::TF]);
        if (buffer) {
          buffer->append(msg);
        }

        // Also write tf messages to evaluation bag
        if (evaluation_bag_writer_) {
          rclcpp::Time msg_time(serialized_message->time_stamp);
          evaluation_bag_writer_->write(msg, "/tf", msg_time);
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Failed to deserialize message on topic %s: %s", topic_name.c_str(),
        e.what());
    }
  }

  // Get kinematic states at 100ms intervals
  const auto evaluation_interval_ms =
    get_parameter_or_default<double>(*this, "evaluation_interval_ms", 100.0);
  auto kinematic_states = bag_data->get_kinematic_states_at_interval(evaluation_interval_ms);

  if (kinematic_states.empty()) {
    RCLCPP_ERROR(get_logger(), "No kinematic states found in the rosbag");
    return {now(), now()};
  }

  // Process each kinematic state with synchronized data
  std::vector<std::shared_ptr<SynchronizedData>> synchronized_data_list;
  const double sync_tolerance_ms =
    get_parameter_or_default<double>(*this, "sync_tolerance_ms", 50.0);

  for (const auto & kinematic_state : kinematic_states) {
    const auto timestamp = rclcpp::Time(kinematic_state->header.stamp).nanoseconds();
    auto sync_data = bag_data->get_synchronized_data_at_time(timestamp, sync_tolerance_ms);

    if (sync_data && sync_data->trajectory) {
      synchronized_data_list.push_back(sync_data);
    }
  }

  // Evaluate the synchronized data
  if (!synchronized_data_list.empty()) {
    ClosedLoopEvaluator evaluator(get_logger(), route_handler_);
    evaluator.evaluate(synchronized_data_list, evaluation_bag_writer_.get());

    // Get and log summary
    auto summary = evaluator.get_summary();
    RCLCPP_INFO(
      get_logger(),
      "Evaluation Summary:\n"
      "  Total samples: %zu\n"
      "  Mean lateral error: %.3f m\n"
      "  Max lateral error: %.3f m\n"
      "  Std lateral error: %.3f m\n"
      "  Mean acceleration: %.3f m/s²\n"
      "  Max acceleration: %.3f m/s²\n"
      "  Steering reversals: %zu\n"
      "  Mean steering angular velocity: %.3f rad/s\n"
      "  Min TTC: %.3f s\n"
      "  Total time: %.3f s",
      summary.num_samples, summary.mean_lateral_error, summary.max_lateral_error,
      summary.std_lateral_error, summary.mean_acceleration, summary.max_acceleration,
      summary.steering_reversals, summary.mean_steering_angular_velocity, summary.min_ttc,
      summary.total_time);

    // Save JSON output
    const auto json_output_path =
      get_parameter_or_default<std::string>(*this, "json_output_path", "~/evaluation_result.json");
    std::string expanded_path = json_output_path;

    // Expand home directory if needed
    if (expanded_path[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home) {
        expanded_path = std::string(home) + expanded_path.substr(1);
      }
    }

    // Add timestamp to filename
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp_ss;
    timestamp_ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

    // Create filename with timestamp
    std::filesystem::path json_path(expanded_path);
    std::string filename = json_path.stem().string() + "_" + timestamp_ss.str() + ".json";
    json_path = json_path.parent_path() / filename;

    // Get JSON summary
    nlohmann::json json_output = evaluator.get_summary_as_json();

    // Add evaluation info
    json_output["evaluation_info"]["timestamp"] = timestamp_ss.str();
    json_output["evaluation_info"]["bag_path"] = bag_path_;
    json_output["evaluation_info"]["evaluation_mode"] = "closed_loop";

    // Write JSON file
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
      json_file << json_output.dump(2);  // Pretty print with 2 spaces
      json_file.close();
      RCLCPP_INFO(get_logger(), "JSON results saved to: %s", json_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save JSON results to: %s", json_path.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "Closed-loop evaluation complete");

  // Return the timestamps of the first and last evaluation data
  if (!kinematic_states.empty()) {
    return {
      rclcpp::Time(kinematic_states.front()->header.stamp),
      rclcpp::Time(kinematic_states.back()->header.stamp)};
  }
  return {now(), now()};
}

void OfflineEvaluatorNode::write_map_and_route_markers_to_bag(const rclcpp::Time & reference_time)
{
  if (!evaluation_bag_writer_) {
    return;
  }

  // Create and write map markers from lanelet map
  if (map_marker_) {
    // Create a copy and update timestamps to match the evaluation bag timeline
    visualization_msgs::msg::MarkerArray time_corrected_markers = *map_marker_;
    
    // Update all marker timestamps to the reference time
    // This ensures markers are synchronized with the TF data in the evaluation bag
    for (auto & marker : time_corrected_markers.markers) {
      marker.header.stamp = reference_time;
    }
    
    evaluation_bag_writer_->write(time_corrected_markers, "/evaluation/map_markers", reference_time);
  }

  // Set route for route handler if available
  if (route_handler_ && route_handler_->isHandlerReady()) {
    // Create and save route markers
    visualization_msgs::msg::MarkerArray route_markers;
    create_route_markers(route_markers);
    if (!route_markers.markers.empty()) {
      evaluation_bag_writer_->write(route_markers, "/evaluation/route_markers", reference_time);
    }
  }
}

void OfflineEvaluatorNode::create_route_markers(
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (!route_handler_ || !route_handler_->isHandlerReady()) {
    return;
  }

  // Get preferred lanes (which are part of the route)
  const auto preferred_lanes = route_handler_->getPreferredLanelets();
  if (preferred_lanes.empty()) {
    return;
  }

  autoware_utils::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "preferred_lane", preferred_lanes, create_marker_color(0.16, 1.0, 0.69, 0.2)),
    &marker_array);
}
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode)
