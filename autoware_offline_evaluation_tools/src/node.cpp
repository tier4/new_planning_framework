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
#include "closed_loop_evaluator.hpp"

#include "autoware/trajectory_selector_common/utils.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/system/stop_watch.hpp"

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <rmw/rmw.h>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <limits>
#include <chrono>
#include <ctime>
#include <iomanip>

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
    [this](const LaneletMapBin::ConstSharedPtr msg) { 
      route_handler_->setMap(*msg); 
    });
    
  // Open bag file
  bag_path_ = get_or_declare_parameter<std::string>(*this, "bag_path");
  try {
    bag_reader_.open(bag_path_);
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
  
  // Read evaluation mode
  const auto mode_str = get_parameter_or_default<std::string>(*this, "evaluation.mode", "closed_loop");
  if (mode_str == "open_loop") {
    evaluation_mode_ = EvaluationMode::OPEN_LOOP;
  } else if (mode_str == "closed_loop") {
    evaluation_mode_ = EvaluationMode::CLOSED_LOOP;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid evaluation mode: %s. Using CLOSED_LOOP.", mode_str.c_str());
    evaluation_mode_ = EvaluationMode::CLOSED_LOOP;
  }
  
  // Create a timer to check map readiness without blocking
  map_check_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      if (route_handler_->isMapMsgReady()) {
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
    auto output_bag_path = get_parameter_or_default<std::string>(
      *this, "evaluation_output_bag_path", "~/trajectory_evaluation_results.bag");
    
    // Expand home directory if needed
    if (output_bag_path[0] == '~') {
      const char* home = std::getenv("HOME");
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
    const rosbag2_storage::StorageOptions storage_options{
      output_bag_path,
      "mcap"
    };
    
    const rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(),
      rmw_get_serialization_format()
    };
    
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
      {"/evaluation/localization", "nav_msgs/msg/Odometry"}
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

void OfflineEvaluatorNode::run_evaluation()
{
  RCLCPP_INFO(get_logger(), "Starting evaluation...");
  
  switch (evaluation_mode_) {
    case EvaluationMode::OPEN_LOOP:
      run_open_loop_evaluation();
      break;
    case EvaluationMode::CLOSED_LOOP:
      run_closed_loop_evaluation();
      break;
  }
  
  RCLCPP_INFO(get_logger(), "Evaluation complete");
}

void OfflineEvaluatorNode::run_open_loop_evaluation()
{
  RCLCPP_INFO(get_logger(), "Running open-loop evaluation for manual driving data");
  
  // Open-loop evaluation is for manual driving data
  // It evaluates the actual driven trajectory without re-planning
  RCLCPP_WARN(get_logger(), "Open-loop evaluation is not yet implemented");
  
  RCLCPP_INFO(get_logger(), "Open-loop evaluation complete");
}

void OfflineEvaluatorNode::run_closed_loop_evaluation()
{
  RCLCPP_INFO(get_logger(), "Running closed-loop evaluation for autonomous driving data");
  
  // Create bag data handler
  const auto buffer_duration_sec = get_parameter_or_default<double>(*this, "buffer_duration_sec", 20.0);
  const size_t max_buffer_messages = static_cast<size_t>(get_parameter_or_default<int64_t>(*this, "max_buffer_messages", 10000));
  
  auto bag_data = std::make_shared<BagData>(0, buffer_duration_sec, max_buffer_messages);
  
  // Read all messages from bag into buffers
  RCLCPP_INFO(get_logger(), "Loading rosbag data into buffers...");
  
  // Storage for the last route and map messages
  autoware_planning_msgs::msg::LaneletRoute::SharedPtr last_route_msg = nullptr;
  autoware_map_msgs::msg::LaneletMapBin::SharedPtr last_map_msg = nullptr;
  
  while (bag_reader_.has_next() && rclcpp::ok()) {
    auto serialized_message = bag_reader_.read_next();
    const auto & topic_name = serialized_message->topic_name;
    
    try {
      if (topic_name == odometry_topic_name_) {
        Odometry msg;
        rclcpp::Serialization<Odometry> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<Odometry>>(bag_data->buffers[TOPIC::ODOMETRY]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == trajectory_topic_name_) {
        Trajectory msg;
        rclcpp::Serialization<Trajectory> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<Trajectory>>(bag_data->buffers[TOPIC::TRAJECTORY]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == acceleration_topic_name_) {
        AccelWithCovarianceStamped msg;
        rclcpp::Serialization<AccelWithCovarianceStamped> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(bag_data->buffers[TOPIC::ACCELERATION]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == steering_topic_name_) {
        SteeringReport msg;
        rclcpp::Serialization<SteeringReport> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<SteeringReport>>(bag_data->buffers[TOPIC::STEERING]);
        if (buffer) {
          buffer->append(msg);
        }
      } else if (topic_name == objects_topic_name_) {
        PredictedObjects msg;
        rclcpp::Serialization<PredictedObjects> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        auto buffer = std::dynamic_pointer_cast<Buffer<PredictedObjects>>(bag_data->buffers[TOPIC::OBJECTS]);
        if (buffer) {
          buffer->append(msg);
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
      } else if (topic_name == route_topic_name_) {
        autoware_planning_msgs::msg::LaneletRoute msg;
        rclcpp::Serialization<autoware_planning_msgs::msg::LaneletRoute> serializer;
        rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
        serializer.deserialize_message(&serialized_msg, &msg);
        
        // Store only the last route message
        last_route_msg = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>(msg);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to deserialize message on topic %s: %s", topic_name.c_str(), e.what());
    }
  }
  
  // Set map and route to route_handler
  if (last_route_msg) {
    route_handler_->setRoute(*last_route_msg);
  } else {
    RCLCPP_WARN(get_logger(), "Map or route message not found in rosbag. Position errors will use trajectory instead of preferred lane.");
  }
  
  // Get kinematic states at 100ms intervals
  const auto evaluation_interval_ms = get_parameter_or_default<double>(*this, "evaluation_interval_ms", 100.0);
  auto kinematic_states = bag_data->get_kinematic_states_at_interval(evaluation_interval_ms);
  
  if (kinematic_states.empty()) {
    RCLCPP_ERROR(get_logger(), "No kinematic states found in the rosbag");
    return;
  }

  // Process each kinematic state with synchronized data
  std::vector<std::shared_ptr<SynchronizedData>> synchronized_data_list;
  const double sync_tolerance_ms = get_parameter_or_default<double>(*this, "sync_tolerance_ms", 50.0);
  
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
    RCLCPP_INFO(get_logger(), 
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
      summary.num_samples,
      summary.mean_lateral_error,
      summary.max_lateral_error,
      summary.std_lateral_error,
      summary.mean_acceleration,
      summary.max_acceleration,
      summary.steering_reversals,
      summary.mean_steering_angular_velocity,
      summary.min_ttc,
      summary.total_time);
    
    // Save summary to file
    auto summary_file = get_parameter_or_default<std::string>(*this, "summary_output_file", "~/evaluation_summary.txt");
    
    // Expand home directory if needed
    if (summary_file[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) {
        summary_file = std::string(home) + summary_file.substr(1);
      }
    }
    
    std::ofstream ofs(summary_file);
    if (ofs.is_open()) {
      ofs << "Closed-Loop Evaluation Summary\n";
      ofs << "==============================\n";
      ofs << "Total samples: " << summary.num_samples << "\n";
      ofs << "Mean lateral error: " << summary.mean_lateral_error << " m\n";
      ofs << "Max lateral error: " << summary.max_lateral_error << " m\n";
      ofs << "Std lateral error: " << summary.std_lateral_error << " m\n";
      ofs << "Mean acceleration: " << summary.mean_acceleration << " m/s²\n";
      ofs << "Max acceleration: " << summary.max_acceleration << " m/s²\n";
      ofs << "Mean jerk: " << summary.mean_jerk << " m/s³\n";
      ofs << "Max jerk: " << summary.max_jerk << " m/s³\n";
      ofs << "\nOscillation Metrics:\n";
      ofs << "Steering reversals: " << summary.steering_reversals << "\n";
      ofs << "Mean steering angular velocity: " << summary.mean_steering_angular_velocity << " rad/s\n";
      ofs << "Max steering angular velocity: " << summary.max_steering_angular_velocity << " rad/s\n";
      ofs << "Std steering angle: " << summary.std_steering_angle << " rad\n";
      ofs << "Mean lateral jerk: " << summary.mean_lateral_jerk << " m/s³\n";
      ofs << "Max lateral jerk: " << summary.max_lateral_jerk << " m/s³\n";
      ofs << "\nMin TTC: " << summary.min_ttc << " s\n";
      ofs << "Total time: " << summary.total_time << " s\n";
      ofs.close();
      RCLCPP_INFO(get_logger(), "Summary saved to: %s", summary_file.c_str());
    }
    
    // Save JSON output
    const auto json_output_path = get_parameter_or_default<std::string>(*this, "json_output_path", "~/evaluation_result.json");
    std::string expanded_path = json_output_path;
    
    // Expand home directory if needed
    if (expanded_path[0] == '~') {
      const char* home = std::getenv("HOME");
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
}
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode)
