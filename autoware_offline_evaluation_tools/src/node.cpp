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

  // Removed unused services

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

// Removed unused parameter functions

// Removed unused get_route function


// Removed unused data processing functions

// Removed unused service functions

// Removed unused validation functions
}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode)
