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

#ifndef BAG_HANDLER_HPP_
#define BAG_HANDLER_HPP_

#include "autoware/trajectory_selector_common/structs.hpp"
#include "autoware/trajectory_selector_common/type_alias.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

using autoware_planning_msgs::msg::Trajectory;

struct SynchronizedData
{
  std::shared_ptr<Odometry> kinematic_state;
  std::shared_ptr<Trajectory> trajectory;
  std::shared_ptr<AccelWithCovarianceStamped> acceleration;
  std::shared_ptr<SteeringReport> steering_status;
  std::shared_ptr<PredictedObjects> objects;
  rclcpp::Time timestamp;  // Header timestamp (for data synchronization)
  rclcpp::Time bag_timestamp;  // Bag recording timestamp (for bag writing)
};

struct TOPIC
{
  static std::string TF;
  static std::string ODOMETRY;
  static std::string ACCELERATION;
  static std::string OBJECTS;
  static std::string TRAJECTORY;
  static std::string STEERING;
  static std::string ROUTE;
};

struct BufferBase
{
  virtual bool ready() const = 0;
  virtual void remove_old_data(const rcutils_time_point_value_t now) = 0;
};

template <typename T>
struct Buffer : BufferBase
{
  std::vector<T> msgs;

  double buffer_time_ns = 20.0 * 1e9;  // Made configurable, default 20 seconds
  size_t max_buffer_size = 10000;  // Maximum number of messages to keep

  bool ready() const override
  {
    if (msgs.empty()) {
      return false;
    }

    return rclcpp::Time(msgs.back().header.stamp).nanoseconds() -
             rclcpp::Time(msgs.front().header.stamp).nanoseconds() >
           buffer_time_ns;
  }

  void remove_old_data(const rcutils_time_point_value_t now) override
  {
    const auto itr = std::remove_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() < now;
    });
    msgs.erase(itr, msgs.end());
  }

  void append(const T & msg) 
  { 
    msgs.push_back(msg);
    
    // Prevent unbounded growth by removing old messages if buffer is too large
    if (msgs.size() > max_buffer_size) {
      // Remove oldest 10% of messages
      const size_t remove_count = max_buffer_size / 10;
      msgs.erase(msgs.begin(), msgs.begin() + remove_count);
    }
  }

  auto get(const rcutils_time_point_value_t now) const -> typename T::SharedPtr
  {
    const auto itr = std::find_if(msgs.begin(), msgs.end(), [&now, this](const auto & msg) {
      return rclcpp::Time(msg.header.stamp).nanoseconds() > now;
    });

    if (itr == msgs.end()) {
      return nullptr;
    }

    return std::make_shared<T>(*itr);
  }

  auto get_closest(const rcutils_time_point_value_t target_time, const double tolerance_ms = 50.0) const -> typename T::SharedPtr
  {
    if (msgs.empty()) {
      return nullptr;
    }

    const double tolerance_ns = tolerance_ms * 1e6;
    
    // Find the message with the closest timestamp
    auto closest_itr = msgs.begin();
    double min_diff = std::abs(static_cast<double>(rclcpp::Time(closest_itr->header.stamp).nanoseconds() - target_time));
    
    for (auto itr = msgs.begin(); itr != msgs.end(); ++itr) {
      const double diff = std::abs(static_cast<double>(rclcpp::Time(itr->header.stamp).nanoseconds() - target_time));
      if (diff < min_diff) {
        min_diff = diff;
        closest_itr = itr;
      }
    }
    
    // Check if within tolerance
    if (min_diff > tolerance_ns) {
      return nullptr;
    }
    
    return std::make_shared<T>(*closest_itr);
  }
};

template <>
bool Buffer<SteeringReport>::ready() const;

template <>
bool Buffer<TFMessage>::ready() const;

template <>
void Buffer<SteeringReport>::remove_old_data(const rcutils_time_point_value_t now);

template <>
void Buffer<TFMessage>::remove_old_data(const rcutils_time_point_value_t now);

template <>
auto Buffer<SteeringReport>::get(const rcutils_time_point_value_t now) const
  -> SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get(const rcutils_time_point_value_t now) const -> TFMessage::SharedPtr;

template <>
auto Buffer<SteeringReport>::get_closest(const rcutils_time_point_value_t target_time, const double tolerance_ms) const -> SteeringReport::SharedPtr;

template <>
auto Buffer<TFMessage>::get_closest(const rcutils_time_point_value_t target_time, const double tolerance_ms) const -> TFMessage::SharedPtr;

struct BagData
{
  // Template helper to create and configure buffer
  template<typename MessageType>
  void create_buffer(const std::string& topic_name, 
                     const double buffer_duration_sec,
                     const size_t max_buffer_msgs)
  {
    auto buffer = std::make_shared<Buffer<MessageType>>();
    buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(topic_name, buffer);
  }

  explicit BagData(const rcutils_time_point_value_t timestamp, 
                   const double buffer_duration_sec = 20.0,
                   const size_t max_buffer_msgs = 10000) : timestamp{timestamp}
  {
    // Create buffers using template helper
    create_buffer<TFMessage>(TOPIC::TF, buffer_duration_sec, max_buffer_msgs);
    create_buffer<Odometry>(TOPIC::ODOMETRY, buffer_duration_sec, max_buffer_msgs);
    create_buffer<AccelWithCovarianceStamped>(TOPIC::ACCELERATION, buffer_duration_sec, max_buffer_msgs);
    create_buffer<Trajectory>(TOPIC::TRAJECTORY, buffer_duration_sec, max_buffer_msgs);
    create_buffer<PredictedObjects>(TOPIC::OBJECTS, buffer_duration_sec, max_buffer_msgs);
    create_buffer<SteeringReport>(TOPIC::STEERING, buffer_duration_sec, max_buffer_msgs);
  }

  rcutils_time_point_value_t timestamp;

  std::map<std::string, std::shared_ptr<BufferBase>> buffers{};

  void update(const rcutils_time_point_value_t dt)
  {
    timestamp += dt;
    remove_old_data();
  }

  void remove_old_data()
  {
    std::for_each(buffers.begin(), buffers.end(), [this](const auto & buffer) {
      buffer.second->remove_old_data(timestamp);
    });
  }

  bool ready() const
  {
    return std::all_of(
      buffers.begin(), buffers.end(), [](const auto & buffer) { return buffer.second->ready(); });
  }

  auto get_synchronized_data_at_time(const rcutils_time_point_value_t target_time, const double tolerance_ms = 50.0) const -> std::shared_ptr<SynchronizedData>
  {
    auto synchronized_data = std::make_shared<SynchronizedData>();
    synchronized_data->timestamp = rclcpp::Time(target_time);
    synchronized_data->bag_timestamp = rclcpp::Time(target_time);  // Set bag_timestamp to the requested time

    // Get odometry buffer
    auto odom_buffer = std::dynamic_pointer_cast<Buffer<Odometry>>(buffers.at(TOPIC::ODOMETRY));
    if (!odom_buffer) return nullptr;
    
    synchronized_data->kinematic_state = odom_buffer->get_closest(target_time, tolerance_ms);
    if (!synchronized_data->kinematic_state) return nullptr;

    // Get trajectory
    auto traj_buffer = std::dynamic_pointer_cast<Buffer<Trajectory>>(buffers.at(TOPIC::TRAJECTORY));
    if (traj_buffer) {
      synchronized_data->trajectory = traj_buffer->get_closest(target_time, tolerance_ms);
    }

    // Get acceleration
    auto accel_buffer = std::dynamic_pointer_cast<Buffer<AccelWithCovarianceStamped>>(buffers.at(TOPIC::ACCELERATION));
    if (accel_buffer) {
      synchronized_data->acceleration = accel_buffer->get_closest(target_time, tolerance_ms);
    }

    // Get steering status
    auto steer_buffer = std::dynamic_pointer_cast<Buffer<SteeringReport>>(buffers.at(TOPIC::STEERING));
    if (steer_buffer) {
      synchronized_data->steering_status = steer_buffer->get_closest(target_time, tolerance_ms);
    }

    // Get objects
    auto obj_buffer = std::dynamic_pointer_cast<Buffer<PredictedObjects>>(buffers.at(TOPIC::OBJECTS));
    if (obj_buffer) {
      synchronized_data->objects = obj_buffer->get_closest(target_time, tolerance_ms);
    }

    return synchronized_data;
  }

  // Template helper to append message to appropriate buffer
  template<typename MessageType>
  bool append_message(const std::string& topic_name, const MessageType& msg)
  {
    auto buffer = std::dynamic_pointer_cast<Buffer<MessageType>>(buffers[topic_name]);
    if (buffer) {
      buffer->append(msg);
      return true;
    }
    return false;
  }

  // Template helper to get buffer for a specific message type
  template<typename MessageType>
  std::shared_ptr<Buffer<MessageType>> get_buffer(const std::string& topic_name)
  {
    return std::dynamic_pointer_cast<Buffer<MessageType>>(buffers[topic_name]);
  }

  auto get_kinematic_states_at_interval(const double interval_ms = 100.0) const -> std::vector<std::shared_ptr<Odometry>>
  {
    std::vector<std::shared_ptr<Odometry>> result;
    
    auto odom_buffer = std::dynamic_pointer_cast<Buffer<Odometry>>(buffers.at(TOPIC::ODOMETRY));
    if (!odom_buffer || odom_buffer->msgs.empty()) {
      return result;
    }

    // Get first and last timestamp from odometry messages
    const auto first_time = rclcpp::Time(odom_buffer->msgs.front().header.stamp).nanoseconds();
    const auto last_time = rclcpp::Time(odom_buffer->msgs.back().header.stamp).nanoseconds();
    const auto interval_ns = static_cast<rcutils_time_point_value_t>(interval_ms * 1e6);

    // Sample at regular intervals
    for (auto current_time = first_time; current_time <= last_time; current_time += interval_ns) {
      auto odom = odom_buffer->get_closest(current_time, interval_ms / 2.0);
      if (odom) {
        result.push_back(odom);
      }
    }

    return result;
  }
};

struct ReplayEvaluationData : public BagData
{
  explicit ReplayEvaluationData(const rcutils_time_point_value_t timestamp,
                               const double buffer_duration_sec = 20.0,
                               const size_t max_buffer_msgs = 10000) 
    : BagData(timestamp, buffer_duration_sec, max_buffer_msgs)
  {
    live_trajectory_buffer = std::make_shared<Buffer<Trajectory>>();
    live_trajectory_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    live_trajectory_buffer->max_buffer_size = max_buffer_msgs;
  }

  std::shared_ptr<Buffer<Trajectory>> live_trajectory_buffer;

  void append_live_trajectory(const Trajectory & trajectory)
  {
    live_trajectory_buffer->append(trajectory);
  }

  auto get_live_trajectory(const rcutils_time_point_value_t now) const -> Trajectory::SharedPtr
  {
    return live_trajectory_buffer->get(now);
  }

  bool live_trajectory_ready() const
  {
    return live_trajectory_buffer->ready();
  }
};

// Helper trait to detect if a type has header.stamp (C++17 compatible)
template<typename T, typename = void>
struct has_header_stamp : std::false_type {};

template<typename T>
struct has_header_stamp<T, std::void_t<decltype(std::declval<T>().header.stamp)>> : std::true_type {};

// Template helper to set timestamp for messages with header
template<typename MessageType>
typename std::enable_if<has_header_stamp<MessageType>::value, void>::type
set_header_timestamp_if_needed(MessageType& msg, bool use_bag_timestamp, const rclcpp::Time& bag_time)
{
  if (use_bag_timestamp && msg.header.stamp != rclcpp::Time(0)) {
    msg.header.stamp = bag_time;
  }
}

// Template helper for messages without header - does nothing
template<typename MessageType>
typename std::enable_if<!has_header_stamp<MessageType>::value, void>::type
set_header_timestamp_if_needed(MessageType&, bool, const rclcpp::Time&)
{
  // No-op for messages without header.stamp
}

// Template helper to process and append message to bag data
template<typename MessageType>
void process_and_append_message(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& serialized_message,
  std::shared_ptr<BagData> bag_data,
  const std::string& topic_key,
  bool use_bag_timestamp,
  rclcpp::Logger logger)
{
  try {
    MessageType msg;
    rclcpp::Serialization<MessageType> serializer;
    rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
    serializer.deserialize_message(&serialized_msg, &msg);
    
    // Override header timestamp with bag timestamp if option is enabled
    set_header_timestamp_if_needed(msg, use_bag_timestamp, rclcpp::Time(serialized_message->time_stamp));
    
    bag_data->append_message<MessageType>(topic_key, msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "Failed to deserialize message on topic %s: %s", 
      serialized_message->topic_name.c_str(), e.what());
  }
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // BAG_HANDLER_HPP_
