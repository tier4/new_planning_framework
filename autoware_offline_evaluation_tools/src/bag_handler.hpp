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

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_selector::offline_evaluation_tools
{

using autoware_planning_msgs::msg::Trajectory;

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

struct BagData
{
  explicit BagData(const rcutils_time_point_value_t timestamp, 
                   const double buffer_duration_sec = 20.0,
                   const size_t max_buffer_msgs = 10000) : timestamp{timestamp}
  {
    // Helper to create buffer with configuration
    auto create_tf_buffer = std::make_shared<Buffer<TFMessage>>();
    create_tf_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    create_tf_buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(TOPIC::TF, create_tf_buffer);
    
    auto create_odom_buffer = std::make_shared<Buffer<Odometry>>();
    create_odom_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    create_odom_buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(TOPIC::ODOMETRY, create_odom_buffer);
    
    auto create_accel_buffer = std::make_shared<Buffer<AccelWithCovarianceStamped>>();
    create_accel_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    create_accel_buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(TOPIC::ACCELERATION, create_accel_buffer);
    
    auto create_traj_buffer = std::make_shared<Buffer<Trajectory>>();
    create_traj_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    create_traj_buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(TOPIC::TRAJECTORY, create_traj_buffer);
    
    auto create_obj_buffer = std::make_shared<Buffer<PredictedObjects>>();
    create_obj_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    create_obj_buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(TOPIC::OBJECTS, create_obj_buffer);
    
    auto create_steer_buffer = std::make_shared<Buffer<SteeringReport>>();
    create_steer_buffer->buffer_time_ns = buffer_duration_sec * 1e9;
    create_steer_buffer->max_buffer_size = max_buffer_msgs;
    buffers.emplace(TOPIC::STEERING, create_steer_buffer);
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

}  // namespace autoware::trajectory_selector::offline_evaluation_tools

#endif  // BAG_HANDLER_HPP_
