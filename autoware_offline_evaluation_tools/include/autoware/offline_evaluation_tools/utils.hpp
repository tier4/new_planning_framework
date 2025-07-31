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

#ifndef AUTOWARE__OFFLINE_EVALUATION_TOOLS__UTILS_HPP_
#define AUTOWARE__OFFLINE_EVALUATION_TOOLS__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <memory>
#include <string>

namespace autoware::trajectory_selector::offline_evaluation_tools::utils
{

// Constants for evaluation
namespace constants
{
constexpr double DEFAULT_SYNC_TOLERANCE_MS = 50.0;
constexpr double DEFAULT_EVALUATION_INTERVAL_MS = 100.0;
constexpr double DEFAULT_BUFFER_DURATION_SEC = 20.0;
constexpr size_t DEFAULT_MAX_BUFFER_MESSAGES = 10000;
}  // namespace constants

// Template helper for safe numeric conversions
template<typename To, typename From>
inline To safe_numeric_cast(From value)
{
  return static_cast<To>(value);
}

// Template helper for message deserialization
template <typename MessageType>
std::shared_ptr<MessageType> deserialize_message(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & serialized_message,
  rclcpp::Logger logger)
{
  try {
    MessageType msg;
    rclcpp::Serialization<MessageType> serializer;
    rclcpp::SerializedMessage serialized_msg(*serialized_message->serialized_data);
    serializer.deserialize_message(&serialized_msg, &msg);
    return std::make_shared<MessageType>(msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "Failed to deserialize message: %s", e.what());
    return nullptr;
  }
}

// Template helper for message deserialization with header timestamp override
template <typename MessageType>
std::shared_ptr<MessageType> deserialize_message_with_bag_timestamp(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & serialized_message,
  bool use_bag_timestamp,
  rclcpp::Logger logger)
{
  auto msg = deserialize_message<MessageType>(serialized_message, logger);
  if (msg && use_bag_timestamp) {
    msg->header.stamp = rclcpp::Time(serialized_message->time_stamp);
  }
  return msg;
}

}  // namespace autoware::trajectory_selector::offline_evaluation_tools::utils

#endif  // AUTOWARE__OFFLINE_EVALUATION_TOOLS__UTILS_HPP_