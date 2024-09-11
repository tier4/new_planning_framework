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

#ifndef AUTOWARE__TRAJECTORY_FILTER_INTERFACE__INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_FILTER_INTERFACE__INTERFACE_HPP_

#include "autoware/trajectory_filter_interface/type_alias.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::trajectory_selector
{

class TrajectoryFilterInterface : public rclcpp::Node
{
public:
  explicit TrajectoryFilterInterface(
    const std::string & name, const rclcpp::NodeOptions & node_options)
  : Node{name, node_options},
    sub_trajectories_{this->create_subscription<Trajectories>(
      "~/input/trajectories", 1,
      std::bind(&TrajectoryFilterInterface::on_trajectories, this, std::placeholders::_1))},
    pub_trajectories_{this->create_publisher<Trajectories>("~/output/trajectories", 1)}
  {
  }

protected:
  virtual auto process(const Trajectories::ConstSharedPtr msg) -> std::optional<Trajectories> = 0;

private:
  void on_trajectories(const Trajectories::ConstSharedPtr msg)
  {
    const auto output = process(msg);

    if (!output.has_value()) {
      return;
    }

    pub_trajectories_->publish(output.value());
  }

  rclcpp::Subscription<Trajectories>::SharedPtr sub_trajectories_;
  rclcpp::Publisher<Trajectories>::SharedPtr pub_trajectories_;
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_FILTER_INTERFACE__INTERFACE_HPP_
