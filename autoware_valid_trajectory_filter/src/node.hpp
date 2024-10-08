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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/trajectory_selector_common/interface/node_interface.hpp"

namespace autoware::trajectory_selector::valid_trajectory_filter
{

class ValidTrajectoryFilterNode : public TrajectoryFilterInterface
{
public:
  explicit ValidTrajectoryFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const Trajectories::ConstSharedPtr msg) override;
};

}  // namespace autoware::trajectory_selector::valid_trajectory_filter

#endif  // NODE_HPP_
