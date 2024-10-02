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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_new_planning_msgs/msg/trajectory.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <vector>

namespace autoware::trajectory_selector::trajectory_ranker
{
// std
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::seconds;

// autoware
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_new_planning_msgs::msg::Trajectory;
using autoware_perception_msgs::msg::PredictedObjectKinematics;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::TrajectoryPoint;
using route_handler::RouteHandler;
using vehicle_info_utils::VehicleInfo;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

// ros2
using builtin_interfaces::msg::Duration;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Header;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
}  // namespace autoware::trajectory_selector::trajectory_ranker

#endif  // TYPE_ALIAS_HPP_