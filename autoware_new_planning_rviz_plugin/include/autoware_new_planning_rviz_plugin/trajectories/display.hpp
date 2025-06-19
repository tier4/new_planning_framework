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

#ifndef AUTOWARE_NEW_PLANNING_RVIZ_PLUGIN__TRAJECTORIES__DISPLAY_HPP_
#define AUTOWARE_NEW_PLANNING_RVIZ_PLUGIN__TRAJECTORIES__DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <autoware_new_planning_msgs/msg/trajectories.hpp>
#include <autoware_new_planning_msgs/msg/trajectory.hpp>
#include <autoware_new_planning_msgs/msg/trajectory_generator_info.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware_new_planning_rviz_plugin
{
class TrajectoriesDisplay
: public rviz_common::MessageFilterDisplay<autoware_new_planning_msgs::msg::Trajectories>
{
  Q_OBJECT

public:
  TrajectoriesDisplay();
  ~TrajectoriesDisplay() override;

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateColorScheme();
  void updateTrajectorySelection();

private:
  void processMessage(
    const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr msg_ptr) override;

  void visualizeTrajectory(
    const autoware_new_planning_msgs::msg::Trajectory & trajectory,
    const Ogre::ColourValue & color,
    float alpha,
    bool show_points = true);

  void visualizeGeneratorInfo(
    const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr msg_ptr);

  bool validateFloats(
    const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr & msg_ptr);

  Ogre::ColourValue getTrajectoryColor(
    const autoware_new_planning_msgs::msg::Trajectory & trajectory,
    size_t index);

  std::string getGeneratorName(
    const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr & msg_ptr,
    const std::string & generator_id);

  // Manual objects for visualization
  std::vector<Ogre::ManualObject *> trajectory_manual_objects_;
  std::vector<Ogre::ManualObject *> points_manual_objects_;
  
  // Text objects for generator info and scores
  std::vector<rviz_rendering::MovableText *> generator_texts_;
  std::vector<Ogre::SceneNode *> generator_text_nodes_;
  std::vector<rviz_rendering::MovableText *> score_texts_;
  std::vector<Ogre::SceneNode *> score_text_nodes_;

  // Properties
  rviz_common::properties::BoolProperty property_view_all_trajectories_;
  rviz_common::properties::IntProperty property_selected_trajectory_index_;
  rviz_common::properties::EnumProperty property_color_scheme_;
  
  // Path visualization properties
  rviz_common::properties::FloatProperty property_line_width_;
  rviz_common::properties::FloatProperty property_alpha_;
  rviz_common::properties::BoolProperty property_show_points_;
  rviz_common::properties::FloatProperty property_point_size_;
  
  // Text display properties
  rviz_common::properties::BoolProperty property_show_generator_info_;
  rviz_common::properties::FloatProperty property_generator_text_scale_;
  rviz_common::properties::BoolProperty property_show_scores_;
  rviz_common::properties::FloatProperty property_score_text_scale_;
  
  // Color properties for different color schemes
  rviz_common::properties::ColorProperty property_best_trajectory_color_;
  rviz_common::properties::ColorProperty property_other_trajectories_color_;
  std::vector<rviz_common::properties::ColorProperty *> property_generator_colors_;

  // Cache for generator colors
  std::unordered_map<std::string, Ogre::ColourValue> generator_color_map_;
  
  // Last message for updates
  autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr last_msg_ptr_;
};

}  // namespace autoware_new_planning_rviz_plugin

#endif  // AUTOWARE_NEW_PLANNING_RVIZ_PLUGIN__TRAJECTORIES__DISPLAY_HPP_