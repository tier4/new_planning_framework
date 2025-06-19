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

#include "autoware_new_planning_rviz_plugin/trajectories/display.hpp"

#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/logging.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace autoware_new_planning_rviz_plugin
{

TrajectoriesDisplay::TrajectoriesDisplay()
: property_view_all_trajectories_("View All Trajectories", true, 
    "Display all trajectories or only selected one", this, SLOT(updateTrajectorySelection())),
  property_selected_trajectory_index_("Selected Trajectory", 0,
    "Index of trajectory to display when not viewing all", &property_view_all_trajectories_),
  property_color_scheme_("Color Scheme", "By Score",
    "How to color trajectories", this, SLOT(updateColorScheme())),
  property_line_width_("Line Width", 0.1f,
    "Width of trajectory lines", this),
  property_alpha_("Alpha", 0.7f,
    "Transparency of trajectories", this),
  property_show_points_("Show Points", true,
    "Display individual trajectory points", this),
  property_point_size_("Point Size", 0.05f,
    "Size of trajectory points", &property_show_points_),
  property_show_generator_info_("Show Generator Info", true,
    "Display generator names", this),
  property_generator_text_scale_("Generator Text Scale", 0.5f,
    "Scale of generator name text", &property_show_generator_info_),
  property_show_scores_("Show Scores", true,
    "Display trajectory scores", this),
  property_score_text_scale_("Score Text Scale", 0.3f,
    "Scale of score text", &property_show_scores_),
  property_best_trajectory_color_("Best Trajectory Color", QColor(0, 255, 0),
    "Color for best scoring trajectory", this),
  property_other_trajectories_color_("Other Trajectories Color", QColor(128, 128, 128),
    "Color for other trajectories", this)
{
  // Set property constraints
  property_selected_trajectory_index_.setMin(0);
  property_line_width_.setMin(0.001f);
  property_alpha_.setMin(0.0f);
  property_alpha_.setMax(1.0f);
  property_point_size_.setMin(0.001f);
  property_generator_text_scale_.setMin(0.1f);
  property_score_text_scale_.setMin(0.1f);

  // Add color scheme options
  property_color_scheme_.addOption("By Score", 0);
  property_color_scheme_.addOption("By Generator", 1);
  property_color_scheme_.addOption("Uniform", 2);

  // Initially hide selected trajectory index when viewing all
  property_selected_trajectory_index_.setHidden(property_view_all_trajectories_.getBool());
}

TrajectoriesDisplay::~TrajectoriesDisplay()
{
  if (initialized()) {
    // Clean up manual objects
    for (auto * manual_object : trajectory_manual_objects_) {
      scene_manager_->destroyManualObject(manual_object);
    }
    for (auto * manual_object : points_manual_objects_) {
      scene_manager_->destroyManualObject(manual_object);
    }

    // Clean up text nodes
    for (size_t i = 0; i < generator_text_nodes_.size(); i++) {
      Ogre::SceneNode * node = generator_text_nodes_.at(i);
      node->removeAndDestroyAllChildren();
      node->detachAllObjects();
      scene_manager_->destroySceneNode(node);
    }
    for (size_t i = 0; i < score_text_nodes_.size(); i++) {
      Ogre::SceneNode * node = score_text_nodes_.at(i);
      node->removeAndDestroyAllChildren();
      node->detachAllObjects();
      scene_manager_->destroySceneNode(node);
    }

    // Clean up generator color properties
    for (auto * color_prop : property_generator_colors_) {
      delete color_prop;
    }
  }
}

void TrajectoriesDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void TrajectoriesDisplay::reset()
{
  MFDClass::reset();

  // Clear all manual objects
  for (auto * manual_object : trajectory_manual_objects_) {
    manual_object->clear();
  }
  for (auto * manual_object : points_manual_objects_) {
    manual_object->clear();
  }

  // Clear text displays
  for (auto * text : generator_texts_) {
    text->setVisible(false);
  }
  for (auto * text : score_texts_) {
    text->setVisible(false);
  }
}

void TrajectoriesDisplay::updateColorScheme()
{
  if (last_msg_ptr_) {
    processMessage(last_msg_ptr_);
  }
}

void TrajectoriesDisplay::updateTrajectorySelection()
{
  property_selected_trajectory_index_.setHidden(property_view_all_trajectories_.getBool());
  if (last_msg_ptr_) {
    processMessage(last_msg_ptr_);
  }
}

bool TrajectoriesDisplay::validateFloats(
  const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr & msg_ptr)
{
  for (const auto & trajectory : msg_ptr->trajectories) {
    for (const auto & point : trajectory.points) {
      if (!rviz_common::validateFloats(point.pose) ||
          !rviz_common::validateFloats(point.longitudinal_velocity_mps) ||
          !rviz_common::validateFloats(point.lateral_velocity_mps) ||
          !rviz_common::validateFloats(point.acceleration_mps2)) {
        return false;
      }
    }
    if (!rviz_common::validateFloats(trajectory.score)) {
      return false;
    }
  }
  return true;
}

std::string TrajectoriesDisplay::getGeneratorName(
  const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr & msg_ptr,
  const std::string & generator_id)
{
  for (const auto & info : msg_ptr->generator_info) {
    if (info.generator_id.uuid == generator_id) {
      return info.generator_name.data;
    }
  }
  return "Unknown";
}

Ogre::ColourValue TrajectoriesDisplay::getTrajectoryColor(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  size_t index)
{
  int color_scheme = property_color_scheme_.getOptionInt();
  
  if (color_scheme == 0) {  // By Score
    // Assume first trajectory has best score
    if (index == 0) {
      return rviz_common::properties::qtToOgre(property_best_trajectory_color_.getColor());
    } else {
      return rviz_common::properties::qtToOgre(property_other_trajectories_color_.getColor());
    }
  } else if (color_scheme == 1) {  // By Generator
    // Use cached color or generate new one
    std::string generator_id = trajectory.generator_id.uuid;
    if (generator_color_map_.find(generator_id) == generator_color_map_.end()) {
      // Generate a color based on generator ID hash
      std::hash<std::string> hasher;
      size_t hash = hasher(generator_id);
      float hue = (hash % 360) / 360.0f;
      
      // Convert HSV to RGB (S=0.8, V=0.8 for pleasant colors)
      float s = 0.8f;
      float v = 0.8f;
      float c = v * s;
      float x = c * (1 - std::abs(std::fmod(hue * 6, 2) - 1));
      float m = v - c;
      
      float r, g, b;
      if (hue < 1.0f/6) {
        r = c; g = x; b = 0;
      } else if (hue < 2.0f/6) {
        r = x; g = c; b = 0;
      } else if (hue < 3.0f/6) {
        r = 0; g = c; b = x;
      } else if (hue < 4.0f/6) {
        r = 0; g = x; b = c;
      } else if (hue < 5.0f/6) {
        r = x; g = 0; b = c;
      } else {
        r = c; g = 0; b = x;
      }
      
      generator_color_map_[generator_id] = Ogre::ColourValue(r + m, g + m, b + m);
    }
    return generator_color_map_[generator_id];
  } else {  // Uniform
    return rviz_common::properties::qtToOgre(property_other_trajectories_color_.getColor());
  }
}

void TrajectoriesDisplay::visualizeTrajectory(
  const autoware_new_planning_msgs::msg::Trajectory & trajectory,
  const Ogre::ColourValue & color,
  float alpha,
  bool show_points)
{
  if (trajectory.points.empty()) {
    return;
  }

  // Get or create manual objects
  static size_t object_index = 0;
  if (object_index >= trajectory_manual_objects_.size()) {
    Ogre::ManualObject * traj_obj = scene_manager_->createManualObject();
    Ogre::ManualObject * points_obj = scene_manager_->createManualObject();
    traj_obj->setDynamic(true);
    points_obj->setDynamic(true);
    scene_node_->attachObject(traj_obj);
    scene_node_->attachObject(points_obj);
    trajectory_manual_objects_.push_back(traj_obj);
    points_manual_objects_.push_back(points_obj);
  }

  Ogre::ManualObject * traj_manual_object = trajectory_manual_objects_[object_index];
  Ogre::ManualObject * points_manual_object = points_manual_objects_[object_index];
  object_index++;

  // Set up material
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);

  // Draw trajectory line
  traj_manual_object->clear();
  traj_manual_object->estimateVertexCount(trajectory.points.size());
  traj_manual_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  Ogre::ColourValue traj_color = color;
  traj_color.a = alpha;

  for (const auto & point : trajectory.points) {
    traj_manual_object->position(
      point.pose.position.x,
      point.pose.position.y,
      point.pose.position.z);
    traj_manual_object->colour(traj_color);
  }
  traj_manual_object->end();

  // Draw points if enabled
  points_manual_object->clear();
  if (show_points && property_show_points_.getBool()) {
    points_manual_object->estimateVertexCount(trajectory.points.size() * 6);
    points_manual_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    const float point_size = property_point_size_.getFloat();
    Ogre::ColourValue point_color = color;
    point_color.a = alpha;

    for (const auto & point : trajectory.points) {
      const float x = point.pose.position.x;
      const float y = point.pose.position.y;
      const float z = point.pose.position.z;

      // Create a simple square for each point
      // Triangle 1
      points_manual_object->position(x - point_size/2, y - point_size/2, z);
      points_manual_object->colour(point_color);
      points_manual_object->position(x + point_size/2, y - point_size/2, z);
      points_manual_object->colour(point_color);
      points_manual_object->position(x + point_size/2, y + point_size/2, z);
      points_manual_object->colour(point_color);

      // Triangle 2
      points_manual_object->position(x - point_size/2, y - point_size/2, z);
      points_manual_object->colour(point_color);
      points_manual_object->position(x + point_size/2, y + point_size/2, z);
      points_manual_object->colour(point_color);
      points_manual_object->position(x - point_size/2, y + point_size/2, z);
      points_manual_object->colour(point_color);
    }
    points_manual_object->end();
  }
}

void TrajectoriesDisplay::visualizeGeneratorInfo(
  const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr msg_ptr)
{
  // Ensure we have enough text objects
  while (generator_texts_.size() < msg_ptr->trajectories.size()) {
    Ogre::SceneNode * node = scene_node_->createChildSceneNode();
    rviz_rendering::MovableText * text = 
      new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.1);
    text->setVisible(false);
    text->setTextAlignment(
      rviz_rendering::MovableText::H_LEFT, rviz_rendering::MovableText::V_ABOVE);
    node->attachObject(text);
    generator_texts_.push_back(text);
    generator_text_nodes_.push_back(node);
  }

  while (score_texts_.size() < msg_ptr->trajectories.size()) {
    Ogre::SceneNode * node = scene_node_->createChildSceneNode();
    rviz_rendering::MovableText * text = 
      new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.1);
    text->setVisible(false);
    text->setTextAlignment(
      rviz_rendering::MovableText::H_LEFT, rviz_rendering::MovableText::V_BELOW);
    node->attachObject(text);
    score_texts_.push_back(text);
    score_text_nodes_.push_back(node);
  }

  // Update text displays
  for (size_t i = 0; i < msg_ptr->trajectories.size(); ++i) {
    const auto & trajectory = msg_ptr->trajectories[i];
    
    if (!trajectory.points.empty()) {
      // Position at first point of trajectory
      const auto & first_point = trajectory.points.front();
      Ogre::Vector3 position(
        first_point.pose.position.x,
        first_point.pose.position.y,
        first_point.pose.position.z);

      // Generator info
      if (property_show_generator_info_.getBool() && i < generator_texts_.size()) {
        generator_text_nodes_[i]->setPosition(position);
        rviz_rendering::MovableText * text = generator_texts_[i];
        
        std::string generator_name = getGeneratorName(msg_ptr, trajectory.generator_id.uuid);
        text->setCaption(generator_name);
        text->setCharacterHeight(property_generator_text_scale_.getFloat());
        text->setColor(getTrajectoryColor(trajectory, i));
        text->setVisible(true);
      } else if (i < generator_texts_.size()) {
        generator_texts_[i]->setVisible(false);
      }

      // Score info
      if (property_show_scores_.getBool() && i < score_texts_.size()) {
        score_text_nodes_[i]->setPosition(position);
        rviz_rendering::MovableText * text = score_texts_[i];
        
        std::stringstream ss;
        ss << "Score: " << std::fixed << std::setprecision(2) << trajectory.score;
        text->setCaption(ss.str());
        text->setCharacterHeight(property_score_text_scale_.getFloat());
        text->setColor(getTrajectoryColor(trajectory, i));
        text->setVisible(true);
      } else if (i < score_texts_.size()) {
        score_texts_[i]->setVisible(false);
      }
    }
  }

  // Hide unused text objects
  for (size_t i = msg_ptr->trajectories.size(); i < generator_texts_.size(); ++i) {
    generator_texts_[i]->setVisible(false);
    score_texts_[i]->setVisible(false);
  }
}

void TrajectoriesDisplay::processMessage(
  const autoware_new_planning_msgs::msg::Trajectories::ConstSharedPtr msg_ptr)
{
  // Clear all manual objects
  for (auto * manual_object : trajectory_manual_objects_) {
    manual_object->clear();
  }
  for (auto * manual_object : points_manual_objects_) {
    manual_object->clear();
  }

  if (!validateFloats(msg_ptr)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Transform
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(
      msg_ptr->trajectories[0].header, position, orientation)) {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(
        msg_ptr->trajectories[0].header.frame_id, ros::Time(), error)) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Transform", 
        QString::fromStdString(error));
    } else {
      setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
        "Could not transform from [" + QString::fromStdString(msg_ptr->trajectories[0].header.frame_id) +
        "] to [" + fixed_frame_ + "]");
    }
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  // Update max trajectory index
  property_selected_trajectory_index_.setMax(
    static_cast<int>(msg_ptr->trajectories.size()) - 1);

  // Sort trajectories by score (best first)
  std::vector<size_t> sorted_indices(msg_ptr->trajectories.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(),
    [&msg_ptr](size_t a, size_t b) {
      return msg_ptr->trajectories[a].score > msg_ptr->trajectories[b].score;
    });

  // Visualize trajectories
  const bool view_all = property_view_all_trajectories_.getBool();
  const int selected_idx = property_selected_trajectory_index_.getInt();
  const float alpha = property_alpha_.getFloat();

  if (view_all) {
    // Display all trajectories
    for (size_t i = 0; i < sorted_indices.size(); ++i) {
      size_t traj_idx = sorted_indices[i];
      const auto & trajectory = msg_ptr->trajectories[traj_idx];
      
      Ogre::ColourValue color = getTrajectoryColor(trajectory, i);
      float traj_alpha = (i == 0) ? alpha : alpha * 0.5f;  // Best trajectory more opaque
      
      visualizeTrajectory(trajectory, color, traj_alpha);
    }
  } else {
    // Display only selected trajectory
    if (selected_idx >= 0 && selected_idx < static_cast<int>(msg_ptr->trajectories.size())) {
      const auto & trajectory = msg_ptr->trajectories[selected_idx];
      Ogre::ColourValue color = getTrajectoryColor(trajectory, 0);
      visualizeTrajectory(trajectory, color, alpha);
    }
  }

  // Visualize generator info and scores
  visualizeGeneratorInfo(msg_ptr);

  // Store message for updates
  last_msg_ptr_ = msg_ptr;

  // Update status
  std::stringstream ss;
  ss << msg_ptr->trajectories.size() << " trajectories";
  setStatus(rviz_common::properties::StatusProperty::Ok, "Trajectories", 
    QString::fromStdString(ss.str()));
}

}  // namespace autoware_new_planning_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware_new_planning_rviz_plugin::TrajectoriesDisplay, rviz_common::Display)