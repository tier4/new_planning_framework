# autoware_new_planning_rviz_plugin

## Overview

This package provides RViz plugins for visualizing messages from the new planning framework, specifically the `autoware_new_planning_msgs::Trajectories` message type.

## Features

### Trajectories Display

The Trajectories display plugin visualizes multiple trajectory candidates from different trajectory generators.

#### Display Options

- **View All Trajectories**: Toggle between displaying all trajectories or a single selected trajectory
- **Selected Trajectory**: When not viewing all, select which trajectory index to display
- **Color Scheme**: Choose how trajectories are colored:
  - By Score: Best scoring trajectory in one color, others in another
  - By Generator: Each generator gets a unique color
  - Uniform: All trajectories in the same color

#### Visualization Settings

- **Line Width**: Width of trajectory lines
- **Alpha**: Transparency of trajectories
- **Show Points**: Display individual trajectory points
- **Point Size**: Size of trajectory points when displayed

#### Text Display

- **Show Generator Info**: Display generator names at trajectory start
- **Generator Text Scale**: Size of generator name text
- **Show Scores**: Display trajectory scores
- **Score Text Scale**: Size of score text

#### Colors

- **Best Trajectory Color**: Color for the highest scoring trajectory (when using "By Score" scheme)
- **Other Trajectories Color**: Color for other trajectories

## Usage

1. Build the package:
   ```bash
   colcon build --packages-select autoware_new_planning_rviz_plugin
   ```

2. Launch RViz2

3. Add the display:
   - Click "Add" button
   - Select "By topic" tab
   - Find your trajectories topic
   - Select "Trajectories" and click OK

4. Configure the display properties as needed

## Message Format

The plugin visualizes `autoware_new_planning_msgs::Trajectories` messages which contain:
- An array of trajectories, each with:
  - Header with timestamp and frame
  - Generator ID (UUID)
  - Array of trajectory points
  - Score value
- Generator information mapping IDs to names