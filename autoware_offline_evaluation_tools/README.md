# Offline Evaluation Tools

## Usage

```sh
ros2 launch autoware_offline_evaluation_tools offline_evaluator.launch.xml bag_path:=<ROSBAG> map_path:=<MAP> vehicle_model:=<VEHICLE> sensor_model:=<SENSOR>
```

## Output

| Name                      | Type                                              | Description                                                     |
| ------------------------- | ------------------------------------------------- | --------------------------------------------------------------- |
| `~/output/manual_metrics` | `tier4_debug_msgs::msg::Float32MultiArrayStamped` | Metrics calculated from the driver's driving trajectory.        |
| `~/output/system_metrics` | `tier4_debug_msgs::msg::Float32MultiArrayStamped` | Metrics calculated from the autoware output.                    |
| `~/output/manual_score`   | `tier4_debug_msgs::msg::Float32MultiArrayStamped` | Driving scores calculated from the driver's driving trajectory. |
| `~/output/system_score`   | `tier4_debug_msgs::msg::Float32MultiArrayStamped` | Driving scores calculated from the autoware output.             |
