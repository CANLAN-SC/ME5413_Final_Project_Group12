# Navigation Package
## Introduction
This navigation package provides navigation configuration for the ME5413 Autonomous Mobile Robots course final project. The package is based on the ROS Navigation Stack, configured to use the TEB (Timed Elastic Band) local planner, and optimized for the Jackal differential-drive robot.

## Features
- Uses TEB local planner for efficient path planning and obstacle avoidance
- Parameters tuned for the Jackal robot in the project environment
- Includes global and local costmap configurations
- Provides both odometry-based and map-based navigation modes
- Integrates multi-sensor data, including lidar and point cloud data

## Dependencies
This package depends on the following ROS packages:
- `roscpp` and `rospy`
- `std_msgs`
- `nav_msgs` and `geometry_msgs`
- `move_base` and `tf2` related packages
- `jackal_navigation`
- `teb_local_planner` (needs to be installed separately)

## Installing teb_local_planner (if not installed)
```bash
sudo apt-get install ros-noetic-teb-local-planner
```

## File Structure
```
navigation/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package dependency definitions
├── README.md               # This document
├── launch/
│   └── move_base.launch    # Launch navigation stack
├── params/
│   ├── base_local_planner_params.yaml    # Base path planner parameters
│   ├── costmap_common_params.yaml        # Common costmap parameters
│   ├── move_base_params.yaml             # move_base configuration
│   ├── teb_local_planner_params.yaml     # TEB local planner parameters
│   ├── map_nav_params/                   # Map-based navigation parameters
│   │   ├── global_costmap_params.yaml
│   │   └── local_costmap_params.yaml
│   └── odom_nav_params/                  # Odometry-based navigation parameters
│       ├── global_costmap_params.yaml
│       └── local_costmap_params.yaml
└── scripts/
    └── send_goal.py                      # Script for sending navigation goals
```

## Usage

### Starting Navigation
This package is typically launched via me5413_world/navigation.launch in the project:

```bash
roslaunch me5413_world navigation.launch
```

Or directly launch the move_base node:

```bash
roslaunch navigation move_base.launch
```

### Sending Navigation Goals

1. Using RVIZ's 2D Nav Goal tool
2. Via script:
```bash
rosrun navigation send_goal.py
```
3. Publishing to a topic:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

## Parameter Adjustments

### Sensor Configuration
In costmap_common_params.yaml, two observation sources are configured:
- `scan`: 2D laser scan data from the topic `/front/scan`
- `lidar`: 3D point cloud data from the topic `/mid/points`

### Key TEB Local Planner Parameters
In teb_local_planner_params.yaml:
- `max_vel_x`: Maximum linear velocity, currently 5.0 m/s
- `max_vel_theta`: Maximum angular velocity, currently 2.0 rad/s
- `min_obstacle_dist`: Minimum obstacle avoidance distance, currently 0.25m
- `inflation_dist`: Obstacle inflation distance for local path planning, currently 0.3m
- `weight_obstacle`: Obstacle weight, currently 50

### Costmap Parameters
Adjust in costmap_common_params.yaml:
- `obstacle_range`: Obstacle detection range, currently 4m
- `raytrace_range`: Ray tracing range, currently 5m
- `inflation_radius`: Obstacle inflation radius, currently 0.30m
- `footprint`: Robot outline, currently set as a rectangle [[-0.21, -0.165], [-0.21, 0.165], [0.21, 0.165], [0.21, -0.165]]
- `footprint_padding`: Additional footprint padding, currently 0.1m

## Common Issues and Solutions

### Path Planning Issues
If the robot cannot plan a path or navigate around obstacles, try:
- Increasing the `inflation_radius` value
- Decreasing the `weight_obstacle` value to make the planner more aggressive
- Ensuring the `footprint` parameter is set accurately

### Sensor Fusion Issues
If sensor data fusion is incorrect:
- Check if coordinate frames for each sensor are set correctly
- Confirm that the `sensor_frame` parameter matches the sensors actually used
- Verify sensor topics are publishing data correctly (`rostopic echo /topic_name`)

### TF Transform Issues
If you encounter TF-related errors:
```
[ WARN] [/move_base]: Transform from map to base_link was unavailable
```
Check the `tf_static` publishing status and robot coordinate frame settings.

---

For questions or improvement suggestions, please contact the maintainer: czhihan@u.nus.edu