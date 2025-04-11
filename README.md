# ME5413 Final Project Group12

## Contributors

- [**Cao Zhihan**](https://github.com/dragonundertheworld) - *Project Coordination, State Machine Design and Object Detection* 
- [**Ni Haozhan**](https://github.com/nihaozhan) - *Frontier Exploration* 
- [**Sun Cheng**](https://github.com/CANLAN-SC) - *OCR Implementation*
- [**Wei Yangcheng**](https://github.com/Wei819cn) - *Navigation Implementation* 
- [**Wu Rui**](https://github.com/Avory1258) - *SLAM and Map Building* 
- [**Huang Xinchen**](https://github.com/koallaaa) - *SLAM and Map Building* 

## Project Introduction

This project presents the development of an autonomous mobile robotic system for the Jackal platform to perform complex navigation and task execution in a simulated Gazebo environment. The robot is required to map the environment, avoid obstacles, explore the randomly placed boxes, detect the number on the box, cross a dynamically generated bridge, unlock a timed blockade, and dock in front of the least frequently occurring box. We implemented a SLAM-based mapping module evaluated against ground truth, a sampling-based 2D localization module, a navigation module with global and local planner, a region-limited frontier exploration strategy, a real-time perception module and a general decision-making framework for robust task coordination. The integrated system demonstrates reliable performance and adaptability in dynamic scenarios.

## Demo
![project demo](./media/final.gif)

## System Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 2.7/3.8
- Library Dependencies:
  - OpenCV
  - pytesseract
  - scikit-learn
  - numpy
  - smach

## Project Structure

The project mainly contains the following modules:

- **fsm**: Finite state machine module, coordinating the execution of various tasks
- **ocr**: Optical character recognition module, for recognizing numerical information in the environment
- **box_detection**: Box detection module, detecting and visualizing boxes in the environment based on point cloud data
- **frontier_explore**: Frontier exploration module, implementing autonomous exploration of unknown areas
- **navigation**: Navigation module, responsible for the robot's path planning and obstacle avoidance
- **SLAM**: Using FAST-LIO for simultaneous localization and mapping

## Installation Steps

1. Create a ROS workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Clone the project:
```bash
git clone https://github.com/CANLAN-SC/ME5413_Final_Project_Group12.git
```

3. Install dependencies:
```bash
sudo apt-get update
sudo apt-get install python-opencv python-numpy tesseract-ocr libtesseract-dev
pip install pytesseract scikit-learn
sudo apt-get install ros-$ROS_DISTRO-smach ros-$ROS_DISTRO-smach-ros ros-$ROS_DISTRO-explore-lite ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-move-base ros-noetic-teb-local-planner ros-noetic-navigation
```

4. Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Running Instructions

### 1. Launch all nodes

```bash
roslaunch fsm final.launch
```
You should see the terminal repeatedly showing:
```bash
[DEBUG] [1743423288.122009188, 605.893000000]: Getting status over the wire.
```
At this point, the simulation environment, rviz, visual recognition, lidar detection, navigation, and SLAM nodes are all launched and the simulated robot can be controlled via keyboard.

> **Note:** If you encounter the error `/usr/bin/env: 'python\r': No such file or directory`, please install and use dos2unix to fix it:
> ```bash
> sudo apt-get install dos2unix
> dos2unix src/ocr/scripts/before_bridge_ocr.py
> dos2unix src/ocr/scripts/after_bridge_ocr.py
> ```

### 2. **After exploration begins**, join the state machine

Open a new terminal:
```bash
cd ME5413_Final_Project_Group12
source devel/setup.bash
python src/fsm/scripts/fsm.py
```

You should see:
```bash
[INFO] [1744190511.107207, 545.144000]: Navigation client connected
[INFO] [1744190511.114783, 545.147000]: Navigation client connected
[INFO] [1744190511.129417, 545.163000]: State machine starting in initial state 'INITIALIZE' with userdata: 
	['costmap']
[INFO] [1744190511.130435, 545.164000]: State machine transitioning 'INITIALIZE':'initialized'-->'EXPLORE_FRONTIER'
[INFO] [1744190511.131264, 545.165000]: Starting frontier exploration task...
[INFO] [1744190511.132374, 545.165000]: Starting to monitor frontier point count, threshold is 0
[INFO] [1744190511.136510, 545.165000]: Map data ready, starting exploration...
```

## Feature Description

- **State Machine Control**: Coordinates the execution sequence of navigation, exploration, detection, and other tasks
- **Autonomous Exploration**: Uses frontier_explore to explore unknown areas
- **Box Detection**: Uses DBSCAN clustering algorithm to detect boxes in the environment
- **OCR Recognition**: Recognizes numbers in the environment, used for bridge unlocking
- **SLAM**: Uses FAST-LIO for environment mapping and localization
- **Autonomous Navigation**: Implements autonomous navigation using move_base

## System Architecture

```plaintext
.
├── docs -> Project documentation and references
├── media -> Images, videos, gifs and other media files
└── src
    ├── frontier_explore -> Autonomous exploration module for unknown areas
    │   ├── launch -> Launch files for frontier exploration
    │   └── scripts -> Python scripts for frontier exploration algorithms
    ├── fsm -> Finite State Machine for task coordination
    │   ├── launch -> Launch files for the state machine
    │   └── scripts -> Python scripts implementing the state machine logic
    ├── interactive_tools -> Tools for user interaction with the robot
    │   ├── include -> C++ header files for interactive tools
    │   ├── launch -> Launch files for interactive tools
    │   ├── resource -> Resource files for interactive tools
    │   ├── rviz -> RViz configuration files
    │   └── src -> C++ source files for interactive tools
    ├── jackal_description -> Robot description files for the Jackal platform
    │   ├── launch -> Launch files for the robot model
    │   ├── meshes -> 3D model files for visualization
    │   ├── scripts -> Helper scripts for the robot model
    │   └── urdf -> URDF robot description files
    ├── me5413_world -> Simulation world configuration
    │   ├── config -> Configuration files for the simulation
    │   ├── include -> C++ header files for world plugins
    │   ├── launch -> Launch files for the simulation world
    │   ├── media -> Media files specific to the simulation
    │   ├── models -> 3D models for simulation elements
    │   ├── rviz -> RViz configuration for world visualization
    │   ├── src -> C++ source files for world plugins
    │   └── worlds -> Gazebo world definition files
    ├── navigation -> Navigation and path planning module
    │   ├── launch -> Launch files for navigation
    │   ├── params -> Parameter files for navigation algorithms
    │   └── scripts -> Python scripts for navigation tasks
    ├── ocr -> Optical Character Recognition module
    │   ├── docs -> Documentation specific to OCR
    │   ├── launch -> Launch files for OCR nodes
    │   └── scripts -> Python scripts for OCR implementation
    ├── slam -> Simultaneous Localization and Mapping module
    │   ├── bagfiles -> ROS bag files for testing and replay
    │   ├── config -> Configuration files for SLAM algorithms
    │   ├── launch -> Launch files for SLAM
    │   ├── maps -> Generated and reference maps
    │   ├── rviz -> RViz configuration for map visualization
    │   └── scripts -> Python scripts for map processing
    └── third_party -> External dependencies and libraries
        ├── FAST_LIO -> Fast LiDAR-Inertial Odometry implementation
        ├── livox_ros_driver -> Driver for Livox LiDAR sensors
        └── pcd_to_map -> Tool for converting point clouds to maps
```

```mermaid
graph LR
    %% Main launch file
    final[final.launch] 
    
    %% Included launch files
    world[world.launch]
    manual[manual.launch]
    ocr_launch[ocr.launch]
    box_detection_launch[box_detection.launch]
    frontier_explore_launch[frontier_explore.launch]
    move_base_launch[move_base.launch]
    fast_lio_launch[fast_lio.launch]
    masked_costmap_launch[masked_costmap.launch]
    frontier_filter_launch[frontier_filter.launch]
    
    %% Nodes
    gazebo[gazebo_ros]
    jackal[spawn_jackal]
    teleop[teleop_twist_keyboard]
    rviz[rviz]
    box_detection_node[box_detection]
    ocr_node[ocr_node/before_bridge_ocr.py]
    explore_node[explore_lite/explore]
    masked_costmap_node[masked_costmap.py]
    frontier_filter_node[frontier_filter.py]
    move_base_node[move_base]
    fast_lio_node[fastlio_mapping]
    
    %% FSM state machine node and states
    fsm_node[task_coordinator]
    init_state[Initialize]
    nav_to_explore_state[NavigateToExplorationArea]
    explore_state[ExploreFrontier]
    detect_bridge_state[DetectBridge]
    nav_to_goal_state[NavigateToGoal]
    
    %% Topics
    mid_points[/mid/points\]
    detected_boxes[/detected_boxes\]
    bounding_boxes[/bounding_boxes\]
    front_image[/front/image_raw\]
    ocr_trigger[/ocr_trigger\]
    cmd_open_bridge[/cmd_open_bridge\]
    recognized_digit[/recognized_digit\]
    mode_digit[/mode_digit\]
    map_topic[/map\]
    masked_costmap[/masked_costmap\]
    explore_frontiers[/explore/frontiers\]
    filtered_frontiers[/filtered_frontiers\]
    move_base_goal[/move_base/goal\]
    move_base_result[/move_base/result\]
    move_base_cancel[/move_base/cancel\]
    odometry_filtered[/odometry/filtered\]
    cmd_vel[/cmd_vel\]
    global_costmap[/move_base/global_costmap/costmap\]
    
    %% TF coordinate systems
    tf_base_link((base_link))
    tf_map((map))
    tf_odom((odom))
    tf_lidar((lidar_link))
    
    %% Launch file hierarchy
    final --> world
    final --> manual
    final --> ocr_launch
    final --> box_detection_launch
    final --> frontier_explore_launch
    final --> move_base_launch
    final --> fast_lio_launch
    
    %% Nodes launched by launch files
    world --> gazebo
    world --> jackal
    manual --> teleop
    manual --> rviz
    ocr_launch --> ocr_node
    box_detection_launch --> box_detection_node
    frontier_explore_launch --> explore_node
    frontier_explore_launch --> masked_costmap_launch
    frontier_explore_launch --> frontier_filter_launch
    masked_costmap_launch --> masked_costmap_node
    frontier_filter_launch --> frontier_filter_node
    move_base_launch --> move_base_node
    fast_lio_launch --> fast_lio_node
    fast_lio_launch --> teleop
    
    %% FSM internal state relations
    fsm_node --> init_state
    init_state -->|initialized| nav_to_explore_state
    init_state -->|failed| mission_failed[Mission Failed]
    nav_to_explore_state -->|succeeded| explore_state
    nav_to_explore_state -->|failed/preempted| mission_failed
    explore_state -->|succeeded| detect_bridge_state
    explore_state -->|failed| mission_failed
    detect_bridge_state -->|succeeded| nav_to_goal_state
    detect_bridge_state -->|failed| mission_failed
    nav_to_goal_state -->|succeeded| mission_completed[Mission Completed]
    nav_to_goal_state -->|failed| mission_failed
    
    %% Topic subscription and publishing relations
    box_detection_node -- subscribes --> mid_points
    box_detection_node -- publishes --> detected_boxes
    box_detection_node -- publishes --> bounding_boxes
    
    ocr_node -- subscribes --> front_image
    ocr_node -- subscribes --> ocr_trigger
    ocr_node -- subscribes --> cmd_open_bridge
    ocr_node -- publishes --> recognized_digit
    ocr_node -- publishes --> mode_digit
    
    masked_costmap_node -- subscribes --> map_topic
    masked_costmap_node -- publishes --> masked_costmap
    
    frontier_filter_node -- subscribes --> explore_frontiers
    frontier_filter_node -- subscribes --> move_base_goal
    frontier_filter_node -- publishes --> filtered_frontiers
    frontier_filter_node -- publishes --> move_base_cancel
    
    explore_node -- subscribes --> global_costmap
    explore_node -- publishes --> explore_frontiers
    explore_node -- publishes --> move_base_goal
    
    move_base_node -- subscribes --> odometry_filtered
    move_base_node -- subscribes --> map_topic
    move_base_node -- publishes --> cmd_vel
    move_base_node -- publishes --> global_costmap
    move_base_node -- publishes --> move_base_result
    
    teleop -- publishes --> cmd_vel
    
    fast_lio_node -- subscribes --> mid_points
    fast_lio_node -- publishes --> map_topic
    fast_lio_node -- publishes --> odometry_filtered
    
    %% FSM interaction with other nodes
    explore_state -- publishes --> ocr_trigger
    nav_to_explore_state -- publishes --> move_base_goal
    nav_to_explore_state -- subscribes --> move_base_result
    nav_to_goal_state -- publishes --> move_base_goal
    nav_to_goal_state -- subscribes --> move_base_result
    detect_bridge_state -- subscribes --> detected_boxes
    detect_bridge_state -- publishes --> cmd_open_bridge
    
    %% TF coordinate transformation relations
    jackal -- publishes --> tf_base_link
    jackal -- publishes --> tf_lidar
    fast_lio_node -- publishes --> tf_map
    fast_lio_node -- publishes --> tf_odom
    
    tf_map -- transforms --> tf_odom
    tf_odom -- transforms --> tf_base_link
    tf_base_link -- transforms --> tf_lidar
    
    %% Style settings
    classDef launch fill:#f9d,stroke:#333,stroke-width:2px;
    classDef node fill:#bbf,stroke:#333,stroke-width:2px;
    classDef state fill:#aaf,stroke:#333,stroke-width:2px;
    classDef topic fill:#ddd,stroke:#333,stroke-width:1px;
    classDef tf fill:#ffd700,stroke:#333,stroke-width:2px;
    classDef outcome fill:#d9f,stroke:#333,stroke-width:1px;
    
    class final,world,manual,ocr_launch,box_detection_launch,frontier_explore_launch,move_base_launch,fast_lio_launch,masked_costmap_launch,frontier_filter_launch launch;
    class gazebo,jackal,teleop,rviz,box_detection_node,ocr_node,explore_node,masked_costmap_node,frontier_filter_node,move_base_node,fast_lio_node,fsm_node node;
    class init_state,nav_to_explore_state,explore_state,detect_bridge_state,nav_to_goal_state state;
    class mid_points,detected_boxes,bounding_boxes,front_image,ocr_trigger,cmd_open_bridge,recognized_digit,mode_digit,map_topic,masked_costmap,explore_frontiers,filtered_frontiers,move_base_goal,move_base_result,move_base_cancel,odometry_filtered,cmd_vel,global_costmap topic;
    class tf_base_link,tf_map,tf_odom,tf_lidar tf;
    class mission_completed,mission_failed outcome;
```

## Appendix
### ROS Graph
![ROS Graph](./media/rosgraph.png)

### TF Tree
![TF Tree](./media/frames.png)
