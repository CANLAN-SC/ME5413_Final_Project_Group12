# ME5413 Final Project Group12
## Project Introduction

This is the final project of the ME5413 course that implements a robotic system with autonomous navigation, exploration, object detection and OCR capabilities. This system was developed based on ROS (Robot Operating System) and utilizes the Jackal robotics platform, which is capable of performing complex navigation tasks in a virtual environment.

## System Requirements

- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- Python 2.7/3.x
- Library Dependencies:
  - OpenCV
  - pytesseract
  - scikit-learn
  - numpy
  - smach

## Project structure

The project consists of the following modules.

- **fsm**: finite state machine module that coordinates the execution of individual tasks
- **ocr**: Optical character recognition module for recognizing numeric information in the environment.
- **box_detection**: Box detection module to detect and visualize boxes in the environment based on point cloud data.
- **frontier_explore**: Frontier exploration module for autonomous exploration of unknown regions.
- **navigation**: Navigation module, responsible for robot path planning and obstacle avoidance.
- **SLAM**: Synchronized localization and map construction using FAST-LIO.

## Installation steps

1. Create a ROS workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Cloning Project:
```bash
git clone https://github.com/CANLAN-SC/ME5413_Final_Project_Group12.git
```

3. Installation dependencies:
```bash
sudo apt-get update
sudo apt-get install python-opencv python-numpy ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-move-base
sudo apt-get install tesseract-ocr libtesseract-dev
pip install pytesseract scikit-learn
```

4. Compilation workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## operating method

### 1. Start all nodes
```bash
roslaunch fsm final.launch
```
Terminals should be seen to repeat
```bash
[DEBUG] [1743423288.122009188, 605.893000000]: Getting status over the wire.
```
At this time, the simulation environment, rviz, visual recognition, radar detection, navigation, SLAM nodes are activated and can be controlled by the keyboard to simulate the car.

> **Note:** If you encounter `/usr/bin/env: 'python\r': No such file or directory` error, please install and fix it with dos2unix:
> ```bash
> sudo apt-get install dos2unix
> dos2unix src/ocr/scripts/before_bridge_ocr.py
> dos2unix src/ocr/scripts/after_bridge_ocr.py
> ```

### 2. Adding a state machine

Newly opened terminals
```bash
cd ME5413_Final_Project_Group12
source devel/setup.bash
python src/fsm/scripts/fsm.py
```

Should see:
```bash
[INFO] [1743423111.980278, 0.000000]: State machine starting in initial state 'INITIALIZE' with userdata: 
        []
[INFO] [1743423111.982093, 0.000000]: Initializing system...
[INFO] [1743423111.983973, 0.000000]: State machine transitioning 'INITIALIZE':'initialized'-->'NAVIGATE_TO_GOAL'
[INFO] [1743423111.985405, 0.000000]: Mission three...
[INFO] [1743423111.987305, 0.000000]: State machine transitioning 'NAVIGATE_TO_GOAL':'succeeded'-->'TASK_ONE'
[INFO] [1743423111.988715, 0.000000]: Mission one...
[INFO] [1743423111.990916, 0.000000]: Trigger Message Posted
[INFO] [1743423127.133987, 562.406000]: State machine transitioning 'TASK_ONE':'succeeded'-->'TASK_TWO'
[INFO] [1743423127.136749, 562.406000]: Mission two...
[INFO] [1743423127.139615, 562.408000]: State machine transitioning 'TASK_TWO':'succeeded'-->'TASK_THREE'
[INFO] [1743423127.142591, 562.411000]: Mission three...
[INFO] [1743423127.144614, 562.412000]: State machine terminating 'TASK_THREE':'succeeded':'mission_completed'
[INFO] [1743423127.147272, 562.414000]: State machine execution completed, result: mission_completed
```

## Functional Description

- **State Machine Control**: Coordinates the order of execution of tasks such as navigation, exploration, and detection.
- **Autonomous exploration**: explore unknown areas using frontier_explore
- **Box detection**: detects boxes in the environment using the DBSCAN clustering algorithm.
- **OCR Recognition**: use Tesseract-OCR to recognize numbers in the environment for bridge unlocking
- **SLAM**: Use FAST-LIO for environment mapping and localization
- **Autonomous Navigation**: using move_base for autonomous robot navigation.

## System Architecture

The system consists of several modules, mainly including.

1. **Main startup file**: final.launch
2. **Sensor processing**: fast_lio for laser SLAM
3. **Target detection**: box_detection for box detection, ocr for character recognition
4. **Path planning**: move_base and TEB planner
5. **frontier_explore**: frontier_explore to realize unknown region exploration
6. **task coordination**: fsm state machine to coordinate the execution of each task

```mermaid
graph LR
    %% 主启动文件
    final[final.launch] 
    
    %% 包含的其他launch文件
    world[world.launch]
    manual[manual.launch]
    ocr_launch[ocr.launch]
    box_detection_launch[box_detection.launch]
    frontier_explore_launch[frontier_explore.launch]
    move_base_launch[move_base.launch]
    fast_lio_launch[fast_lio.launch]
    masked_costmap_launch[masked_costmap.launch]
    frontier_filter_launch[frontier_filter.launch]
    
    %% 节点
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
    
    %% FSM状态机节点和状态
    fsm_node[task_coordinator]
    init_state[Initialize]
    nav_to_explore_state[NavigateToExplorationArea]
    explore_state[ExploreFrontier]
    detect_bridge_state[DetectBridge]
    nav_to_goal_state[NavigateToGoal]
    
    %% 话题
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
    
    %% TF坐标系
    tf_base_link((base_link))
    tf_map((map))
    tf_odom((odom))
    tf_lidar((lidar_link))
    
    %% Launch文件层次结构
    final --> world
    final --> manual
    final --> ocr_launch
    final --> box_detection_launch
    final --> frontier_explore_launch
    final --> move_base_launch
    final --> fast_lio_launch
    
    %% launch文件启动的节点
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
    
    %% FSM内部状态关系
    fsm_node --> init_state
    init_state -->|initialized| nav_to_explore_state
    init_state -->|failed| mission_failed[任务失败]
    nav_to_explore_state -->|succeeded| explore_state
    nav_to_explore_state -->|failed/preempted| mission_failed
    explore_state -->|succeeded| detect_bridge_state
    explore_state -->|failed| mission_failed
    detect_bridge_state -->|succeeded| nav_to_goal_state
    detect_bridge_state -->|failed| mission_failed
    nav_to_goal_state -->|succeeded| mission_completed[任务完成]
    nav_to_goal_state -->|failed| mission_failed
    
    %% 话题订阅与发布关系
    box_detection_node -- 订阅 --> mid_points
    box_detection_node -- 发布 --> detected_boxes
    box_detection_node -- 发布 --> bounding_boxes
    
    ocr_node -- 订阅 --> front_image
    ocr_node -- 订阅 --> ocr_trigger
    ocr_node -- 订阅 --> cmd_open_bridge
    ocr_node -- 发布 --> recognized_digit
    ocr_node -- 发布 --> mode_digit
    
    masked_costmap_node -- 订阅 --> map_topic
    masked_costmap_node -- 发布 --> masked_costmap
    
    frontier_filter_node -- 订阅 --> explore_frontiers
    frontier_filter_node -- 订阅 --> move_base_goal
    frontier_filter_node -- 发布 --> filtered_frontiers
    frontier_filter_node -- 发布 --> move_base_cancel
    
    explore_node -- 订阅 --> global_costmap
    explore_node -- 发布 --> explore_frontiers
    explore_node -- 发布 --> move_base_goal
    
    move_base_node -- 订阅 --> odometry_filtered
    move_base_node -- 订阅 --> map_topic
    move_base_node -- 发布 --> cmd_vel
    move_base_node -- 发布 --> global_costmap
    move_base_node -- 发布 --> move_base_result
    
    teleop -- 发布 --> cmd_vel
    
    fast_lio_node -- 订阅 --> mid_points
    fast_lio_node -- 发布 --> map_topic
    fast_lio_node -- 发布 --> odometry_filtered
    
    %% FSM与其他节点的交互
    explore_state -- 发布 --> ocr_trigger
    nav_to_explore_state -- 发布 --> move_base_goal
    nav_to_explore_state -- 订阅 --> move_base_result
    nav_to_goal_state -- 发布 --> move_base_goal
    nav_to_goal_state -- 订阅 --> move_base_result
    detect_bridge_state -- 订阅 --> detected_boxes
    detect_bridge_state -- 发布 --> cmd_open_bridge
    
    %% TF坐标转换关系
    jackal -- 发布 --> tf_base_link
    jackal -- 发布 --> tf_lidar
    fast_lio_node -- 发布 --> tf_map
    fast_lio_node -- 发布 --> tf_odom
    
    tf_map -- 转换 --> tf_odom
    tf_odom -- 转换 --> tf_base_link
    tf_base_link -- 转换 --> tf_lidar
    
    %% 样式设置
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
## Troubleshooting
### Where do the target position box coordinates come from?

These box coordinates are derived by looking at the formula in the code. The calculations are shown below:

#### Key calculations in the source code

There is code like this in the `spawnRandomBoxes()` function (lines 197-202):

```cpp
const double spacing = (MAX_X_COORD - MIN_X_COORD)/(box_labels.size() + 1);
for (int i = 0; i < box_labels.size(); i++)
{
  const ignition::math::Vector3d point = ignition::math::Vector3d(spacing*(i + 1) + MIN_X_COORD, 0.0, Z_COORD);
  // 后续代码...
}
```

#### Constant values used 

The following constants are defined at the top of the file:
- `NUM_BOX_TYPES = 4` (indicates the use of 4 different types of boxes)
- `MIN_X_COORD = 2.0`
- `MAX_X_COORD = 22.0`
- `Z_COORD = 3.0`

#### Calculation procedure

1. first the number of box types is taken as `NUM_BOX_TYPES = 4` (the result of cropping the `box_labels` vector in lines 96-97)
2. calculate the spacing: `spacing = (MAX_X_COORD - MIN_X_COORD)/(box_labels.size() + 1)`
   - `spacing = (22.0 - 2.0)/(4 + 1) = 20.0/5 = 4.0`. 3.
3. Then calculate the position of each box: `point_x = spacing*(i + 1) + MIN_X_COORD`.

For each of the 4 boxes:
- Box 1 (i=0): `4.0*(0+1) + 2.0 = 6.0`
- box 2 (i=1): `4.0*(1+1) + 2.0 = 10.0`
- Box 3 (i=2): `4.0*(2+1) + 2.0 = 14.0`
- Box 4 (i=3): `4.0*(3+1) + 2.0 = 18.0`
所有盒子的Y坐标都是0.0，Z坐标都是3.0。

### What is the law of generation of random boxes?
Analyze the provided `object_spawner_gz_plugin.cpp` code to get the generation law of random boxes in the explored region.

#### Box generation area and basic parameters

The area for box generation is defined as a rectangular area with the range:
- X coordinate range: 2.0 to 22.0 (MIN_X_COORD to MAX_X_COORD)
- Y coordinate range: 11.0 to 19.0 (MIN_Y_COORD to MAX_Y_COORD)
- Z coordinate fixed: 3.0 (Z_COORD)

#### Type and number of boxes

There are several key variables defined in the code to control the generation of boxes:

1. `NUM_BOX_TYPES = 4`: 4 different types of boxes will be generated
2. `box_labels` array: contains the possible box labels (numbers between 1 and 9)
3. `box_nums` array: defines the number of boxes of each type

#### Generation process

The generation process of the box is as follows:

1. **Randomization process**:
   - Randomly shuffle `box_labels` and `box_nums` using a randomization device
   - The first `NUM_BOX_TYPES` elements are selected as the final labels and quantities

2. **Random box generation**:
   - Generate random boxes according to the number of each type defined in the `box_nums` array
   - Box positions are randomly selected within a defined range
   - Generation ensures that there are no collisions between boxes (minimum distance of 1.2 units)

#### Key Rules

1. **Collision avoidance**: when a box is generated at a random location, the distance to the generated box is checked to ensure that it is separated by at least 1.2 units
   ```cpp
   for (const auto& pre_point : this->box_points)
   {
     const double dist = (point - pre_point).Length();
     if (dist <= 1.2)
     {
       has_collision = true;
       break;
     }
   }
   ```

2. **Labeling and quantity matching**:
   - Each label type generates a corresponding quantity of boxes in the exploration area
   - For example, if label [1,3,5,7] and quantity [2,1,3,1] are selected, then 2 boxes labeled 1 will be generated, 1 box labeled 3, and so on

3. **unique solution**:
   ``cpp
   // The comment in the code indicates that the quantity array should ensure that there is only one solution
   std::vector<int> box_nums = {1, 2, 3, 4, 5}; // can contain any positive number, but make sure there's only one solution
   ``

## Contributors

- Group 12 Members

## License

Detailed in the LICENSE file
