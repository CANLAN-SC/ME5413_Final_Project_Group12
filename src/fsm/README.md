# Introduction

This is a ROS node implemented using the SMACH state machine framework to coordinate the execution sequence of various robot tasks. This state machine is responsible for managing transitions between multiple task states and provides a visualization interface for debugging and monitoring.

# Main Features

- Uses state machine framework to manage complex task workflows
- Implements autonomous navigation to specified locations
- Triggers OCR recognition system for processing
- Detects boxes in the environment and processes related information
- Manages specific tasks such as bridge detection
- Provides task execution status visualization

# State Machine Architecture

The state machine contains the following main states:

1. **Initialize**: System initialization state
   - Responsible for initial system setup
   - Transitions to NavigateToGoal state on success

2. **NavigateToGoal**: Navigate to target point
   - Uses move_base action interface to navigate to specified coordinates
   - Monitors navigation results and transitions states accordingly

3. **ExploreFrontier (TASK_ONE)**: Execute task one
   - Publishes trigger messages to the `/ocr_trigger` topic
   - Subscribes to the `/detected_boxes` topic to receive box position information
   - Processes and records detected boxes

4. **DetectBridge (TASK_TWO)**: Execute task two
   - Handles tasks related to bridge detection

5. **NavigateToGoal (TASK_THREE)**: Execute task three
   - Performs navigation tasks after task completion
   - Completes the entire task workflow

# Usage

1. Ensure required dependencies are installed:
   ```bash
   sudo apt-get install ros-$ROS_DISTRO-smach ros-$ROS_DISTRO-smach-ros 
   ```

2. Launch the node in terminal:
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   python src/fsm/scripts/fsm.py
   ```

3. View state machine execution:
   ```bash
   rosrun smach_viewer smach_viewer.py
   ```

# Interaction with Other Nodes

- Communicates with `move_base` node to perform navigation tasks
- Publishes messages to `/ocr_trigger` topic to trigger OCR recognition
- Receives detected box information from `/detected_boxes` topic

# State Transition Logic

```
Initialize ──success──→ NavigateToGoal ──success──→ TASK_ONE ──success──→ TASK_TWO ──success──→ TASK_THREE ──success──→ mission_completed
     │                     │                  │                 │                 │
     └───failure───┐   ┌───failure───┐      ┌───failure───┐     ┌───failure───┐     ┌───failure───┐
                   ▼   ▼             ▼      ▼             ▼     ▼             ▼     ▼
                 mission_failed
```

# Data Flow
## Analysis of box_position Transmission in Task State Machine

The transmission of `box_position` in your state machine is implemented through the `userdata` mechanism of the SMACH state machine framework. Let me outline the complete process:

### 1. Data Generation - DetectBoxPose State

In the `DetectBoxPose` state, `box_positions` are generated through the following steps:

1. Extract obstacle information from the cost map
2. Use DBSCAN clustering algorithm to cluster obstacle points
3. Calculate the center point of each cluster and convert to positions in map coordinate system
4. Filter out boxes within the exploration area based on configured boundaries
5. Package detected box positions as a `PoseArray` message

Key code:
```python
# Create box pose array as PoseArray for transmission
pose_array = PoseArray()
pose_array.header.frame_id = "map"
pose_array.header.stamp = rospy.Time.now()
pose_array.poses = [box.pose for box in costmap_boxes]
userdata.box_positions_out = pose_array
```

### 2. Data Output Definition

Output keys are declared in the initialization of the `DetectBoxPose` state:
```python
smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                     output_keys=['box_positions_out'])
```

### 3. Data Mapping in State Machine

In the top-level state machine, data is mapped between states using the `remapping` parameter:
```python
smach.StateMachine.add('DETECT_BOX_POSE', DetectBoxPose(), 
                      transitions={'succeeded':'NAVIGATE_TO_BOX_AND_OCR', 
                                 'failed':'mission_failed'},
                      remapping={'box_positions_out':'box_positions'})
```

This maps the `box_positions_out` output key from the `DetectBoxPose` state to the state machine's `box_positions` variable.

### 4. Data Reception - NavigateToBoxAndOCR State

In the `NavigateToBoxAndOCR` state, box position data is received as follows:

```python
smach.State.__init__(self, 
                   outcomes=['succeeded', 'failed'], 
                   input_keys=['box_positions_in'])
```

And mapped in the state machine:
```python
smach.StateMachine.add('NAVIGATE_TO_BOX_AND_OCR', NavigateToBoxAndOCR(),
                       transitions={'succeeded':'DETECT_BRIDGE', 
                                   'failed':'mission_failed'},
                       remapping={'box_positions_in':'box_positions'})
```

### 5. Data Usage - In NavigateToBoxAndOCR State

In the `NavigateToBoxAndOCR::execute` method, box positions are retrieved from `userdata`:
```python
# Get box positions from userdata
self.box_positions = userdata.box_positions_in
```

These position data are then used for navigation and OCR processing:
```python
# Navigate to each box and perform OCR
for i, pose in enumerate(self.box_positions.poses):
    try:
        self.navigate_to_best_viewing_positions(pose)
        
    except Exception as e:
        rospy.logerr('Error processing box: %s', str(e))
```

### 6. Data Type and Format

- **Data Type**: `PoseArray` message type, containing multiple `Pose` objects
- **Structure**: 
  - `header`: Standard message header with timestamp and coordinate system information
  - `poses[]`: Array of multiple `Pose` objects, each containing position(x,y,z) and orientation(quaternion)

### 7. Data Flow in ROS System

Besides internal state machine transmission, `box_positions` is also published via ROS topics:

```python
self.box_publisher.publish(pose_array)
```

This allows other nodes to subscribe to and use box position information, such as visualization tools or external monitoring nodes.

## Summary

The data flow is as follows:
1. `DetectBoxPose` detects box positions from cost map
2. Positions are packaged as `PoseArray` and stored in `userdata.box_positions_out`
3. Through the state machine's `remapping` mechanism, data is mapped to the shared variable `box_positions`
4. `NavigateToBoxAndOCR` retrieves data from `userdata.box_positions_in`
5. Uses these positions to navigate to each box and perform OCR processing
6. Simultaneously, data is published via ROS topics for use by other components in the system

This design allows different states to share data while maintaining loose coupling between states.

# Notes

- Code contains commented-out functionality for launching launch files, uncomment relevant sections if needed
- Navigation goal coordinates are hardcoded in the `NavigateToExplorationArea` class and can be modified according to actual requirements
- State machine visualization service runs in the `/TASK_COORDINATOR` namespace

# Extension and Modification

To add new task states, follow these steps:
1. Create a new state class inheriting from `smach.State`
2. Implement the `execute()` method to handle task logic
3. Use `smach.StateMachine.add()` to add it to the state machine in the `main()` function
4. Update transition relationships for relevant states

For more information about SMACH state machines, visit [ROS SMACH Tutorials](http://wiki.ros.org/smach/Tutorials)

# Troubleshooting
## What are the sources of `/move_base/global_costmap/costmap` and `/move_base/global_costmap/costmap_updates`?

Both topics are core components of the ROS navigation stack, specifically generated by the `global_costmap` functionality in the `move_base` node:

### `/move_base/global_costmap/costmap`

**Source**: This topic is created and published by the `global_costmap` object within the `move_base` node.

**Content**: Complete global costmap, which is the result of the map provided by the SLAM system overlaid with various cost layers.

**Data flow**:
1. SLAM system (like gmapping, hector_slam, cartographer, or FAST-LIO that you're using) generates the base map, typically published on the `/map` topic
2. `move_base` subscribes to this map and uses it as the static layer for the `global_costmap`
3. `move_base` adds other layers (such as obstacle layer, inflation layer, etc.) to get the final global costmap
4. This complete map is published to `/move_base/global_costmap/costmap`

### `/move_base/global_costmap/costmap_updates`

**Source**: Also created and published by the `global_costmap` object in the `move_base` node.

**Content**: Contains only the recently changed parts of the costmap, an incremental update mechanism.

**Data flow**:
1. When sensor data causes changes in the costmap, `move_base` calculates the changed areas
2. These changes are packaged as a small OccupancyGrid message, containing only the modified areas
3. This incremental update is published to `/move_base/global_costmap/costmap_updates`

## What are the triggers?

### Published Trigger Messages

1. **`/box_detection_trigger`**
   - Type: `std_msgs/Bool`
   - Publisher: `DetectBoxPose` state
   - Purpose: Triggers external box detection node to start detecting boxes in the scene
   - Data: `True` indicates start of detection

2. **`/ocr_trigger`**
   - Type: `std_msgs/Bool`
   - Publisher: `NavigateToBoxAndOCR` and `NavigateToGoalAndOCR` states
   - Purpose: Triggers OCR node to start recognizing text in images
   - Data: `True` indicates start of OCR recognition
   - Command line test: `rostopic pub /ocr_trigger std_msgs/Bool "data: true" -1`

3. **`/bridge_detection_trigger`**
   - Type: `std_msgs/Bool`
   - Publisher: `DetectBridge` state
   - Purpose: Triggers bridge detection node to start detecting bridges in the scene
   - Data: `True` indicates start of bridge detection

4. **`/open_bridge`**
   - Type: `std_msgs/Bool`
   - Publisher: `OpenBridgeAndNavigate` state
   - Purpose: Triggers bridge control node to open the bridge
   - Data: `True` indicates open the bridge

5. **`/move_base/global_costmap/costmap`** and **`/move_base/global_costmap/costmap_updates`**
   - Type: `nav_msgs/OccupancyGrid`
   - Publisher: `ExploreFrontier` state
   - Purpose: Publishes cost map data of exploration area for navigation planning

### Subscribed Messages

1. **`/detected_boxes`**
   - Type: `geometry_msgs/PoseArray`
   - Subscriber: `DetectBoxPose` state
   - Purpose: Receives box location information discovered by box detection node
   - Callback: `box_callback`, stores detected box positions in a list

2. **`/detected_bridges`**
   - Type: `geometry_msgs/PoseStamped`
   - Subscriber: `DetectBridge` state
   - Purpose: Receives bridge entrance location information discovered by bridge detection node
   - Callback: `bridge_callback`, records detected bridge positions

3. **`/cmd_stop`**
   - Type: `std_msgs/Bool`
   - Subscriber: `NavigateToGoalAndOCR` state
   - Purpose: Receives stop signal sent by OCR node, indicating the target box has been found
   - Callback: `cmd_stop_callback`, if `True` is received, marks the task as complete and stops navigation

### Trigger Process

1. Initialize → Navigate to exploration area
2. Perform frontier exploration, publish map data
3. Trigger box detection, receive box positions
4. Navigate to each box and trigger OCR recognition
5. Trigger bridge detection, receive bridge positions
6. Navigate to bridge entrance
7. Send bridge opening command and navigate across the bridge
8. Navigate to target area and trigger OCR until stop signal is received

## Latest Progress
Known box positions + navigation
```bash
[INFO] [1743674765.743773, 512.718000]: Waiting to receive box position data...
[INFO] [1743674767.528165, 513.218000]: Navigating to 9 boxes and starting OCR...
[INFO] [1743674767.529933, 513.220000]: [1/9] Navigating to box location: x=15.37, y=-16.05
```