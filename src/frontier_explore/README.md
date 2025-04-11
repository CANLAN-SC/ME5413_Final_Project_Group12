# frontier_explore
## Overview
This package provides greedy frontier-based exploration functionality. When the node is running, the robot will greedily explore its environment until no more frontiers are found. Movement commands will be sent to move_base.

Unlike similar packages, explore_lite does not create its own costmap, making it easier to configure and more efficient (using fewer resources). The node simply subscribes to nav_msgs/OccupancyGrid messages. Robot movement commands are sent to the move_base node.

The node can perform frontier filtering and can operate on non-inflated maps. The goal blacklist feature allows handling places that the robot cannot reach.

## Architecture
explore_lite uses move_base for navigation. You need to run a properly configured move_base node.

![explore_lite architecture diagram](http://wiki.ros.org/explore_lite?action=AttachFile&do=get&target=architecture.svg)

explore_lite subscribes to nav_msgs/OccupancyGrid and map_msgs/OccupancyGridUpdate messages to build a map for finding frontiers. **You can use the costmap published by move_base (e.g., `<move_base>/global_costmap/costmap`), or you can use the `map` built by a mapping algorithm (SLAM).**

Depending on your environment, you might get better results using either the SLAM map or the costmap published by move_base. The advantage of the move_base costmap is the inflation effect, which helps handle very small unexplorable frontiers. When using the raw map produced by SLAM, you should set the min_frontier_size parameter to a reasonable value to handle small frontiers. For detailed information on both setups, check the explore.launch and explore_costmap.launch files.

## Setup
Before trying explore_lite, you need to have a working move_base for navigation. You should be able to navigate manually using move_base through rviz. Refer to navigation#Tutorials to set up move_base and the rest of the navigation stack for your robot.

You should also be able to navigate through unknown spaces in your map using move_base. If you set a goal in an unknown area of the map, planning and navigation should work. This should work by default for most planners, but if you need to set this feature for the navfn planner, see navfn#Parameters (though it should be enabled by default). explore_lite needs to be able to navigate through unknown space.

If you want to use the costmap provided by move_base, you need to enable unknown space tracking by setting track_unknown_space: true.

If you have properly configured move_base, you can start trying explore_lite. The provided explore.launch should work out of the box in most cases, but as usual, you might need to adjust topic names and coordinate frame names according to your setup.

## Called Actions

**move_base** (move_base_msgs/MoveBaseAction)  
The move_base actionlib API is used to publish navigation goals. See move_base#Action API for details. This requires the move_base node to be in the same namespace as explore_lite, or you may need to remap the node if this is not the case.

## Subscribed Topics

**costmap** (nav_msgs/OccupancyGrid)  
Map used for exploration planning. Can be either a costmap from move_base or a map created by SLAM (see above). The occupancy grid must correctly mark unknown space, which map-building algorithms typically track by default. If you want to use the costmap provided by move_base, you need to enable unknown space tracking by setting track_unknown_space: true.

**costmap_updates** (map_msgs/OccupancyGridUpdate)  
Incremental updates to the costmap. Not necessary to subscribe if the map source always publishes full updates (i.e., does not provide this topic).

## Published Topics

**~frontiers** (visualization_msgs/MarkerArray)  
Visualization of frontiers considered by the exploration algorithm. Each frontier is visualized by blue frontier points and a small sphere representing the cost of the frontier (higher cost frontiers have smaller spheres).

## Parameters

**~robot_base_frame** (string, default: base_link)  
The name of the robot's base frame. Used to determine the robot's position on the map. Required parameter.

**~costmap_topic** (string, default: costmap)  
Specifies the topic for the source nav_msgs/OccupancyGrid. Required parameter.

**~costmap_updates_topic** (string, default: costmap_updates)  
Specifies the topic for the source map_msgs/OccupancyGridUpdate. Not necessary to set if the map source always publishes full updates (i.e., does not provide this topic).

**~visualize** (bool, default: false)  
Specifies whether to publish frontier visualization information.

**~planner_frequency** (double, default: 1.0)  
The frequency at which to compute new frontiers and reconsider goals, in hertz.

**~progress_timeout** (double, default: 30.0)  
Time in seconds. The current goal will be abandoned when the robot has not made any progress for progress_timeout seconds.

**~potential_scale** (double, default: 1e-3)  
Used for frontier weight calculation. This multiplication parameter affects the potential component (distance to frontier) of the frontier weight.

**~orientation_scale** (double, default: 0)  
Used for frontier weight calculation. This multiplication parameter affects the orientation component of the frontier weight. This parameter currently has no effect and is provided only for backward compatibility.

**~gain_scale** (double, default: 1.0)  
Used for frontier weight calculation. This multiplication parameter affects the gain component (frontier size) of the frontier weight.

**~transform_tolerance** (double, default: 0.3)  
The tolerance used when transforming the robot's pose.

**~min_frontier_size** (double, default: 0.5)  
Minimum frontier size to consider frontiers as exploration goals, in meters.

## Required tf Transforms

**global_frame → robot_base_frame**  
This transformation is typically provided by the map-building algorithm. These coordinate frames are commonly called map and base_link. See the corresponding parameter for adjusting the robot_base_frame name. You don't need to set global_frame. The name of the global_frame will be automatically obtained from the costmap_topic.