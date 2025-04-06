# carto_2d_evo
1. Record relevant topics from ROS to get the trajectory 
```
cd src/me5413_world/bagfiles/ 
rosbag record -O trajectories.bag /gazebo/ground_truth/state /tf /tracked_pose 
```

2. Evaluate the trajectory using evo 
```
evo_ape bag trajectories.bag /gazebo/ground_truth/state /tf:map.base_link -as --plot 
```



# carto_3d_evo
1. Record relevant topics from ROS to get the trajectory 
```
cd src/me5413_world/bagfiles/ 
rosbag record -O trajectories_3d.bag /gazebo/ground_truth/state /tf /tracked_pose
```

2. Evaluate the trajectory using evo 
```
evo_ape bag trajectories_3d.bag /gazebo/ground_truth/state /tf:map.base_link -a --plot 
```
or 
```
evo_ape bag trajectories_3d.bag /gazebo/ground_truth/state /tracked_pose -a --plot 
```



# Message types supported by evo include:

geometry_msgs/PoseStamped 
nav_msgs/Odometry 
geometry_msgs/PoseWithCovarianceStamped 
geometry_msgs/TransformStamped (from TF transforms)

