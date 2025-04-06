# Cartographer 2D Mapping Steps

1. Build the Map
```
roslaunch me5413_world world.launch
roslaunch me5413_world carto_2d.launch
```
--- 

2. Save the Map
2.1 finish the trajectory
```
rosservice call /finish_trajectory 0
```

2.2 create a map.pbstream file
```
rosservice call /write_state "{filename: '${HOME}/ME5413_Final_Project/src/me5413_world/maps/map_2d.pbstream'}"
```

2.3 Convert map.pbstream files to map.yaml & map.pgm files
```
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -map_filestem=${HOME}/ME5413_Final_Project/src/me5413_world/maps/map_2d \
    -pbstream_filename=${HOME}/ME5413_Final_Project/src/me5413_world/maps/map_2d.pbstream \
    -resolution=0.05
```


---



# Cartographer 3D Mapping Steps
1. 
roslaunch me5413_world world.launch
2. 
roslaunch me5413_world carto_3d.launch
3. Record data, generate bags (to record trajectories and generate ply files)
```
cd src/me5413_world/bagfiles/
rosbag record -O carto_3d.bag /mid/points /odometry/filtered /imu/data /navsat/fix /tf /tf_static /front/scan /gazebo/ground_truth/state /tracked_pose 
```
4. Generate map file
```
rosservice call /finish_trajectory 0
```
Generate .pbstream file
```
rosservice call /write_state "{filename: '${HOME}/ME5413_Final_Project/src/me5413_world/maps/map_3d.pbstream'}"
```
5. Generate .ply file
```
roslaunch cartographer_ros assets_writer_backpack_3d.launch bag_filenames:='/home/wu/ME5413_Final_Project/src/me5413_world/bagfiles/carto_3d.bag' pose_graph_filename:='/home/wu/ME5413_Final_Project/src/me5413_world/maps/map_3d.pbstream'
```
6. View .ply file
```
cd ~/ME5413_Final_Project/src/me5413_world/scripts
```
copy with carto_3d.bag_points.ply
```
python3 ply_view.py carto_3d.bag_points.ply
```
7. Generate .pcd file
```
pcl_ply2pcd carto_3d.bag_points.ply carto3d.pcd
```
8. Generate pgm&yaml file生成pgm和yaml文件
```
roslaunch pcd_to_map pcd_to_map.launch
rosrun map_server map_saver -f carto_3d
```
9. Evaluate trajectory using Evo
```
evo_ape bag carto_3d.bag /gazebo/ground_truth/state /tf:map.base_link -as --plot

```


---

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

---

# Fast-livo


1. Mapping


# Launch world
in the first terminal:
```bash
#roslaunch me5413_world world.launch
```

# Launch fastlio, This will publish the 3D point cloud information topic:`/cloud_registed `
in the second terminal:
```bash
#roslaunch me5413_world fast_lio.launch
```

# Launch pcd_to_map. This will publish the 2D map topic:`/map` 
in the third terminal:
```bash
# roslaunch pcd_to_map pcd_to_map.launch
```

# Save the map as `my_map` .You can see the .png in the `\ME5413_Final_Project_Group12\src\pcd_to_map\map"` folder
```bash
# rosrun map_server map_saver -f my_map
```

# Use evo to evaluate the results
in the forth terminal:
```bash
# rosbag record -O output.bag /gazebo/ground_truth/state /Odometry
```
after record, you can see the visualize the results by using :
```bash
# evo_ape bag output.bag /gazebo/ground_truth/state /Odometry -a --plot
```

