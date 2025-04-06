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
