# Cartographer 3D Mapping Steps
summary: you can get the ply file at the following path:
```
ME5413_Final_Project_Group12/src/me5413_world/scripts/carto_points.ply
```
## 1. Record the Data
To start, record the data package for your robot using the following commands:

```bash
roslaunch me5413_world world.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosbag record -O carto /mid/points /odometry/filtered /imu/data /navsat/fix /tf /tf_static /front/scan
```
The data package have been saved at the following path:
```
~/ME5413_Final_Project/src/me5413_world/bagfiles/carto.bag
```
---
## 2.Replay the Data and Build the Map
Next, launch Cartographer to construct the 3D map from the recorded data:
```
roslaunch me5413_world carto_3d.launch
```
Note: Make sure to modify the path in the launch file as needed.
---
## 3.Save the Map

Once the map is built, stop the map building process and save the map:
```
rosservice call /finish_trajectory 0
```
This will create a carto.pbstream file. You can then convert the state to a .pbstream file by running:
```
rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"
```
To generate a visualized point cloud file, run the following:
```
roslaunch cartographer_ros assets_writer_backpack_3d.launch bag_filenames:='/home/wu/ME5413_Final_Project/src/me5413_world/bagfiles/carto.bag' pose_graph_filename:='/home/wu/ME5413_Final_Project/src/me5413_world/maps/carto.pbstream'

```
---
## 4. View the Point Cloud (PLY) File

To view the generated PLY file, install Open3D and Numpy:
```
pip install open3d numpy
```
Then, navigate to the scripts folder and run the following Python script to view the PLY file:
```
cd ~/ME5413_Final_Project/src/me5413_world/scripts
python3 view_ply.py carto_points.ply
```


