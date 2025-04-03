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
rosservice call /write_state "{filename: '${HOME}/map.pbstream'}"
```

2.3 Convert map.pbstream files to map.yaml & map.pgm files
```
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -map_filestem=${HOME}/Downloads/map \
    -pbstream_filename=${HOME}/Downloads/map.pbstream \
    -resolution=0.05
```
