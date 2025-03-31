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

# Save the map as `my_map` .
```bash
# rosrun map_server map_saver -f my_map
```