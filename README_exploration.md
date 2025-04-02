## Exploartion
explore_lite第三方库安装：
官方网站：https://github.com/hrnr/m-explore， https://wiki.ros.org/explore_lite
我安装在了'third_party/m-explore'文件夹下，其中‘/explore’文件夹是'explore_lite'核心文件，'map_merge'适用于多智能体探索情况下的地图融合，本次任务用不到。\
安装TEB和global_planner 包（内置 A* 支持）
```bash
sudo apt install ros-noetic-teb-local-planner
sudo apt install ros-noetic-navigation
```

### 对navigation新增内容
对 'me5413_world/params/map_nav_params/global_costmap_params.yaml' 文件稍作添加：
```
track_unknown_space: true
```

### 新增内容：
添加了 'me5413_world/launch/explore.launch' 文件；
添加了 'third_party/m_explore' 第三方库；

### 运行：
```
roslaunch me5413_world navigation.launch
```
```
roslaunch me5413_world explore.launch
```
运行'explore.launch'之后，耐心等个几秒钟，小车就会自动探索（加载过程需要一点时间，不能launch之后小车马上寻路）。

### 后续问题（按重要程度）
1.小车在探索过程中会掉河里；
2.定位效果一般；