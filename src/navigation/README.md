# 导航包 (Navigation Package)
## 简介
此导航包为ME5413自主移动机器人课程的期末项目提供了导航配置。该包基于ROS Navigation Stack，配置为使用TEB局部规划器(Timed Elastic Band)，针对Jackal差速机器人进行了优化。

## 功能特点
- 使用TEB局部规划器，支持高效路径规划和障碍物避让
- 已完成参数调优，适用于项目环境中的Jackal机器人
- 包含全局和局部代价地图配置
- 提供了基于里程计和地图两种导航模式

## 依赖
本包依赖以下ROS包：
- `roscpp` 和 `rospy`
- `std_msgs` 
- `nav_msgs` 和 `geometry_msgs`
- `move_base` 和 `tf2` 相关包
- `jackal_navigation`
- `teb_local_planner`（需单独安装）

## 安装 teb_local_planner（如未安装）
```bash
sudo apt-get install ros-noetic-teb-local-planner
```

## 文件结构
```
navigation/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包依赖定义
├── README.md               # 本文档
├── launch/
│   └── move_base.launch    # 启动导航堆栈
├── params/
│   ├── base_local_planner_params.yaml    # 基础路径规划器参数
│   ├── costmap_common_params.yaml        # 通用代价地图参数
│   ├── move_base_params.yaml             # move_base配置
│   ├── teb_local_planner_params.yaml     # TEB局部规划器参数
│   ├── map_nav_params/                   # 基于地图的导航参数
│   │   ├── global_costmap_params.yaml
│   │   └── local_costmap_params.yaml
│   └── odom_nav_params/                  # 基于里程计的导航参数
│       ├── global_costmap_params.yaml
│       └── local_costmap_params.yaml
└── scripts/
    └── send_goal.py                      # 发送导航目标的脚本
```

## 使用方法

### 启动导航
此包通常通过项目中的 me5413_world/navigation.launch 启动：

```bash
roslaunch me5413_world navigation.launch
```

或者直接启动move_base节点：

```bash
roslaunch navigation move_base.launch
```

### 发送导航目标

1. 使用RVIZ的2D Nav Goal工具
2. 通过脚本发送：
```bash
rosrun navigation send_goal.py
```
3. 发布到topic:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

## 参数调整

### TEB局部规划器关键参数
在 teb_local_planner_params.yaml 中：
- `max_vel_x`: 最大线速度，当前为5.0 m/s
- `max_vel_theta`: 最大角速度，当前为2.0 rad/s
- `min_obstacle_dist`: 避障最小距离，当前为0.25m
- `weight_obstacle`: 障碍物权重，当前为50

### 代价地图参数
调整 costmap_common_params.yaml 中的：
- `obstacle_range`: 障碍物检测范围，当前为2.5m
- `inflation_radius`: 障碍物膨胀半径，当前为0.30m

## 常见问题与解决方案

### 路径规划问题
如果机器人无法规划路径或绕过障碍物，请尝试：
- 增加 `inflation_radius` 值
- 减小 `weight_obstacle` 值使规划器更激进
- 确保`footprint`参数设置准确

### TF变换问题
如出现TF相关错误：
```
[ WARN] [/move_base]: Transform from map to base_link was unavailable
```
请检查`tf_static`发布状态和机器人坐标系设置。

---

有问题或需改进建议请联系维护者：czhihan@u.nus.edu