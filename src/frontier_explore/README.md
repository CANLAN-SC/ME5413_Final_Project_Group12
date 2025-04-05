# frontier_explore
## 概述
该软件包提供贪婪的基于前沿的探索功能。当节点运行时，机器人将贪婪地探索其环境，直到找不到更多前沿为止。移动命令将被发送到move_base。

与类似的软件包不同，explore_lite不创建自己的代价地图，这使其更易于配置且更高效（资源占用更少）。节点只需订阅nav_msgs/OccupancyGrid消息。机器人移动命令会被发送到move_base节点。

节点可以进行前沿过滤，并且可以在非膨胀地图上运行。目标黑名单功能允许处理机器人无法到达的地方。

## 架构
explore_lite使用move_base进行导航。您需要运行一个正确配置的move_base节点。

![explore_lite架构图](http://wiki.ros.org/explore_lite?action=AttachFile&do=get&target=architecture.svg)

explore_lite订阅nav_msgs/OccupancyGrid和map_msgs/OccupancyGridUpdate消息来构建一个用于寻找前沿的地图。**您可以使用move_base发布的代价地图（例如`<move_base>/global_costmap/costmap`），也可以使用由测绘算法（SLAM）构建的`map`。**

根据您的环境，您可能使用SLAM地图或move_base发布的代价地图会获得更好的结果。move_base代价地图的优势在于膨胀效果，这有助于处理一些非常小的无法探索的前沿。当您使用SLAM产生的原始地图时，应该将min_frontier_size参数设置为一个合理的数值，以处理小型前沿。有关两种设置的详细信息，请查看explore.launch和explore_costmap.launch启动文件。

## 设置
在开始尝试explore_lite之前，您需要有一个可用于导航的move_base。您应该能够通过rviz手动使用move_base进行导航。请参考navigation#Tutorials来为您的机器人设置move_base和导航栈的其余部分。

您还应该能够使用move_base导航穿过地图中的未知空间。如果您将目标设置在地图的未知区域，规划和导航应该能够正常工作。对于大多数规划器，这应该默认就能工作，如果需要为navfn规划器设置此功能，请参考navfn#Parameters（但应该默认已启用）。explore_lite需要能够通过未知空间进行导航。

如果您想使用move_base提供的代价地图，您需要通过设置track_unknown_space: true来启用未知空间跟踪。

如果您已正确配置move_base，就可以开始尝试explore_lite。提供的explore.launch在大多数情况下应该可以直接使用，但如往常一样，您可能需要根据您的设置调整话题名称和坐标系名称。

## 调用的动作

**move_base** (move_base_msgs/MoveBaseAction)  
move_base actionlib API 用于发布导航目标。详情请参见 move_base#Action API。这要求 move_base 节点与 explore_lite 在同一命名空间中，如果不是这样，您可能需要重新映射该节点。

## 订阅的话题

**costmap** (nav_msgs/OccupancyGrid)  
用于探索规划的地图。可以是来自 move_base 的代价地图或由 SLAM 创建的地图（见上文）。占用栅格必须正确标记未知空间，地图构建算法通常默认会跟踪未知空间。如果您想使用 move_base 提供的代价地图，需要通过设置 track_unknown_space: true 来启用未知空间跟踪。

**costmap_updates** (map_msgs/OccupancyGridUpdate)  
代价地图的增量更新。如果地图源始终发布完整更新（即不提供此话题），则不必订阅。

## 发布的话题

**~frontiers** (visualization_msgs/MarkerArray)  
探索算法考虑的前沿可视化。每个前沿都通过蓝色的前沿点和一个小球体来可视化，小球体表示前沿的成本（成本更高的前沿会有更小的球体）。

## 参数

**~robot_base_frame** (string, 默认值: base_link)  
机器人基座坐标系的名称。用于确定机器人在地图上的位置。必需参数。

**~costmap_topic** (string, 默认值: costmap)  
指定源 nav_msgs/OccupancyGrid 的话题。必需参数。

**~costmap_updates_topic** (string, 默认值: costmap_updates)  
指定源 map_msgs/OccupancyGridUpdate 的话题。如果地图源始终发布完整更新（即不提供此话题），则不必设置。

**~visualize** (bool, 默认值: false)  
指定是否发布前沿可视化信息。

**~planner_frequency** (double, 默认值: 1.0)  
计算新前沿和重新考虑目标的频率，单位为赫兹。

**~progress_timeout** (double, 默认值: 30.0)  
时间，单位为秒。当机器人在 progress_timeout 时间内没有取得任何进展时，当前目标将被放弃。

**~potential_scale** (double, 默认值: 1e-3)  
用于前沿权重计算。这个乘法参数影响前沿权重的潜力组件（到前沿的距离）。

**~orientation_scale** (double, 默认值: 0)  
用于前沿权重计算。这个乘法参数影响前沿权重的方向组件。该参数目前不起作用，仅为向前兼容性而提供。

**~gain_scale** (double, 默认值: 1.0)  
用于前沿权重计算。这个乘法参数影响前沿权重的增益组件（前沿大小）。

**~transform_tolerance** (double, 默认值: 0.3)  
变换机器人姿态时使用的容差。

**~min_frontier_size** (double, 默认值: 0.5)  
考虑前沿作为探索目标的最小前沿尺寸，单位为米。

## 所需的 tf 变换

**global_frame → robot_base_frame**  
此变换通常由地图构建算法提供。这些坐标系通常称为 map 和 base_link。有关调整 robot_base_frame 名称，请参见相应参数。您不需要设置 global_frame。global_frame 的名称将自动从 costmap_topic 中获取。