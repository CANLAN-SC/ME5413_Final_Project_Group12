## 简介

这是一个使用 SMACH 状态机框架实现的 ROS 节点，用于协调机器人完成一系列任务的执行顺序。本状态机负责管理多个任务状态的转换，并提供可视化界面以便于调试和监控。

## 主要功能

- 使用状态机框架管理复杂任务流程
- 实现机器人自主导航到指定位置
- 触发 OCR 识别系统进行处理
- 检测环境中的盒子并处理相关信息
- 管理桥梁检测等特定任务
- 提供任务执行状态可视化

## 状态机架构

状态机包含以下几个主要状态：

1. **Initialize**：系统初始化状态
   - 负责系统的初始设置
   - 成功时转到 NavigateToGoal 状态

2. **NavigateToGoal**：导航到目标点
   - 使用 move_base action 接口导航到指定坐标
   - 监控导航结果并相应地转换状态

3. **ExploreFrontier (TASK_ONE)**：执行任务一
   - 发布触发消息到 `/ocr_trigger` 话题
   - 订阅 `/detected_boxes` 话题接收盒子位置信息
   - 处理并记录检测到的盒子

4. **DetectBridge (TASK_TWO)**：执行任务二
   - 处理与桥梁检测相关的任务

5. **NavigateToGoal (TASK_THREE)**：执行任务三
   - 执行任务完成后的导航任务
   - 最终完成整个任务流程

## 使用方法

1. 确保已安装所需依赖：
   ```bash
   sudo apt-get install ros-$ROS_DISTRO-smach ros-$ROS_DISTRO-smach-ros python-smach
   ```

2. 在终端中启动节点：
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   python src/fsm/scripts/fsm.py
   ```

3. 查看状态机执行情况：
   ```bash
   rosrun smach_viewer smach_viewer.py
   ```

## 与其他节点的交互

- 与 `move_base` 节点通信以执行导航任务
- 发布消息到 `/ocr_trigger` 话题触发OCR识别
- 接收 `/detected_boxes` 话题获取检测到的盒子信息

## 状态转换逻辑

```
Initialize ──成功──→ NavigateToGoal ──成功──→ TASK_ONE ──成功──→ TASK_TWO ──成功──→ TASK_THREE ──成功──→ mission_completed
     │                     │                  │                 │                 │
     └───失败───┐   ┌───失败───┐      ┌───失败───┐     ┌───失败───┐     ┌───失败───┐
                ▼   ▼           ▼      ▼           ▼     ▼           ▼     ▼
              mission_failed
```

## 数据流
### box_position 在任务状态机中的传递流程分析

`box_position` 在您的状态机中的传递是通过 SMACH 状态机框架的 `userdata` 机制实现的。让我梳理一下它的完整流程：

#### 1. 数据的产生 - DetectBoxPose 状态

在 `DetectBoxPose` 状态中，通过以下步骤生成 `box_positions`：

1. 从代价地图提取障碍物信息
2. 使用 DBSCAN 聚类算法对障碍物点进行聚类
3. 计算每个聚类的中心点，将其转换为地图坐标系下的位置
4. 根据配置的探索区域边界，过滤出位于探索区域内的盒子
5. 将检测到的盒子位置包装成 `PoseArray` 消息

关键代码：
```python
# 将盒子姿态数组创建为PoseArray，便于传递
pose_array = PoseArray()
pose_array.header.frame_id = "map"
pose_array.header.stamp = rospy.Time.now()
pose_array.poses = [box.pose for box in costmap_boxes]
userdata.box_positions_out = pose_array
```

#### 2. 数据的输出定义

在 `DetectBoxPose` 状态的初始化中声明了输出键：
```python
smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                     output_keys=['box_positions_out'])
```

#### 3. 状态机中的数据映射

在顶层状态机中，通过 `remapping` 参数将状态之间的数据进行映射：
```python
smach.StateMachine.add('DETECT_BOX_POSE', DetectBoxPose(), 
                       transitions={'succeeded':'NAVIGATE_TO_BOX_AND_OCR', 
                                   'failed':'mission_failed'},
                       remapping={'box_positions_out':'box_positions'})
```

这里将 `DetectBoxPose` 状态的 `box_positions_out` 输出键映射到状态机的 `box_positions` 变量。

#### 4. 数据的接收 - NavigateToBoxAndOCR 状态

在 `NavigateToBoxAndOCR` 状态中，通过以下方式接收盒子位置数据：

```python
smach.State.__init__(self, 
                   outcomes=['succeeded', 'failed'], 
                   input_keys=['box_positions_in'])
```

并在状态机中进行映射：
```python
smach.StateMachine.add('NAVIGATE_TO_BOX_AND_OCR', NavigateToBoxAndOCR(),
                       transitions={'succeeded':'DETECT_BRIDGE', 
                                   'failed':'mission_failed'},
                       remapping={'box_positions_in':'box_positions'})
```

#### 5. 数据的使用 - NavigateToBoxAndOCR 状态中

在 `NavigateToBoxAndOCR::execute` 方法中，从 `userdata` 获取盒子位置：
```python
# 从userdata中获取盒子位置
self.box_positions = userdata.box_positions_in
```

然后使用这些位置数据进行导航和 OCR 处理：
```python
# 对每个盒子进行导航和OCR
for i, pose in enumerate(self.box_positions.poses):
    try:
        self.navigate_to_best_viewing_positions(pose)
        
    except Exception as e:
        rospy.logerr('处理盒子时发生错误: %s', str(e))
```

#### 6. 数据类型和格式

- **数据类型**: `PoseArray` 消息类型，包含多个 `Pose` 对象
- **结构**: 
  - `header`: 标准消息头，包含时间戳和坐标系信息
  - `poses[]`: 包含多个 `Pose` 对象的数组，每个对象包含位置(x,y,z)和方向(四元数)

#### 7. 数据在ROS系统中的流动

除了在状态机内部传递外，`box_positions` 还通过 ROS 话题发布：

```python
self.box_publisher.publish(pose_array)
```

这样可以使其他节点也能订阅和使用盒子位置信息，比如可视化工具或外部监控节点。

## 总结

数据流程如下：
1. `DetectBoxPose` 从代价地图检测盒子位置
2. 将位置包装为 `PoseArray` 并存储到 `userdata.box_positions_out`
3. 通过状态机的 `remapping` 机制，数据被映射到状态机共享变量 `box_positions`
4. `NavigateToBoxAndOCR` 从 `userdata.box_positions_in` 获取数据
5. 使用这些位置导航到每个盒子并执行 OCR 处理
6. 同时，数据通过 ROS 话题发布，方便系统中其他组件使用

这种设计使各个状态能够共享数据，同时保持了状态之间的松耦合性。

## 注意事项

- 代码中有注释掉的启动 launch 文件的功能，如需启用请取消相关注释
- 导航目标点坐标在 `NavigateToExplorationArea` 类中硬编码设置，可根据实际需求修改
- 状态机的可视化服务运行在 `/TASK_COORDINATOR` 命名空间下

## 扩展与修改

如需添加新的任务状态，请按以下步骤进行：
1. 创建新的状态类，继承 `smach.State`
2. 实现 `execute()` 方法处理任务逻辑
3. 在 `main()` 函数中使用 `smach.StateMachine.add()` 添加到状态机
4. 更新相关状态的转换关系

要了解更多关于SMACH状态机的信息，请访问 [ROS SMACH 教程](http://wiki.ros.org/smach/Tutorials)

## 疑难杂症
### `/move_base/global_costmap/costmap` 和 `/move_base/global_costmap/costmap_updates` 的来源是什么

这两个话题都是ROS导航栈(navigation stack)中的核心组件，特别是`move_base`节点中的`global_costmap`功能生成的：

#### `/move_base/global_costmap/costmap`

**来源**：这个话题由`move_base`节点内部的`global_costmap`对象创建和发布。

**内容**：完整的全局代价地图，是SLAM系统提供的地图与各种代价层(cost layers)叠加后的结果。

**数据流**：
1. SLAM系统(如gmapping、hector_slam、cartographer或您使用的FAST-LIO)生成基础地图，通常发布在`/map`话题上
2. `move_base`订阅这个地图，将其作为`global_costmap`的静态层(static layer)
3. `move_base`加入其他层(如障碍层、膨胀层等)，得到最终的全局代价地图
4. 这个完整的地图被发布到`/move_base/global_costmap/costmap`

#### `/move_base/global_costmap/costmap_updates`

**来源**：同样由`move_base`节点的`global_costmap`对象创建和发布。

**内容**：只包含了最近发生变化的代价地图部分，是一种增量更新机制。

**数据流**：
1. 当传感器数据导致代价地图变化时，`move_base`计算出变化的部分
2. 这些变化被打包成一个小型OccupancyGrid消息，只包含修改过的区域
3. 这个增量更新被发布到`/move_base/global_costmap/costmap_updates`

### 触发器有哪些

#### 发布的触发消息

1. **`/box_detection_trigger`**
   - 类型：`std_msgs/Bool`
   - 发布者：`DetectBoxPose`状态
   - 作用：触发外部盒子检测节点开始检测场景中的盒子
   - 数据：`True`表示开始检测

2. **`/ocr_trigger`**
   - 类型：`std_msgs/Bool`
   - 发布者：`NavigateToBoxAndOCR`和`NavigateToGoalAndOCR`状态
   - 作用：触发OCR节点开始识别图像中的文字
   - 数据：`True`表示开始OCR识别
   - 命令行测试：`rostopic pub /ocr_trigger std_msgs/Bool "data: true" -1`

3. **`/bridge_detection_trigger`**
   - 类型：`std_msgs/Bool`
   - 发布者：`DetectBridge`状态
   - 作用：触发桥梁检测节点开始检测场景中的桥梁
   - 数据：`True`表示开始桥梁检测

4. **`/open_bridge`**
   - 类型：`std_msgs/Bool`
   - 发布者：`OpenBridgeAndNavigate`状态
   - 作用：触发桥梁控制节点打开桥梁
   - 数据：`True`表示打开桥梁

5. **`/move_base/global_costmap/costmap`**和**`/move_base/global_costmap/costmap_updates`**
   - 类型：`nav_msgs/OccupancyGrid`
   - 发布者：`ExploreFrontier`状态
   - 作用：发布探索区域的代价地图数据，用于导航规划

#### 订阅的消息

1. **`/detected_boxes`**
   - 类型：`geometry_msgs/PoseArray`
   - 订阅者：`DetectBoxPose`状态
   - 作用：接收盒子检测节点发现的盒子位置信息
   - 回调：`box_callback`，将检测到的盒子位置存储在列表中

2. **`/detected_bridges`**
   - 类型：`geometry_msgs/PoseStamped`
   - 订阅者：`DetectBridge`状态
   - 作用：接收桥梁检测节点发现的桥梁入口位置信息
   - 回调：`bridge_callback`，将检测到的桥梁位置记录下来

3. **`/cmd_stop`**
   - 类型：`std_msgs/Bool`
   - 订阅者：`NavigateToGoalAndOCR`状态
   - 作用：接收OCR节点发送的停止信号，表示已找到目标盒子
   - 回调：`cmd_stop_callback`，如果收到`True`，则标记任务完成并停止导航

#### 触发流程

1. 初始化 → 导航到探索区域
2. 执行前沿探索，发布地图数据
3. 触发盒子检测，接收盒子位置
4. 导航到每个盒子并触发OCR识别
5. 触发桥梁检测，接收桥梁位置
6. 导航到桥梁入口
7. 发送开桥指令并导航过桥
8. 导航到目标区域并触发OCR，直到收到停止信号

### 最新进展
已知盒子位置+导航
```bash
[INFO] [1743674765.743773, 512.718000]: 等待接收盒子位置数据...
[INFO] [1743674767.528165, 513.218000]: 导航到9个盒子并启动OCR...
[INFO] [1743674767.529933, 513.220000]: [1/9] 导航到盒子位置: x=15.37, y=-16.05
```
