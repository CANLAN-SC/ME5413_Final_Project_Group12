#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist, Point
import math
from std_msgs.msg import Int32, ColorRGBA

# 可视化
import matplotlib.pyplot as plt
import numpy as np

# ====================== 全局配置参数 ======================
class Config:
    # 话题名称配置
    TOPICS = {
        # 代价地图相关话题
        'GLOBAL_COSTMAP': '/move_base/global_costmap/costmap',
        'GLOBAL_COSTMAP_UPDATES': '/move_base/global_costmap/costmap_updates',
        'EXPLORE_COSTMAP': '/frontier_explore/costmap',
        'EXPLORE_COSTMAP_UPDATES': '/frontier_explore/costmap_updates',
        'FRONTIERS': '/explore/frontiers',
        
        # 检测触发和结果话题
        'BOX_DETECTION_TRIGGER': '/box_detection_trigger',
        'DETECTED_BOXES': '/detected_boxes',
        'BRIDGE_DETECTION_TRIGGER': '/bridge_detection_trigger',
        'DETECTED_BRIDGES': '/detected_bridges',
        'OCR_TRIGGER': '/ocr_trigger',
        'CMD_STOP': '/cmd_stop',
        'OPEN_BRIDGE': '/open_bridge',
        
        # 导航相关话题
        'MOVE_BASE': 'move_base',

        # 盒子提取相关话题
        # 'BOX_EXTRACTION': '/move_base/global_costmap/costmap/obstacles_layer',
        'BOX_EXTRACTION': '/move_base/global_costmap/costmap',
        # OCR相关话题
        'RECOGNIZED_DIGIT': '/recognized_digit',
    }
    
    # 超时设置(秒)
    TIMEOUTS = {
        'INIT': 60.0,                # 初始化超时
        'EXPLORE': 300.0,            # 探索任务超时
        'BOX_DETECTION': 3.0,       # 盒子检测超时
        'BRIDGE_DETECTION': 15.0,    # 桥梁检测超时
        'NAVIGATION': 60.0,          # 导航超时
        'OCR_PROCESSING': 5.0,       # OCR处理超时
        'BRIDGE_OPEN': 3.0,          # 桥梁打开等待时间
    }

    # 障碍物阈值
    OBSTACLE_THRESHOLD = 99  # 动态调整的初始阈值
    
    # 禁入区域设置
    RESTRICTED_MAP_BOUNDS = {
        'X_MIN': 0.0,    # 地图X坐标最小值
        'X_MAX': 11.0,   # 地图X坐标最大值
        'Y_MIN': 0.0,    # 地图Y坐标最小值
        'Y_MAX': 22.0,   # 地图Y坐标最大值
    }

    # 探索区域设置
    EXPLORE_MAP_BOUNDS = {
        'X_MIN': 9.0,    # 地图X坐标最小值
        'X_MAX': 19.0,   # 地图X坐标最大值
        'Y_MIN': -22.0,    # 地图Y坐标最小值
        'Y_MAX': -2.0,   # 地图Y坐标最大值
    }
    
    # 探索区域坐标表示
    # x,y,w,z
    EXPLORE_POINTS = [
        (19.0, -22.0, 1.0, 0.0),
        (9.0, -22.0, 0.7071, 0.7071),
        (9.0, -2.0, 1.0, 0.0),
        (19.0, -2.0, 0.7071, -0.7071)
    ]

    # 掩码操作设置, 如果使用的是my_map_exploration.map，则需要设置USE_EXPLORE_MASK为False
    # 因为my_map_explore地图自带探索区域
    # 如果想直接使用原图，可以一键设置USE_MASKED为True
    MASKED_CONFIG = {
        'USE_MASKED': False,  # 是否使用掩码
        'USE_RESOLUTION': True,  # 是否使用分辨率
        'USE_RESTRICTED_MASK': True,  # 是否使用禁入区域
        'USE_EXPLORE_MASK': False,  # 是否使用探索区域
    }

    # 探索相关设置
    EXPLORE = {
        'FRONTIER_THRESHOLD': 0,     # 前沿点数量阈值，低于此值认为探索完成
        'CHECK_RATE': 0.5,           # 检查频率(Hz)
    }
    
    # 桥梁相关设置
    BRIDGE = {
        'EXIT_Y_OFFSET': -3.0,       # 桥出口相对于入口的Y轴偏移
    }
    
    # 目标位置坐标列表 (x, y, orientation.w)
    GOALS = [
        (6.0, 0.0, 1.0),
        (10.0, 0.0, 1.0),
        (14.0, 0.0, 1.0),
        (18.0, 0.0, 1.0)
    ]

    # 最佳观察距离
    VIEWING_DISTANCE = 1.0

    # 观察角度
    VIEWING_ANGLES = [0, math.pi/2, math.pi, 3*math.pi/2]  # 0°, 90°, 180°, 270°

    # 盒子边长
    BOX_SIZE = 0.5  # 米

    # 聚类参数
    CLUSTERING = {
        'width': 30,
        'height': 30,
        'MIN_POINTS': 20,  # 最小点数
        'MAX_POINTS': 200,  # 最大点数
    }

# 定义状态：初始化
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.timeout = Config.TIMEOUTS['INIT']

    def execute(self, userdata):
        # try:
        #     # 使用直接速度控制快速通过第一路段
        #     if hasattr(Config, 'DIRECT_CONTROL') and Config.DIRECT_CONTROL.get('ENABLED', False):
        #         if self.fast_navigate_first_segment():
        #             rospy.loginfo('通过直接控制快速到达探索区域')
        #             return 'initialized'
        #         else:
        #             rospy.logwarn('直接控制导航失败，使用move_base作为备选方案')
            
        #     # 如果直接控制被禁用或失败，使用move_base导航
        #     # move_base依次前往探索区域的四个点
        #     for i, point in enumerate(Config.EXPLORE_POINTS):
        #         x, y, w, z = point
        #         rospy.loginfo('[%d/%d] 初始化导航到探索区域点: x=%.2f, y=%.2f', 
        #                     i+1, len(Config.EXPLORE_POINTS), x, y)
                
        #         # 创建目标位置
        #         goal = MoveBaseGoal()
        #         goal.target_pose.header.frame_id = "map"
        #         goal.target_pose.header.stamp = rospy.Time.now()
        #         goal.target_pose.pose.position.x = x
        #         goal.target_pose.pose.position.y = y
        #         goal.target_pose.pose.orientation.w = w
        #         goal.target_pose.pose.orientation.z = z
                
        #         # 发送目标位置
        #         self.move_base_client.send_goal(goal)
                
        #         # 检查导航超时
        #         if not self.move_base_client.wait_for_result(rospy.Duration(self.timeout)):
        #             rospy.logerr('导航到目标位置超时，取消目标')
        #             self.move_base_client.cancel_goal()
        #             return 'failed'
                
        #         # 检查导航结果
        #         result_state = self.move_base_client.get_state()
        #         if result_state != actionlib.GoalStatus.SUCCEEDED:
        #             rospy.logerr('导航到目标位置失败，状态码: %d', result_state)
        #             return 'failed'
                    
        #         rospy.loginfo('成功到达探索区域点 [%d/%d]', i+1, len(Config.EXPLORE_POINTS))
                
        #     rospy.loginfo('初始化完成：所有探索区域点导航成功')
        #     return 'initialized'
        # except Exception as e:
        #     rospy.logerr('初始化过程出错: %s', str(e))
        #     return 'failed'
        return 'initialized'

# 定义状态：前沿探索任务
class ExploreFrontier(smach.State):
    def __init__(self):
        import threading
        self.map_lock = threading.Lock()
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        # 初始化探索区域的地图数据
        self.costmap_msg = OccupancyGrid()
        self.costmap_updates_msg = OccupancyGridUpdate()
        
        # 全局代价地图的订阅器
        self.costmap_subscriber = rospy.Subscriber(
            Config.TOPICS['GLOBAL_COSTMAP'], 
            OccupancyGrid, 
            self.costmap_callback
        )
        self.costmap_updates_subscriber = rospy.Subscriber(
            Config.TOPICS['GLOBAL_COSTMAP_UPDATES'], 
            OccupancyGridUpdate,
            self.costmap_updates_callback
        )
        
        # 前沿探索区域的地图数据的发布器
        self.explore_costmap_publisher = rospy.Publisher(
            Config.TOPICS['EXPLORE_COSTMAP'], 
            OccupancyGrid, 
            queue_size=10
        )
        self.explore_costmap_updates_publisher = rospy.Publisher(
            Config.TOPICS['EXPLORE_COSTMAP_UPDATES'], 
            OccupancyGridUpdate,  # 使用正确的消息类型
            queue_size=10
        )        
        
        # 添加前沿点订阅器 - 监听explore_lite发布的前沿点可视化
        self.frontier_subscriber = rospy.Subscriber(
            Config.TOPICS['FRONTIERS'], 
            MarkerArray, 
            self.frontier_callback
        )
        
        # 前沿点计数和阈值设置
        self.frontier_count = 999  # 初始化为一个大数字
        self.frontier_threshold = Config.EXPLORE['FRONTIER_THRESHOLD']  
        self.max_explore_time = Config.TIMEOUTS['EXPLORE']  
        
    def execute(self, userdata):
        rospy.loginfo('开始前沿探索任务...')

        # 定期检查前沿点数量和更新掩码地图
        rate = rospy.Rate(Config.EXPLORE['CHECK_RATE'])  
        explore_start_time = rospy.Time.now()
        
        rospy.loginfo('开始监控前沿点数量，阈值为%d', self.frontier_threshold)
        
        # 等待前沿探索启动并开始发布前沿点
        rospy.loginfo('等待前沿探索启动...')
        
        # 等待地图数据就绪
        while not rospy.is_shutdown():
            with self.map_lock:
                has_map_data = (len(self.costmap_msg.data) > 0)
            
            if not has_map_data:
                rospy.loginfo('等待地图数据...')
                rate.sleep()
                continue
            
            break  # 跳出循环，开始主处理逻辑
        
        # 主处理循环
        rospy.loginfo('地图数据就绪，开始探索...')
        previous_frontier_count = self.frontier_count
        
        while (rospy.Time.now() - explore_start_time).to_sec() < self.max_explore_time:
            # 创建掩码地图，并立即释放锁
            masked_costmap = None
            masked_costmap_updates = None
            frontier_count = 0
            
            # 使用锁保护地图数据的访问，但尽量减少持有锁的时间
            with self.map_lock:
                # 首先，检查地图数据是否有效
                if len(self.costmap_msg.data) == 0:
                    rospy.loginfo('等待有效的地图数据...')
                    rate.sleep()
                    continue
                    
                # 处理地图数据
                masked_costmap = self.process_costmap(self.costmap_msg) if Config.MASKED_CONFIG['USE_MASKED'] else self.costmap_msg
                masked_costmap_updates = self.process_costmap_updates(self.costmap_updates_msg) if Config.MASKED_CONFIG['USE_MASKED'] else self.costmap_updates_msg
                frontier_count = self.frontier_count
                if previous_frontier_count != frontier_count:
                    rospy.loginfo('前沿点数量变化: %d -> %d', previous_frontier_count, frontier_count)
                    previous_frontier_count = frontier_count
            
            # 发布处理过的地图（在锁外）
            self.explore_costmap_publisher.publish(masked_costmap)
            self.explore_costmap_updates_publisher.publish(masked_costmap_updates)
                     
            # 如果前沿点数量小于阈值，认为探索完成
            if frontier_count <= self.frontier_threshold:
                # 取消订阅器
                self.costmap_subscriber.unregister()
                self.costmap_updates_subscriber.unregister()
                self.frontier_subscriber.unregister()
                # 取消发布器
                self.explore_costmap_publisher.unregister()
                self.explore_costmap_updates_publisher.unregister()
                # 关闭explore_lite节点
                rospy.loginfo('前沿点数量已低于阈值，探索任务完成')
                return 'succeeded'
            else:
                continue  # 继续循环，检查下一次

        # 停止探索
        rospy.loginfo('前沿探索超时, 停止任务')
        # 取消订阅器
        self.costmap_subscriber.unregister()
        self.costmap_updates_subscriber.unregister()
        self.frontier_subscriber.unregister()
        # 取消发布器
        self.explore_costmap_publisher.unregister()
        self.explore_costmap_updates_publisher.unregister()
        
        return 'succeeded'  # 返回成功状态

    def costmap_callback(self, msg):
        with self.map_lock:
            # 处理全局代价地图
            self.costmap_msg = msg
            self.last_complete_map = msg
    
    def costmap_updates_callback(self, msg):
        with self.map_lock:
            self.costmap_updates_msg = msg
    
    def frontier_callback(self, msg):
        with self.map_lock:
            # 计算有效的前沿点数量（通常每个前沿由多个标记点组成）
            # 只计算TYPE_SPHERE类型的标记，它代表前沿中心点
            frontier_count = 0
            for marker in msg.markers:
                if marker.type == marker.SPHERE:
                    frontier_count += 1
            
            self.frontier_count = frontier_count
        
    def process_costmap(self, costmap_msg):
        """处理代价地图，标记要探索的区域为自由空间，其他区域为障碍物"""
        import copy
        # 复制原始代价地图
        masked_costmap = copy.deepcopy(costmap_msg)
        
        # 更新时间戳和帧 ID
        masked_costmap.header.stamp = rospy.Time.now()
        if masked_costmap.header.frame_id == "":
            masked_costmap.header.frame_id = "map"
        
        # 获取地图信息
        width = masked_costmap.info.width
        height = masked_costmap.info.height
        resolution = masked_costmap.info.resolution if Config.MASKED_CONFIG['USE_RESOLUTION'] else 1.0
        origin_x = masked_costmap.info.origin.position.x
        origin_y = masked_costmap.info.origin.position.y
        
        # 计算禁入区域的边界
        restricted_area_x_min = int((Config.RESTRICTED_MAP_BOUNDS['X_MIN'] ) / resolution)
        restricted_area_x_max = int((Config.RESTRICTED_MAP_BOUNDS['X_MAX'] ) / resolution)
        restricted_area_y_min = int((Config.RESTRICTED_MAP_BOUNDS['Y_MIN'] ) / resolution)
        restricted_area_y_max = int((Config.RESTRICTED_MAP_BOUNDS['Y_MAX'] ) / resolution)
        
        # 限制索引在地图范围内
        restricted_area_x_min = max(0, min(width-1, restricted_area_x_min))
        restricted_area_x_max = max(0, min(width-1, restricted_area_x_max))
        restricted_area_y_min = max(0, min(height-1, restricted_area_y_min))
        restricted_area_y_max = max(0, min(height-1, restricted_area_y_max))

        # 计算探索区域的边界
        explore_area_x_min = int((Config.EXPLORE_MAP_BOUNDS['X_MIN'] ) / resolution)
        explore_area_x_max = int((Config.EXPLORE_MAP_BOUNDS['X_MAX'] ) / resolution)
        explore_area_y_min = int((Config.EXPLORE_MAP_BOUNDS['Y_MIN'] ) / resolution)
        explore_area_y_max = int((Config.EXPLORE_MAP_BOUNDS['Y_MAX'] ) / resolution)

        # 限制索引在地图范围内
        explore_area_x_min = max(0, min(width-1, explore_area_x_min))
        explore_area_x_max = max(0, min(width-1, explore_area_x_max))
        explore_area_y_min = max(0, min(height-1, explore_area_y_min))
        explore_area_y_max = max(0, min(height-1, explore_area_y_max))
        
        rospy.loginfo('禁入区域边界: (%d, %d) - (%d, %d)', 
                      restricted_area_x_min, restricted_area_y_min,
                      restricted_area_x_max, restricted_area_y_max)
        rospy.loginfo('探索区域边界: (%d, %d) - (%d, %d)', 
                      explore_area_x_min, explore_area_y_min,
                      explore_area_x_max, explore_area_y_max)
        
        # 将数据转换为可变列表
        data_list = list(masked_costmap.data)
        
        # 标记禁入区域为障碍物，探索区域为未知，其余不变
        modified_cells = 0
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if idx < len(data_list):
                    if (restricted_area_x_min <= x <= restricted_area_x_max and
                        restricted_area_y_min <= y <= restricted_area_y_max and
                        Config.MASKED_CONFIG['USE_RESTRICTED_MASK']):
                        # 在禁入区域内：标记为障碍物
                        data_list[idx] = 100
                        modified_cells += 1
                    elif (explore_area_x_min <= x <= explore_area_x_max and 
                          explore_area_y_min <= y <= explore_area_y_max and 
                          Config.MASKED_CONFIG['USE_EXPLORE_MASK']):
                        # 在探索区域内：标记为未知
                        data_list[idx] = -1
                        modified_cells += 1
                    else:
                        # 其他区域：保持原值
                        pass
                        
        
        # 转换回元组
        masked_costmap.data = tuple(data_list)
        
        rospy.loginfo('掩码代价地图处理完成: 修改了 %d 个单元格', modified_cells)
        
        return masked_costmap

    def process_costmap_updates(self, update_msg):
        """处理代价地图更新，设置禁入区域为障碍物，探索区域为未知"""
        import copy
        masked_update = copy.deepcopy(update_msg)
        
        # 获取更新区域的信息
        width = masked_update.width
        height = masked_update.height
        x_start = masked_update.x
        y_start = masked_update.y
        
        # 如果没有数据，直接返回
        if not masked_update.data:
            return masked_update
        
        # 获取全局代价地图的信息（用于坐标转换）
        if not hasattr(self, 'last_complete_map') or not self.last_complete_map.info:
            rospy.logwarn('没有全局代价地图信息，无法处理更新')
            return masked_update
        
        # 从完整地图中获取分辨率和原点位置
        resolution = self.last_complete_map.info.resolution if Config.MASKED_CONFIG['USE_RESOLUTION'] else 1.0
        origin_x = self.last_complete_map.info.origin.position.x
        origin_y = self.last_complete_map.info.origin.position.y
        
        # 计算更新区域在实际世界中的绝对坐标
        update_origin_x = origin_x + x_start * resolution
        update_origin_y = origin_y + y_start * resolution
        
        # 获取禁入区域的世界坐标边界
        restricted_x_min_world = Config.RESTRICTED_MAP_BOUNDS['X_MIN']
        restricted_x_max_world = Config.RESTRICTED_MAP_BOUNDS['X_MAX']
        restricted_y_min_world = Config.RESTRICTED_MAP_BOUNDS['Y_MIN']
        restricted_y_max_world = Config.RESTRICTED_MAP_BOUNDS['Y_MAX']
        
        # 获取探索区域的世界坐标边界
        explore_x_min_world = Config.EXPLORE_MAP_BOUNDS['X_MIN']
        explore_x_max_world = Config.EXPLORE_MAP_BOUNDS['X_MAX']
        explore_y_min_world = Config.EXPLORE_MAP_BOUNDS['Y_MIN']
        explore_y_max_world = Config.EXPLORE_MAP_BOUNDS['Y_MAX']
        
        # 将世界坐标转换为更新区域内的局部坐标
        # 首先计算这些边界在全局地图中的单元格坐标
        restricted_x_min_global = int((restricted_x_min_world - origin_x) / resolution)
        restricted_x_max_global = int((restricted_x_max_world - origin_x) / resolution)
        restricted_y_min_global = int((restricted_y_min_world - origin_y) / resolution)
        restricted_y_max_global = int((restricted_y_max_world - origin_y) / resolution)
        
        explore_x_min_global = int((explore_x_min_world - origin_x) / resolution)
        explore_x_max_global = int((explore_x_max_world - origin_x) / resolution)
        explore_y_min_global = int((explore_y_min_world - origin_y) / resolution)
        explore_y_max_global = int((explore_y_max_world - origin_y) / resolution)
        
        # 然后将全局坐标转换为更新区域内的局部坐标
        restricted_x_min_local = restricted_x_min_global - x_start
        restricted_x_max_local = restricted_x_max_global - x_start
        restricted_y_min_local = restricted_y_min_global - y_start
        restricted_y_max_local = restricted_y_max_global - y_start
        
        explore_x_min_local = explore_x_min_global - x_start
        explore_x_max_local = explore_x_max_global - x_start
        explore_y_min_local = explore_y_min_global - y_start
        explore_y_max_local = explore_y_max_global - y_start
        
        # 限制局部坐标在更新区域范围内
        restricted_x_min_local = max(0, min(width-1, restricted_x_min_local))
        restricted_x_max_local = max(0, min(width-1, restricted_x_max_local))
        restricted_y_min_local = max(0, min(height-1, restricted_y_min_local))
        restricted_y_max_local = max(0, min(height-1, restricted_y_max_local))
        
        explore_x_min_local = max(0, min(width-1, explore_x_min_local))
        explore_x_max_local = max(0, min(width-1, explore_x_max_local))
        explore_y_min_local = max(0, min(height-1, explore_y_min_local))
        explore_y_max_local = max(0, min(height-1, explore_y_max_local))
        
        rospy.loginfo('代价地图更新: 局部禁入区域边界: (%d,%d)-(%d,%d)', 
                    restricted_x_min_local, restricted_y_min_local,
                    restricted_x_max_local, restricted_y_max_local)
        rospy.loginfo('代价地图更新: 局部探索区域边界: (%d,%d)-(%d,%d)', 
                    explore_x_min_local, explore_y_min_local,
                    explore_x_max_local, explore_y_max_local)

        # 标记区域 - 将数据转换为列表以便修改
        data_list = list(masked_update.data)
        modified_cells = 0
        
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if idx < len(data_list):
                    if (restricted_x_min_local <= x <= restricted_x_max_local and 
                        restricted_y_min_local <= y <= restricted_y_max_local and
                        Config.MASKED_CONFIG['USE_RESTRICTED_MASK']):
                        # 在禁入区域内：标记为障碍物
                        data_list[idx] = 100
                        modified_cells += 1
                    elif (explore_x_min_local <= x <= explore_x_max_local and 
                        explore_y_min_local <= y <= explore_y_max_local and
                        Config.MASKED_CONFIG['USE_EXPLORE_MASK']):
                        # 在探索区域内：标记为未知
                        data_list[idx] = -1
                        modified_cells += 1
                    else:
                        # 其他区域：保持原值
                        pass
        
        # 转换回元组并更新消息
        masked_update.data = tuple(data_list)
        
        rospy.loginfo('更新掩码已处理: 修改了 %d 个单元格', modified_cells)
        
        return masked_update

# 定义状态：盒子位置检测任务
class DetectBoxPose(smach.State):
    def __init__(self):      
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                             output_keys=['box_positions_out'])
        
        # 添加全局代价地图订阅
        self.costmap_subscriber = rospy.Subscriber(
            Config.TOPICS['BOX_EXTRACTION'], 
            OccupancyGrid, 
            self.costmap_callback
        )
        # 添加盒子位置发布器
        self.box_publisher = rospy.Publisher(
            Config.TOPICS['DETECTED_BOXES'], 
            PoseArray, 
            queue_size=10
        )
        # 添加盒子生成区域发布器
        self.box_area_publisher = rospy.Publisher(
            '/box_area', 
            MarkerArray, 
            queue_size=10
        )

        self.box_positions = []  # 用于存储检测到的盒子位置
        self.detection_timeout = Config.TIMEOUTS['BOX_DETECTION']
        self.costmap = None  # 存储最新的代价地图
               
    def execute(self, userdata):
        # 重置盒子位置列表
        self.box_positions = []

        rospy.loginfo('执行盒子检测任务...')
        try:
            # 使用代价地图检测盒子
            rospy.loginfo('使用代价地图进行盒子检测...')
            costmap_boxes = self.detect_boxes_from_costmap()

            # 如果检测到盒子，保存到userdata中
            if costmap_boxes:
                self.box_positions = costmap_boxes
                # 将盒子姿态数组创建为PoseArray，便于传递
                pose_array = PoseArray()
                pose_array.header.frame_id = "map"
                pose_array.header.stamp = rospy.Time.now()
                pose_array.poses = [box.pose for box in costmap_boxes]
                userdata.box_positions_out = pose_array
                rospy.loginfo('成功从代价地图检测到%d个盒子', len(costmap_boxes))
                
                # 发布检测到的盒子位置
                self.box_publisher.publish(pose_array)
                return 'succeeded'
            else:
                rospy.logwarn('从代价地图未检测到盒子')
                return 'failed'
        
        except Exception as e:
            rospy.logerr('盒子检测错误: %s', str(e))
            return 'failed'    
       
    def costmap_callback(self, msg):
        """处理接收到的代价地图"""
        self.costmap = msg

    def detect_boxes_from_costmap(self):
        """从代价地图中提取盒子位置"""
        if self.costmap is None:
            rospy.logwarn("无法从代价地图检测盒子：代价地图未接收")
            return []
            
        # 将代价地图转换为numpy数组进行处理
        rospy.loginfo('正在处理代价地图以检测盒子...')
        width = self.costmap.info.width
        height = self.costmap.info.height
        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        rospy.loginfo('代价地图分辨率: %.2f m/pixel', resolution)
        rospy.loginfo('代价地图原点: (%.2f, %.2f)', origin_x, origin_y)
        
        # 将一维数组转为二维网格
        grid = np.array(self.costmap.data).reshape((height, width))
        
        # 检测高占用值区域（障碍物/盒子）
        obstacle_threshold = Config.OBSTACLE_THRESHOLD
        obstacles = np.where(grid > obstacle_threshold)
        
        # 聚类 - 使用简单的基于距离的聚类
        from sklearn.cluster import DBSCAN
        
        # 如果没有障碍物点，返回空列表
        if len(obstacles[0]) == 0:
            rospy.logwarn("未检测到障碍物点")
            return []
        else:
            rospy.loginfo("检测到 %d 个障碍物点", len(obstacles[0]))
            
        # 将障碍物点转换为坐标点列表
        points = np.column_stack([obstacles[1], obstacles[0]])  # x对应列，y对应行
        
        # 使用DBSCAN进行聚类
        clustering = DBSCAN(eps=5, min_samples=10).fit(points)
        labels = clustering.labels_
        
        # 计算每个聚类的中心点
        box_positions = []
        unique_labels = set(labels)

        for label in unique_labels:
            # 跳过噪声点（标签为-1）
            if label == -1:
                continue
                
            # 获取当前聚类的所有点
            cluster_points = points[labels == label]
            
            # 计算聚类大小
            cluster_size = len(cluster_points)
            
            # 计算聚类的边界框
            min_x, min_y = np.min(cluster_points, axis=0)
            max_x, max_y = np.max(cluster_points, axis=0)
            width = max_x - min_x
            height = max_y - min_y
            
            # 过滤条件: 盒子应该是小型紧凑的聚类
            # 1. 聚类点数不应太多 (根据实际盒子大小调整)
            # 2. 形状应接近正方形
            # 3. 大小应该在合理范围内
            if (Config.CLUSTERING['MIN_POINTS'] <= cluster_size <= Config.CLUSTERING['MAX_POINTS'] and  # 点数范围
                width < Config.CLUSTERING['width'] and height < Config.CLUSTERING['height']):  # 大小限制(单位是格子数)
                
                # 计算聚类中心
                center_x = np.mean(cluster_points[:, 0])
                center_y = np.mean(cluster_points[:, 1])
                
                # 将中心点转换为地图坐标
                map_x = center_x * resolution + origin_x
                map_y = center_y * resolution + origin_y
            
                # 创建姿态消息
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = map_x
                pose.pose.position.y = map_y
                pose.pose.position.z = 0.0
                # 默认朝向，方向为正前方
                pose.pose.orientation.w = 1.0
                
                # 只检测探索区的盒子
                rospy.loginfo('盒子位置: x=%.2f, y=%.2f', map_x, map_y)
            
                # 发布探索区域
                self.publish_explore_area()

                # 检查盒子是否在探索区域内
                rospy.loginfo('检查盒子是否在探索区域内...')
                if (Config.EXPLORE_MAP_BOUNDS['X_MIN'] <= map_x <= Config.EXPLORE_MAP_BOUNDS['X_MAX'] and
                    Config.EXPLORE_MAP_BOUNDS['Y_MIN'] <= map_y <= Config.EXPLORE_MAP_BOUNDS['Y_MAX']):
                    box_positions.append(pose)
                    rospy.loginfo(f"从代价地图检测到盒子: x={map_x:.2f}, y={map_y:.2f}")
                else:
                    rospy.loginfo(
                        f"盒子位置x={map_x:.2f}, y={map_y:.2f}超出探索区域范围{Config.EXPLORE_MAP_BOUNDS['X_MIN']:.2f}~{Config.EXPLORE_MAP_BOUNDS['X_MAX']:.2f}, {Config.EXPLORE_MAP_BOUNDS['Y_MIN']:.2f}~{Config.EXPLORE_MAP_BOUNDS['Y_MAX']:.2f}")
        return box_positions

    def publish_explore_area(self):
        """发布探索区域的可视化标记"""
        from visualization_msgs.msg import Marker, MarkerArray
        from std_msgs.msg import ColorRGBA
        from geometry_msgs.msg import Point
        
        # 创建标记数组
        marker_array = MarkerArray()
        
        # 创建区域边界标记
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "explore_area"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 线宽
        
        # 设置标记颜色为绿色，透明度为0
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.0)  # R, G, B, A
        
        # 添加探索区域的四个角点，形成闭环
        points = []
        # 左下角
        p1 = Point()
        p1.x = Config.EXPLORE_MAP_BOUNDS['X_MIN']
        p1.y = Config.EXPLORE_MAP_BOUNDS['Y_MIN']
        p1.z = 0.1  # 略高于地面
        points.append(p1)
        
        # 右下角
        p2 = Point()
        p2.x = Config.EXPLORE_MAP_BOUNDS['X_MAX']
        p2.y = Config.EXPLORE_MAP_BOUNDS['Y_MIN']
        p2.z = 0.1
        points.append(p2)
        
        # 右上角
        p3 = Point()
        p3.x = Config.EXPLORE_MAP_BOUNDS['X_MAX']
        p3.y = Config.EXPLORE_MAP_BOUNDS['Y_MAX']
        p3.z = 0.1
        points.append(p3)
        
        # 左上角
        p4 = Point()
        p4.x = Config.EXPLORE_MAP_BOUNDS['X_MIN']
        p4.y = Config.EXPLORE_MAP_BOUNDS['Y_MAX']
        p4.z = 0.1
        points.append(p4)
        
        # 回到起点，形成闭环
        points.append(p1)
        
        marker.points = points
        marker.lifetime = rospy.Duration(5.0)  # 标记持续显示5秒
        
        # 创建填充区域标记
        fill_marker = Marker()
        fill_marker.header.frame_id = "map"
        fill_marker.header.stamp = rospy.Time.now()
        fill_marker.ns = "explore_area_fill"
        fill_marker.id = 1
        fill_marker.type = Marker.TRIANGLE_LIST
        fill_marker.action = Marker.ADD
        fill_marker.pose.orientation.w = 1.0
        fill_marker.scale.x = 1.0
        fill_marker.scale.y = 1.0
        fill_marker.scale.z = 1.0
        
        # 设置填充颜色为浅绿色，半透明
        fill_marker.color = ColorRGBA(0.0, 0.8, 0.2, 0.3)  # R, G, B, A
        
        # 创建两个三角形来填充矩形区域
        fill_points = []
        # 三角形1: 左下 - 右下 - 左上
        fill_points.extend([p1, p2, p4])
        # 三角形2: 右下 - 右上 - 左上
        fill_points.extend([p2, p3, p4])
        
        fill_marker.points = fill_points
        fill_marker.lifetime = rospy.Duration(5.0)
        
        # 将标记添加到数组
        marker_array.markers.append(marker)
        marker_array.markers.append(fill_marker)
        
        # 发布标记数组
        self.box_area_publisher.publish(marker_array)
        rospy.loginfo("已发布探索区域可视化")

# 定义状态：导航至每个盒子并启动OCR
class NavigateToBoxAndOCR(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'], 
                           input_keys=['box_positions_in'])
        self.ocr_trigger_publisher = rospy.Publisher(
            Config.TOPICS['OCR_TRIGGER'], 
            Bool, 
            queue_size=10
        )
        self.ocr_result_subscriber = rospy.Subscriber(
            Config.TOPICS['RECOGNIZED_DIGIT'], 
            Int32, 
            self.ocr_result_callback
        )
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.view_positions_publisher = rospy.Publisher(
                '/box_view_positions', 
                MarkerArray, 
                queue_size=10
            )
        self.client.wait_for_server()
        rospy.loginfo('导航客户端已连接')
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.ocr_timeout = Config.TIMEOUTS['OCR_PROCESSING']
        self.ocr_result = None  # 用于存储OCR结果
  
    def execute(self, userdata):
        # 从userdata中获取盒子位置
        self.box_positions = userdata.box_positions_in

        wait_timeout = rospy.Duration(10.0)  # 设置合理的超时时间
        start_time = rospy.Time.now()
        
        while not self.box_positions and (rospy.Time.now() - start_time) < wait_timeout:
            rospy.loginfo("等待接收盒子位置数据...")
            rospy.sleep(0.5)  # 短暂休眠，避免CPU占用过高
        
        if not self.box_positions:
            rospy.logwarn('没有盒子位置可供导航')
            return 'failed'

        rospy.loginfo('导航到%d个盒子并启动OCR...', len(self.box_positions.poses))
        
        # 创建PoseArray存储提取的姿态
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        # 对每个盒子进行导航和OCR
        for i, pose in enumerate(self.box_positions.poses):
            try:
                self.navigate_to_best_viewing_positions(pose)
                
            except Exception as e:
                rospy.logerr('处理盒子时发生错误: %s', str(e))
                
        return 'succeeded'

    def box_positions_callback(self, msg):
        """处理盒子位置订阅"""
        # rospy.loginfo('接收到盒子位置消息，数量: %d', len(msg.markers))
        # 保存位置
        self.box_positions = msg

    def navigate_to_best_viewing_positions(self, box_pose):
        """计算并导航到盒子周围的最佳观察位置"""     
        # 计算观察位置
        viewing_positions = self.calculate_box_viewing_positions(box_pose)
        
        # 发布可视化标记
        self.visualize_box_viewing_positions(box_pose, viewing_positions)
               
        for i, viewing_position in enumerate(viewing_positions):

            # 创建朝向盒子的姿态
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = viewing_position['position']['x']
            goal.target_pose.pose.position.y = viewing_position['position']['y']
            goal.target_pose.pose.position.z = 0.0
            
            # 将角度转换为四元数
            goal.target_pose.pose.orientation.z = viewing_position['orientation']['z']
            goal.target_pose.pose.orientation.w = viewing_position['orientation']['w']
            
            rospy.loginfo('导航到盒子观察位置 [%d/4]: x=%.2f, y=%.2f, 角度=%.2f°', 
                        i+1, viewing_position['position']['x'],
                        viewing_position['position']['y'],
                        math.degrees(viewing_position['angle']))
                        
            self.client.send_goal(goal)
            
            # 等待导航结果，添加超时
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('导航到观察位置超时，尝试下一个位置')
                continue
            
            if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('导航到观察位置失败，尝试下一个位置')
                continue
            
            # 到达观察位置后，触发OCR
            ocr_trigger_msg = Bool()
            ocr_trigger_msg.data = True
            self.ocr_trigger_publisher.publish(ocr_trigger_msg)
            rospy.loginfo('在观察位置触发OCR...')
            rospy.sleep(1.0)  # 给OCR处理时间

            # 如果OCR处理完成，话题/recognized_digit(Int32)会发布结果
            # 识别成功，跳出循环
            if self.ocr_result is not None:  # 修改判断条件，检查是否有任何结果
                rospy.loginfo('OCR处理成功，识别到数字: %d', self.ocr_result)
                self.ocr_result = None  # 重置结果以便下一次识别
                break
            else:
                rospy.logwarn('OCR处理失败，尝试下一个观察位置')
                continue

    def calculate_box_viewing_positions(self, box_pose):
        """
        计算盒子周围的最佳观察位置
        
        参数:
            box_pose: 盒子的姿态，包含position和orientation
            
        返回:
            viewing_positions: 包含观察位置的列表，每个位置是一个字典，包含:
                - position: 观察位置坐标(x, y, z)
                - orientation: 观察位置的朝向(四元数)
                - angle: 观察角度(弧度)
        """
        # 定义4个最佳观察位置（盒子的四周）
        viewing_angles = Config.VIEWING_ANGLES  # 0°, 90°, 180°, 270°
        viewing_distance = Config.VIEWING_DISTANCE  # 最佳观察距离
        
        viewing_positions = []
        
        for i, angle in enumerate(viewing_angles):
            # 计算观察位置
            view_x = box_pose.position.x + viewing_distance * math.cos(angle)
            view_y = box_pose.position.y + viewing_distance * math.sin(angle)
            
            # 计算朝向盒子的角度
            dx = box_pose.position.x - view_x
            dy = box_pose.position.y - view_y
            facing_angle = math.atan2(dy, dx)  # atan2正确的参数顺序是(y, x)
            
            # 创建四元数表示朝向
            orientation = {
                'z': math.sin(facing_angle / 2.0),
                'w': math.cos(facing_angle / 2.0)
            }
            
            # 保存观察位置信息
            position = {
                'x': view_x,
                'y': view_y,
                'z': 0.0
            }
            
            # 添加到结果列表
            viewing_positions.append({
                'position': position,
                'orientation': orientation,
                'angle': facing_angle
            })
        
        return viewing_positions
    
    def visualize_box_viewing_positions(self, box_pose, viewing_positions):
        """可视化盒子位置最佳观察位置标记"""
        # 创建标记数组
        marker_array = MarkerArray()

        # 创建盒子标记
        box_marker = Marker()
        box_marker.header.frame_id = "map"
        box_marker.header.stamp = rospy.Time.now()
        box_marker.ns = "box_visualization"
        box_marker.id = 0
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        
        # 设置盒子位置和大小
        box_marker.pose = box_pose
        box_marker.scale.x = Config.BOX_SIZE
        box_marker.scale.y = Config.BOX_SIZE
        box_marker.scale.z = Config.BOX_SIZE
        
        # 设置盒子颜色为红色
        box_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.0)  # R, G, B, A
        box_marker.lifetime = rospy.Duration(10.0)  # 显示10秒
        
        marker_array.markers.append(box_marker)
        
        # 创建观察位置标记
        for i, viewing_position in enumerate(viewing_positions):            
            # 创建观察位置标记
            view_marker = Marker()
            view_marker.header.frame_id = "map"
            view_marker.header.stamp = rospy.Time.now()
            view_marker.ns = "view_position"
            view_marker.id = i + 1  # 从1开始，0是盒子
            view_marker.type = Marker.ARROW  # 使用箭头表示机器人朝向
            view_marker.action = Marker.ADD
            
            # 设置观察位置
            view_marker.pose.position.x = viewing_position['position']['x']
            view_marker.pose.position.y = viewing_position['position']['y']
            view_marker.pose.position.z = 0.1  # 略高于地面
            
            # 设置朝向（四元数）
            view_marker.pose.orientation.z = viewing_position['orientation']['z']
            view_marker.pose.orientation.w = viewing_position['orientation']['w']
            
            # 设置箭头大小
            view_marker.scale.x = 0.3  # 箭头长度
            view_marker.scale.y = 0.1  # 箭头宽度
            view_marker.scale.z = 0.1  # 箭头高度
            
            # 设置颜色，使用不同颜色区分四个位置
            colors = [
                ColorRGBA(0.0, 0.8, 0.0, 0.8),  # 绿色 - 0°
                ColorRGBA(0.0, 0.0, 0.8, 0.8),  # 蓝色 - 90°
                ColorRGBA(0.8, 0.8, 0.0, 0.8),  # 黄色 - 180°
                ColorRGBA(0.8, 0.0, 0.8, 0.8)   # 紫色 - 270°
            ]
            view_marker.color = colors[i]
            view_marker.lifetime = rospy.Duration(10.0)  # 显示10秒
            
            marker_array.markers.append(view_marker)
            
            # 添加连接线，从观察位置指向盒子
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "view_lines"
            line_marker.id = i + 5  # 从5开始，避免ID冲突
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # 添加线的起点和终点
            start_point = Point()
            start_point.x = viewing_position['position']['x']
            start_point.y = viewing_position['position']['y']
            start_point.z = 0.1
            
            end_point = Point()
            end_point.x = box_pose.position.x
            end_point.y = box_pose.position.y
            end_point.z = 0.1
            
            line_marker.points = [start_point, end_point]
            
            # 设置线的粗细和颜色
            line_marker.scale.x = 0.03  # 线宽
            line_marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.5)  # 灰色半透明
            line_marker.lifetime = rospy.Duration(10.0)  # 显示10秒
            
            marker_array.markers.append(line_marker)
        
        # 创建文本标记，显示观察位置编号
        for i, viewing_position in enumerate(viewing_positions):
            # 创建文本标记
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "view_text"
            text_marker.id = i + 9  # 避免ID冲突
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 设置文本位置（稍高一点）
            text_marker.pose.position.x = viewing_position['position']['x']
            text_marker.pose.position.y = viewing_position['position']['y']
            text_marker.pose.position.z = 0.3  # 文本高度
            
            # 设置文本内容和外观
            text_marker.text = f"位置 {i+1}"
            text_marker.scale.z = 0.2  # 文本大小
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 0.8)  # 白色文本
            text_marker.lifetime = rospy.Duration(10.0)  # 显示10秒
            
            marker_array.markers.append(text_marker)
        
        # 发布标记数组    
        self.view_positions_publisher.publish(marker_array)
        rospy.loginfo("已发布盒子及其观察位置可视化")

    def ocr_result_callback(self, msg):
        self.ocr_result = msg.data

# 定义状态：桥检测任务  
class DetectBridge(smach.State):
    def __init__(self):
        # 添加输出键bridge_entrance和bridge_exit
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'failed'],
                            output_keys=['bridge_entrance_out', 'bridge_exit_out'])
        self.bridge_detection_trigger_publisher = rospy.Publisher(
            Config.TOPICS['BRIDGE_DETECTION_TRIGGER'], 
            Bool, 
            queue_size=10
        )
        self.bridge_pose_subscriber = rospy.Subscriber(
            Config.TOPICS['DETECTED_BRIDGES'], 
            PoseStamped, 
            self.bridge_callback
        )
        self.bridge_entrance = None  # 桥入口坐标
        self.detection_complete = False  # 检测完成标志
        self.detection_timeout = Config.TIMEOUTS['BRIDGE_DETECTION']
        
    def execute(self, userdata):
        # 重置检测状态
        self.bridge_entrance = None
        self.detection_complete = False
        
        rospy.loginfo('执行桥检测任务...')
        try:
            # 发布桥检测触发消息
            bridge_detection_trigger_msg = Bool()
            bridge_detection_trigger_msg.data = True
            self.bridge_detection_trigger_publisher.publish(bridge_detection_trigger_msg)
            rospy.loginfo('桥检测触发消息已发布')
            
            # 等待桥梁检测结果
            start_time = rospy.Time.now()
            
            rate = rospy.Rate(2)  # 2Hz检查频率
            while not self.detection_complete and (rospy.Time.now() - start_time).to_sec() < self.detection_timeout:
                rate.sleep()
                
            if self.bridge_entrance is None:
                rospy.logwarn('未检测到桥梁或检测超时')
                return 'failed'
            
            # 计算出口坐标（入口坐标y值加上偏移量）
            bridge_exit = PoseStamped()
            bridge_exit.header = self.bridge_entrance.header
            bridge_exit.pose.position.x = self.bridge_entrance.pose.position.x
            bridge_exit.pose.position.y = self.bridge_entrance.pose.position.y + Config.BRIDGE['EXIT_Y_OFFSET']
            bridge_exit.pose.orientation = self.bridge_entrance.pose.orientation
            
            # 将入口和出口坐标保存到userdata中传递给下一个状态
            userdata.bridge_entrance_out = self.bridge_entrance
            userdata.bridge_exit_out = bridge_exit
            
            rospy.loginfo('桥梁检测成功')
            rospy.loginfo('桥入口坐标: x=%.2f, y=%.2f', 
                          self.bridge_entrance.pose.position.x, 
                          self.bridge_entrance.pose.position.y)
            rospy.loginfo('桥出口坐标: x=%.2f, y=%.2f', 
                          bridge_exit.pose.position.x, 
                          bridge_exit.pose.position.y)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('桥梁检测错误: %s', str(e))
            return 'failed'
    
    def bridge_callback(self, msg):
        rospy.loginfo('检测到桥，位置：x=%.2f, y=%.2f', msg.pose.position.x, msg.pose.position.y)
        self.bridge_entrance = msg
        self.detection_complete = True

# 定义状态：导航到桥入口
class NavigateToBridgeEntrance(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'],
                           input_keys=['bridge_entrance_in', 'bridge_exit_in'],
                           output_keys=['bridge_entrance_out', 'bridge_exit_out'])
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        
    def execute(self, userdata):
        # 从userdata中获取桥梁入口坐标
        bridge_entrance = userdata.bridge_entrance_in
        
        if bridge_entrance is None:
            rospy.logwarn('缺少桥梁入口坐标信息')
            return 'failed'
        
        rospy.loginfo('开始导航到桥入口...')
        
        try:
            # 导航到桥入口
            rospy.loginfo('导航到桥入口: x=%.2f, y=%.2f', 
                         bridge_entrance.pose.position.x, 
                         bridge_entrance.pose.position.y)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = bridge_entrance.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = bridge_entrance.pose
            
            self.client.send_goal(goal)
            
            # 添加超时
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('导航到桥入口超时')
                return 'failed'
                
            result = self.client.get_state()
            
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('导航到桥入口失败')
                return 'failed'
            
            rospy.loginfo('已成功到达桥入口')
            # 传递坐标到下一个状态
            userdata.bridge_entrance_out = bridge_entrance
            userdata.bridge_exit_out = userdata.bridge_exit_in
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('导航到桥入口时出错: %s', str(e))
            return 'failed'

# 定义状态：开桥并导航过桥
class OpenBridgeAndNavigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'failed'],
                           input_keys=['bridge_exit_in'])
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.open_bridge_publisher = rospy.Publisher(
            Config.TOPICS['OPEN_BRIDGE'], 
            Bool, 
            queue_size=10
        )
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.bridge_open_wait_time = Config.TIMEOUTS['BRIDGE_OPEN']
        
    def execute(self, userdata):
        # 从userdata中获取桥梁出口坐标
        bridge_exit = userdata.bridge_exit_in
        
        if bridge_exit is None:
            rospy.logwarn('缺少桥梁出口坐标信息')
            return 'failed'
        
        try:
            # 发送开桥指令
            open_bridge_msg = Bool()
            open_bridge_msg.data = True
            self.open_bridge_publisher.publish(open_bridge_msg)
            rospy.loginfo('已发送开桥指令')
            
            # 给桥梁足够的打开时间
            rospy.sleep(self.bridge_open_wait_time)
            
            # 导航到桥出口（过桥）
            rospy.loginfo('开始过桥，导航到桥出口: x=%.2f, y=%.2f', 
                         bridge_exit.pose.position.x, 
                         bridge_exit.pose.position.y)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = bridge_exit.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = bridge_exit.pose
            
            self.client.send_goal(goal)
            
            # 添加超时
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('过桥导航超时')
                return 'failed'
                
            result = self.client.get_state()
            
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('过桥失败')
                return 'failed'
            
            rospy.loginfo('成功过桥')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('过桥时出错: %s', str(e))
            return 'failed'

# 定义状态：导航到目标并启动OCR
class NavigateToGoalAndOCR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # 获取目标盒子的坐标列表
        self.goals = Config.GOALS
        
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        self.ocr_trigger_publisher = rospy.Publisher(
            Config.TOPICS['OCR_TRIGGER'], 
            Bool, 
            queue_size=10
        )

        # 一旦匹配到目标盒子，OCR节点会发布/cmd_stop
        self.cmd_stop_subscriber = rospy.Subscriber(
            Config.TOPICS['CMD_STOP'], 
            Bool, 
            self.cmd_stop_callback
        )
        self.cmd_stop_received = False  # 标记是否收到停止指令
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.ocr_timeout = Config.TIMEOUTS['OCR_PROCESSING']
        
    def execute(self, userdata):
        rospy.loginfo('执行导航到目标任务...')
        
        try:
            # 遍历目标坐标，依次导航
            for i, goal_position in enumerate(self.goals):
                self.goal.target_pose.pose.position.x = goal_position[0]
                self.goal.target_pose.pose.position.y = goal_position[1]
                self.goal.target_pose.pose.orientation.w = goal_position[2]
                
                rospy.loginfo('[%d/%d] 导航到目标位置: x=%.2f, y=%.2f', 
                              i+1, len(self.goals),
                              goal_position[0], goal_position[1])
                              
                self.client.send_goal(self.goal)
                
                # 添加超时
                if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                    rospy.logwarn('导航到目标位置超时，尝试下一个目标')
                    continue
                    
                result = self.client.get_state()
                
                if result == actionlib.GoalStatus.SUCCEEDED:
                    # 启动OCR处理
                    ocr_trigger_msg = Bool()
                    ocr_trigger_msg.data = True
                    self.ocr_trigger_publisher.publish(ocr_trigger_msg)
                    rospy.loginfo('OCR触发消息已发布，等待处理...')
                    
                    # 等待OCR处理完成
                    wait_start_time = rospy.Time.now()
                    rate = rospy.Rate(2)  # 2Hz检查频率
                    
                    while (rospy.Time.now() - wait_start_time).to_sec() < self.ocr_timeout:
                        if self.cmd_stop_received:
                            rospy.loginfo('OCR任务完成，停止导航')
                            self.cmd_stop_received = False
                            return 'succeeded'
                        rate.sleep()
                        
                    rospy.loginfo('OCR处理超时，继续导航到下一目标')
                else:
                    rospy.logwarn('导航到目标位置失败，尝试下一个目标')

            rospy.loginfo('所有目标位置导航完成')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('导航到目标时出错: %s', str(e))
            return 'failed'
    
    def cmd_stop_callback(self, msg):
        if msg.data:
            rospy.loginfo('收到停止指令，OCR任务完成')
            self.cmd_stop_received = True
        else:
            rospy.loginfo('未收到停止指令，继续等待')

# 主函数
def main():
    # 初始化ROS节点
    rospy.init_node('task_coordinator')

    # 创建顶层状态机
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    
    # 添加状态到状态机
    with sm:
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'initialized':'EXPLORE_FRONTIER', # 暂时跳过探索
                                           'failed':'mission_failed'})
                
        smach.StateMachine.add('EXPLORE_FRONTIER', ExploreFrontier(),
                               transitions={'succeeded':'DETECT_BOX_POSE', 
                                           'failed':'mission_failed'})
        
        smach.StateMachine.add('DETECT_BOX_POSE', DetectBoxPose(), 
                               transitions={'succeeded':'NAVIGATE_TO_BOX_AND_OCR', 
                                           'failed':'mission_failed'},
                               remapping={'box_positions_out':'box_positions'})
        
        smach.StateMachine.add('NAVIGATE_TO_BOX_AND_OCR', NavigateToBoxAndOCR(),
                               transitions={'succeeded':'DETECT_BRIDGE', 
                                           'failed':'mission_failed'},
                               remapping={'box_positions_in':'box_positions'})
        
        smach.StateMachine.add('DETECT_BRIDGE', DetectBridge(),
                               transitions={'succeeded':'NAVIGATE_TO_BRIDGE_ENTRANCE', 
                                           'failed':'mission_failed'},
                               remapping={'bridge_entrance_out':'bridge_entrance',
                                          'bridge_exit_out':'bridge_exit'})
        
        smach.StateMachine.add('NAVIGATE_TO_BRIDGE_ENTRANCE', NavigateToBridgeEntrance(),
                               transitions={'succeeded':'OPEN_BRIDGE_AND_NAVIGATE', 
                                           'failed':'mission_failed'},
                               remapping={'bridge_entrance_in':'bridge_entrance',
                                          'bridge_entrance_out':'bridge_entrance',
                                          'bridge_exit_in':'bridge_exit',
                                          'bridge_exit_out':'bridge_exit'})
        
        smach.StateMachine.add('OPEN_BRIDGE_AND_NAVIGATE', OpenBridgeAndNavigate(),
                               transitions={'succeeded':'NAVIGATE_TO_GOAL_AND_OCR', 
                                           'failed':'mission_failed'},
                               remapping={'bridge_exit_in':'bridge_exit'})
        
        smach.StateMachine.add('NAVIGATE_TO_GOAL_AND_OCR', NavigateToGoalAndOCR(),
                               transitions={'succeeded':'mission_completed', 
                                           'failed':'mission_failed'})
    
    # 创建状态机的可视化工具
    sis = smach_ros.IntrospectionServer('task_coordinator', sm, '/TASK_COORDINATOR')
    sis.start()
    
    # 执行状态机
    outcome = sm.execute()
    
    rospy.loginfo('状态机执行完毕，结果：%s', outcome)
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()