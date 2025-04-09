#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int32

# Import configurations
from config import Config

# Import utility functions
from utils import (detect_object_from_costmap, 
                   visualize_area,  
                   navigate_to_goal,
                   navigate_to_best_viewing_positions_and_visualize_and_ocr)

# =============================== Global Variables =========================
latest_ocr_result = None  # Global variable to store the latest OCR result
# ============================== State Machine Definition =========================
# Define state: Initialize
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.timeout = Config.TIMEOUTS['INIT']

    def execute(self, userdata):
        return 'initialized'

# Define state: Frontier Exploration Task
class ExploreFrontier(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])    
        
        # Add frontier subscriber - listen to frontier points visualization published by explore_lite
        self.frontier_subscriber = rospy.Subscriber(
            Config.TOPICS['FRONTIERS'], 
            MarkerArray, 
            self.frontier_callback
        )
        
        # Frontier point count and threshold setting
        self.frontier_count = 999  # Initialize to a large number
        self.frontier_threshold = Config.EXPLORE['FRONTIER_THRESHOLD']  
        self.max_explore_time = Config.TIMEOUTS['EXPLORE']  

        self.last_frontier_time = rospy.Time.now()  # Last frontier update time
        
    def execute(self, userdata):
        rospy.loginfo('Starting frontier exploration task...')
        rospy.loginfo('Starting to monitor frontier point count, threshold is %d', self.frontier_threshold)
        
        # Main processing loop
        rospy.loginfo('Map data ready, starting exploration...')
        previous_frontier_count = self.frontier_count
        
        while (rospy.Time.now() - self.last_frontier_time).to_sec() < self.max_explore_time:
            # If frontier point count changes
            if self.frontier_count != previous_frontier_count:
                rospy.loginfo('Frontier point count changed: %d -> %d', previous_frontier_count, self.frontier_count)
                previous_frontier_count = self.frontier_count
            # If frontier point count is less than threshold or timeout, exploration is considered complete
            if self.frontier_count <= self.frontier_threshold:
                rospy.loginfo('Frontier point count below threshold, exploration task complete')
                return 'succeeded'
            else:
                continue  # Continue loop, check next time

        # Stop exploration
        rospy.loginfo('Frontier exploration timeout, stopping task')
        
        return 'succeeded'  # Return success status

    def frontier_callback(self, msg):
        self.frontier_count = len(msg.markers)
        self.last_frontier_time = rospy.Time.now()

# Define state: Box and Bridge Position Detection Task
class DetectBoxPoseNavigateAndOCR(smach.State):
    def __init__(self):      
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                             output_keys=['costmap_out'])
        
        # Add bounding box publisher
        self.bounding_box_publisher = rospy.Publisher(
            Config.TOPICS['DETECTED_BOUNDING_BOXES'], 
            MarkerArray, 
            queue_size=10
        )

        # Add box generation area publisher
        self.box_area_publisher = rospy.Publisher(
            '/box_area', 
            MarkerArray, 
            queue_size=10
        )
        
        self.detection_timeout = Config.TIMEOUTS['BOX_DETECTION']
        self.costmap = None  # Store latest costmap
        self.bridge_entrance = None
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
        rospy.loginfo('Navigation client connected')
    
    def execute(self, userdata):
        self.costmap_subscriber = rospy.Subscriber(
            Config.TOPICS['BOX_EXTRACTION'], 
            OccupancyGrid, 
            self.costmap_callback,
            queue_size=50
        )

        rospy.loginfo('Executing box detection task...')
        
        # Set maximum retry count and retry interval
        max_retries = 5
        retry_interval = 1.0  # seconds
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                # Check if costmap is ready
                if self.costmap is None:
                    retry_count += 1
                    rospy.logwarn('Costmap not received, retrying %d/%d...', retry_count, max_retries)
                    rospy.sleep(retry_interval)
                    continue
                else:
                    userdata.costmap_out = self.costmap  # Pass costmap to next state
                
                # Visualize box area
                visualize_area(x_min=Config.EXPLORE_MAP_BOUNDS['X_MIN'],
                               y_min=Config.EXPLORE_MAP_BOUNDS['Y_MIN'],
                               x_max=Config.EXPLORE_MAP_BOUNDS['X_MAX'],
                               y_max=Config.EXPLORE_MAP_BOUNDS['Y_MAX'],
                               area_publisher=self.box_area_publisher,
                               description='box_area')
                rospy.loginfo('Using costmap for box detection...')
                costmap_boxes, costmap_bounding_boxes = detect_object_from_costmap(self.costmap, eps=Config.CLUSTERING['EPS'],
                                                                                 min_samples=Config.CLUSTERING['MIN_SAMPLES'],
                                                                                 min_cluster_size=Config.CLUSTERING['MIN_POINTS'],
                                                                                 max_cluster_size=Config.CLUSTERING['MAX_POINTS'],
                                                                                 width_object=Config.CLUSTERING['width'],
                                                                                 height_object=Config.CLUSTERING['height'],
                                                                                 description='boxes',
                                                                                 area_min_x_threshold=Config.EXPLORE_MAP_BOUNDS['X_MIN'],
                                                                                 area_max_x_threshold=Config.EXPLORE_MAP_BOUNDS['X_MAX'],
                                                                                 area_min_y_threshold=Config.EXPLORE_MAP_BOUNDS['Y_MIN'],
                                                                                 area_max_y_threshold=Config.EXPLORE_MAP_BOUNDS['Y_MAX'],
                                                                                 obstacle_threshold=Config.OBSTACLE_THRESHOLD)
                # If boxes are detected, go and ocr
                if costmap_boxes:
                    for i, pose in enumerate(costmap_boxes):
                        try:                            
                            navigate_to_best_viewing_positions_and_visualize_and_ocr(Config.VIEWING_ANGLES, 
                                                                                     Config.VIEWING_DISTANCE,
                                                                                     pose,
                                                                                     Config.BOX_SIZE,
                                                                                     Config.EXPLORE_MAP_BOUNDS['X_MIN'], 
                                                                                     Config.EXPLORE_MAP_BOUNDS['Y_MIN'],
                                                                                     Config.EXPLORE_MAP_BOUNDS['X_MAX'], 
                                                                                     Config.EXPLORE_MAP_BOUNDS['Y_MAX'],
                                                                                     self.client,
                                                                                     Config.TIMEOUTS['NAVIGATION'],
                                                                                     self.ocr_trigger_publisher, 
                                                                                     self.view_positions_publisher,)
                            # Check if OCR was successful
                            if latest_ocr_result is not None:
                                rospy.loginfo('Successfully recognized digit for box %d: %d', i+1, latest_ocr_result)
                            
                        except Exception as e:
                            rospy.logerr('Error processing box: %s', str(e))
                            # Continue with the next box
                    
                    else:
                        rospy.logwarn('Failed to process any boxes')
                        # Consider it a success even if no boxes were processed (e.g., no boxes to process)
                        return 'succeeded'

                else:
                    retry_count += 1
                    rospy.logwarn('No boxes detected from costmap, retrying %d/%d...', retry_count, max_retries)
                    # Wait a while before retrying
                    rospy.sleep(retry_interval)
                    continue

            except Exception as e:
                retry_count += 1
                rospy.logerr('Box detection error: %s, retrying %d/%d...', str(e), retry_count, max_retries)
                rospy.sleep(retry_interval)
        
        # If all retries fail, return failure
        rospy.logerr('Failed to detect boxes after %d attempts, giving up', max_retries)
        return 'failed'
              
    def costmap_callback(self, msg):
        """Process received costmap"""
        self.costmap = msg
        rospy.loginfo('Received costmap, width: %d, height: %d', msg.info.width, msg.info.height)
    
    def ocr_result_callback(self, msg):
        global latest_ocr_result
        latest_ocr_result = msg.data
        rospy.loginfo('Received OCR result: %d', latest_ocr_result)

# Define state: Bridge Position Detection Task
class DetectBridgeAndToEntrance(smach.State):
    def __init__(self):      
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                             input_keys=['costmap_in'],)
        
        # Add bridge entrance publisher
        self.detected_bridge_publisher = rospy.Publisher(
            Config.TOPICS['DETECTED_BRIDGES'], 
            MarkerArray, 
            queue_size=10
        )
        
        # River area visualization publisher
        self.river_area_publisher = rospy.Publisher(
            'river_area', 
            MarkerArray, 
            queue_size=10
        )

        # move_base client
        self.client = actionlib.SimpleActionClient(
            Config.TOPICS['MOVE_BASE'], 
            MoveBaseAction
        )
        self.client.wait_for_server()
        rospy.loginfo('Navigation client connected')
        
        self.detection_timeout = Config.TIMEOUTS['BRIDGE_DETECTION']
        self.costmap = None  # Store the latest costmap
        self.costmap_subscriber = None
               
    def execute(self, userdata):
        # Get costmap from previous state
        # self.costmap = userdata.costmap_in

        # Test: subscribe to costmap topic
        self.costmap_subscriber = rospy.Subscriber(
            Config.TOPICS['BOX_EXTRACTION'], 
            OccupancyGrid, 
            self.costmap_callback,
            queue_size=50
        )
        
        rospy.loginfo('Starting bridge detection task...')
        
        # Set maximum retry count and retry interval
        max_retries = 5
        retry_interval = 1.0  # seconds
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                # Check if costmap is ready
                if self.costmap is None:
                    retry_count += 1
                    rospy.logwarn('Costmap not received yet, retrying %d/%d...', retry_count, max_retries)
                    rospy.sleep(retry_interval)
                    continue
                
                # Visualize river area
                visualize_area(Config.RIVER_BOUNDS['X_MIN'],
                               Config.RIVER_BOUNDS['Y_MIN'],
                                Config.RIVER_BOUNDS['X_MAX'],
                                Config.RIVER_BOUNDS['Y_MAX'],
                                self.river_area_publisher,
                                description='river_area')
                rospy.loginfo('Using costmap for box detection...')
                # Detect bridge positions from costmap
                bridge_positions, bridge_bounding_boxes = detect_object_from_costmap(self.costmap, eps=Config.CLUSTERING['EPS_BRIDGE'],
                                                                                min_samples=Config.CLUSTERING['MIN_SAMPLES_BRIDGE'],
                                                                                min_cluster_size=Config.CLUSTERING['MIN_POINTS_BRIDGE'],
                                                                                max_cluster_size=Config.CLUSTERING['MAX_POINTS_BRIDGE'],
                                                                                width_object=Config.CLUSTERING['width_bridge'],
                                                                                height_object=Config.CLUSTERING['height_bridge'],
                                                                                description='bridge',
                                                                                area_min_x_threshold=Config.RIVER_BOUNDS['X_MIN'],
                                                                                area_max_x_threshold=Config.RIVER_BOUNDS['X_MAX'],
                                                                                area_min_y_threshold=Config.RIVER_BOUNDS['Y_MIN'],
                                                                                area_max_y_threshold=Config.RIVER_BOUNDS['Y_MAX'],
                                                                                obstacle_threshold=Config.OBSTACLE_THRESHOLD)
                # If bridge positions are detected, publish them
                if bridge_positions:
                    if len(bridge_positions) == 1:
                        # Publish bridge visualization
                        self.detected_bridge_publisher.publish(bridge_bounding_boxes)
                        # Use the first detected bridge position
                        bridge_position = bridge_positions[0]

                    else:
                        rospy.logwarn('Multiple bridges detected, trying to average positions...')
                        # Average bridge positions
                        bridge_position = Pose()
                        bridge_position.position.x = sum(p.position.x for p in bridge_positions) / len(bridge_positions)
                        bridge_position.position.y = sum(p.position.y for p in bridge_positions) / len(bridge_positions)
                        bridge_position.position.z = 0.0
                    
                    # Create bridge entrance pose
                    bridge_entrance = Pose()
                    bridge_entrance.position.x = Config.BRIDGE_X
                    bridge_entrance.position.y = bridge_position.position.y
                    bridge_entrance.position.z = bridge_position.position.z
                    bridge_entrance.orientation.w = 0.0
                    bridge_entrance.orientation.z = 1.0

                    # Send bridge entrance pose to move_base
                    navigation_result = navigate_to_goal(self.client,
                                     bridge_entrance.position.x,
                                     bridge_entrance.position.y,
                                     bridge_entrance.position.z,
                                     bridge_entrance.orientation.w,
                                     bridge_entrance.orientation.z,
                                     Config.TIMEOUTS['NAVIGATION'],
                                     description='bridge_entrance',)
                    if navigation_result != 'Arrived':
                        rospy.logwarn('Failed to navigate to bridge entrance')
                        return 'failed'
                    return 'succeeded'
                else:
                    retry_count += 1
                    rospy.logwarn('No bridges detected from costmap, retrying %d/%d...', retry_count, max_retries)
                    # Wait for a while before retrying
                    rospy.sleep(retry_interval)
                    continue
                    
            except Exception as e:
                retry_count += 1
                rospy.logerr('Bridge detection error: %s, retrying %d/%d...', str(e), retry_count, max_retries)
                rospy.sleep(retry_interval)
        
        # If all retries fail, return failure
        rospy.logerr('Failed to detect bridge after %d attempts, giving up', max_retries)
        return 'failed'
              
    def costmap_callback(self, msg):
        """Process received costmap"""
        self.costmap = msg
        rospy.loginfo('Received costmap, width: %d, height: %d', msg.info.width, msg.info.height)

# Define state: Open bridge and navigate across
class OpenBridgeAndCross(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.open_bridge_publisher = rospy.Publisher(
            Config.TOPICS['OPEN_BRIDGE'], 
            Bool, 
            queue_size=10
        )
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def execute(self, userdata):        
        try:
            # Send command to open bridge multiple times
            open_bridge_msg = Bool()
            open_bridge_msg.data = True
            for _ in range(3):
                self.open_bridge_publisher.publish(open_bridge_msg)
                rospy.loginfo('Bridge open command sent')
                rospy.sleep(0.1)  # Wait for a second before sending the next command
            rospy.loginfo('Bridge open command sent')
            
            # Use cmd_vel to move forward for 5 seconds
            twist_msg = Twist()
            twist_msg.linear.x = Config.CROSSING_SPEED  # 设置线速度
            twist_msg.angular.z = 0.0  # 设置角速度为0，直线前进
            
            # 持续发布速度命令5秒
            start_time = rospy.Time.now()
            rate = rospy.Rate(10)  # 10Hz的频率
            
            while (rospy.Time.now() - start_time).to_sec() < Config.TIMEOUTS['BRIDGE_OPEN']:
                self.cmd_vel_publisher.publish(twist_msg)
                rate.sleep()
            
            # 停止机器人
            twist_msg.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist_msg)
            
            rospy.loginfo('Successfully crossed the bridge')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('Error crossing bridge: %s', str(e))
            return 'failed'

# Define state: Navigate to goal and start OCR
class NavigateToGoalAndOCR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        # Get target box coordinate list
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

        self.view_positions_publisher = rospy.Publisher(
            '/goal_view_positions', 
            MarkerArray, 
            queue_size=10
        )

        # Once target box is matched, OCR node will publish /cmd_stop
        self.cmd_stop_subscriber = rospy.Subscriber(
            Config.TOPICS['CMD_STOP'], 
            Bool, 
            self.cmd_stop_callback
        )
        self.cmd_stop_received = False  # Flag whether stop command received
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.ocr_timeout = Config.TIMEOUTS['OCR_PROCESSING']
        
    def execute(self, userdata):
        rospy.loginfo('Executing navigate to goal task...')
        try:
            # Loop through target coordinates, navigate one by one
            for i, goal_coords in enumerate(self.goals):
                # 将元组转换为 Pose 对象
                goal_position = Pose()
                # self.goals 中的元素是形如 (x, y) 的元组
                goal_position.position.x = goal_coords[0]
                goal_position.position.y = goal_coords[1]
                goal_position.position.z = 0.0
                
                # 设置默认方向（朝向正前方）
                goal_position.orientation.w = 1.0
                goal_position.orientation.x = 0.0
                goal_position.orientation.y = 0.0
                goal_position.orientation.z = 0.0
                
                navigate_to_best_viewing_positions_and_visualize_and_ocr(
                    Config.VIEWING_ANGLES, 
                    Config.VIEWING_DISTANCE,
                    goal_position,  # Pose 
                    Config.BOX_SIZE,
                    x_min=Config.GOAL_MAP_BOUNDS['X_MIN'], 
                    y_min=Config.GOAL_MAP_BOUNDS['Y_MIN'],
                    x_max=Config.GOAL_MAP_BOUNDS['X_MAX'], 
                    y_max=Config.GOAL_MAP_BOUNDS['Y_MAX'],
                    client=self.client,
                    navigation_timeout=self.navigation_timeout,
                    ocr_trigger_publisher=self.ocr_trigger_publisher, 
                    view_positions_publisher=self.view_positions_publisher)

                # 等待 OCR 处理完成的代码保持不变
                wait_start_time = rospy.Time.now()
                rate = rospy.Rate(2)  # 2Hz check frequency
                
                while (rospy.Time.now() - wait_start_time).to_sec() < self.ocr_timeout:
                    if self.cmd_stop_received:
                        rospy.loginfo('OCR task complete, stopping navigation')
                        self.cmd_stop_received = False
                        return 'succeeded'
                    rate.sleep()
                    
                rospy.loginfo('OCR processing timed out, continuing to next target')

            rospy.loginfo('All target positions navigation completed')
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('Error navigating to target: %s', str(e))
            return 'failed'   
    
    def cmd_stop_callback(self, msg):
        if msg.data:
            rospy.loginfo('Stop command received, OCR task complete')
            self.cmd_stop_received = True
        else:
            rospy.loginfo('No stop command received, continuing to wait')

# Main function
def main():
    # Initialize ROS node
    rospy.init_node('task_coordinator')

    # Create top-level state machine
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    
    # 为状态机添加共享的userdata字段
    sm.userdata.costmap = None

    # Add states to state machine
    with sm:
        # Add Initialize state - the entry point of the state machine
        smach.StateMachine.add('INITIALIZE',
                            Initialize(),
                            transitions={'initialized': 'DETECT_BOX_POSE',
                                            'failed': 'mission_failed'})
        
        # Add Frontier Exploration state - robot explores the environment to build a map
        # smach.StateMachine.add('EXPLORE_FRONTIER',
        #                     ExploreFrontier(),
        #                     transitions={'succeeded': 'DETECT_BOX_POSE',
        #                                     'failed': 'mission_failed'})
        
        # Add Box Detection state - detect boxes in the environment and try OCR
        smach.StateMachine.add('DETECT_BOX_POSE',
                            DetectBoxPoseNavigateAndOCR(),
                            transitions={'succeeded': 'DETECT_BRIDGE',
                                        'failed': 'mission_failed'},
                            remapping={'costmap_out': 'costmap'})
        
        # Add Bridge Detection state - find the bridge location
        smach.StateMachine.add('DETECT_BRIDGE',
                            DetectBridgeAndToEntrance(),
                            transitions={'succeeded': 'OPEN_BRIDGE_AND_CROSS',
                                        'failed': 'mission_failed'},
                            remapping={'costmap_in': 'costmap'})
        
        # Add Bridge Crossing state - open and cross the bridge
        smach.StateMachine.add('OPEN_BRIDGE_AND_CROSS',
                            OpenBridgeAndCross(),
                            transitions={'succeeded': 'NAVIGATE_TO_GOAL',
                                        'failed': 'mission_failed'})
        
        # Add Final Navigation state - navigate to goal position and perform OCR
        smach.StateMachine.add('NAVIGATE_TO_GOAL',
                            NavigateToGoalAndOCR(),
                            transitions={'succeeded': 'mission_completed',
                                        'failed': 'mission_failed'})
        
        sis = smach_ros.IntrospectionServer('task_coordinator', sm, '/TASK_COORDINATOR')
        sis.start()
        
        # Execute state machine
        outcome = sm.execute()
        
        rospy.loginfo('State machine execution complete, result: %s', outcome)
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    main()