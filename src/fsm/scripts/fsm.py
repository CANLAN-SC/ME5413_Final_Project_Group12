#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math
from std_msgs.msg import Int32, ColorRGBA

# Visualization
import matplotlib.pyplot as plt
import numpy as np

# ====================== Global Configuration Parameters ======================
class Config:
    # Topic name configuration
    TOPICS = {
        # Costmap related topics
        'GLOBAL_COSTMAP': '/move_base/global_costmap/costmap',
        'GLOBAL_COSTMAP_UPDATES': '/move_base/global_costmap/costmap_updates',
        'EXPLORE_COSTMAP': '/frontier_explore/costmap',
        'EXPLORE_COSTMAP_UPDATES': '/frontier_explore/costmap_updates',
        'FRONTIERS': '/explore/frontiers',
        
        # Detection trigger and result topics
        'BOX_DETECTION_TRIGGER': '/box_detection_trigger',
        'DETECTED_BOXES': '/detected_boxes',
        'BRIDGE_DETECTION_TRIGGER': '/bridge_detection_trigger',
        'DETECTED_BRIDGES': '/detected_bridges',
        'OCR_TRIGGER': '/ocr_trigger',
        'CMD_STOP': '/cmd_stop',
        'OPEN_BRIDGE': '/open_bridge',
        
        # Navigation related topics
        'MOVE_BASE': 'move_base',

        # Box extraction related topics
        # 'BOX_EXTRACTION': '/move_base/global_costmap/costmap/obstacles_layer',
        'BOX_EXTRACTION': '/move_base/global_costmap/costmap',
        # OCR related topics
        'RECOGNIZED_DIGIT': '/recognized_digit',
    }
    
    # Timeout settings (seconds)
    TIMEOUTS = {
        'INIT': 60.0,                # Initialization timeout
        'EXPLORE': 30.0,            # Exploration task timeout
        'BOX_DETECTION': 3.0,       # Box detection timeout
        'BRIDGE_DETECTION': 15.0,    # Bridge detection timeout
        'NAVIGATION': 60.0,          # Navigation timeout
        'OCR_PROCESSING': 5.0,       # OCR processing timeout
        'BRIDGE_OPEN': 3.0,          # Bridge opening wait time
    }

    # Obstacle threshold
    OBSTACLE_THRESHOLD = 99  # Initial dynamically adjustable threshold

    # Exploration area settings
    EXPLORE_MAP_BOUNDS = {
        'X_MIN': 9.0,    # Map X coordinate minimum value
        'X_MAX': 19.0,   # Map X coordinate maximum value
        'Y_MIN': -22.0,    # Map Y coordinate minimum value
        'Y_MAX': -2.0,   # Map Y coordinate maximum value
    }

    # Exploration related settings
    EXPLORE = {
        'FRONTIER_THRESHOLD': 0,     # Frontier point count threshold, below this value exploration is considered complete
        'CHECK_RATE': 0.5,           # Check frequency (Hz)
    }
    
    # Bridge related settings
    BRIDGE = {
        'EXIT_Y_OFFSET': -3.0,       # Bridge exit Y-axis offset relative to entrance
    }
    
    # Target position coordinate list (x, y, orientation.w)
    GOALS = [
        (6.0, 0.0, 1.0),
        (10.0, 0.0, 1.0),
        (14.0, 0.0, 1.0),
        (18.0, 0.0, 1.0)
    ]

    # Best viewing distance
    VIEWING_DISTANCE = 2.0

    # Viewing angles
    VIEWING_ANGLES = [0, math.pi/2, math.pi, 3*math.pi/2]  # 0°, 90°, 180°, 270°

    # Box side length
    BOX_SIZE = 0.5  # meters

    # Clustering parameters
    CLUSTERING = {
        'width': 35,
        'height': 35,
        'MIN_POINTS': 15,  # Minimum point count
        'MAX_POINTS': 200,  # Maximum point count
        'MIN_SAMPLES': 5,  # Minimum sample size
        'EPS': 5,  # DBSCAN eps parameter, smaller eps creates more, smaller clusters
    }

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

# Define state: Box Position Detection Task
class DetectBoxPose(smach.State):
    def __init__(self):      
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], 
                             output_keys=['box_positions_out'])

        # Add box position publisher
        self.box_publisher = rospy.Publisher(
            Config.TOPICS['DETECTED_BOXES'], 
            PoseArray, 
            queue_size=10
        )
        # Add box generation area publisher
        self.box_area_publisher = rospy.Publisher(
            '/box_area', 
            MarkerArray, 
            queue_size=10
        )
        # Add global costmap subscription
        # self.costmap_subscriber = rospy.Subscriber(
        #     Config.TOPICS['BOX_EXTRACTION'], 
        #     OccupancyGrid, 
        #     self.costmap_callback,
        #     queue_size=50
        # )
        self.box_positions = []  # For storing detected box positions
        self.detection_timeout = Config.TIMEOUTS['BOX_DETECTION']
        self.costmap = None  # Store latest costmap
               
    def execute(self, userdata):
        self.costmap_subscriber = rospy.Subscriber(
            Config.TOPICS['BOX_EXTRACTION'], 
            OccupancyGrid, 
            self.costmap_callback,
            queue_size=50
        )
        # Reset box position list
        self.box_positions = []

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
                
                # Use costmap to detect boxes
                rospy.loginfo('Using costmap for box detection...')
                costmap_boxes = self.detect_boxes_from_costmap()

                # If boxes are detected, save to userdata
                if costmap_boxes:
                    self.box_positions = costmap_boxes
                    if len(costmap_boxes) < 5:
                        continue
                    # Create PoseArray from box poses for easier passing
                    pose_array = PoseArray()
                    pose_array.header.frame_id = "map"
                    pose_array.header.stamp = rospy.Time.now()
                    pose_array.poses = [box.pose for box in costmap_boxes]
                    userdata.box_positions_out = pose_array
                    rospy.loginfo('Successfully detected %d boxes from costmap', len(costmap_boxes))
                    
                    # Publish detected box positions
                    self.box_publisher.publish(pose_array)
                    return 'succeeded'
                # If no boxes detected, try again
                else:
                    retry_count += 1
                    rospy.logwarn('No boxes detected from costmap, retrying %d/%d...', retry_count, max_retries)
                    # Wait a while before retrying
                    rospy.sleep(retry_interval)
            
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

    def detect_boxes_from_costmap(self):
        """Extract box positions from costmap"""
        if self.costmap is None:
            rospy.logwarn("Cannot detect boxes from costmap: costmap not received")
            return []
            
        # Convert costmap to numpy array for processing
        rospy.loginfo('Processing costmap to detect boxes...')
        width = self.costmap.info.width
        height = self.costmap.info.height
        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        rospy.loginfo('Costmap resolution: %.2f m/pixel', resolution)
        rospy.loginfo('Costmap origin: (%.2f, %.2f)', origin_x, origin_y)
        
        # Convert 1D array to 2D grid
        grid = np.array(self.costmap.data).reshape((height, width))

        # Save costmap image
        timestamp = rospy.Time.now().to_sec()
        filename = f"./map_for/costmap_{timestamp:.0f}.png"
        
        plt.figure(figsize=(10, 10))
        plt.imshow(grid, cmap='hot', origin='lower')
        plt.colorbar(label='cost')
        plt.title(f'costmap ({width}x{height}, resolution: {resolution:.2f}m/pixel)')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        plt.savefig(filename)
        plt.close()
        rospy.loginfo(f'Costmap for clustering saved to: {filename}')
        
        rospy.loginfo('Processing costmap to detect boxes...')
        
        # Detect high occupancy value areas (obstacles/boxes)
        obstacle_threshold = Config.OBSTACLE_THRESHOLD
        obstacles = np.where(grid > obstacle_threshold)
        
        # Clustering - using simple distance-based clustering
        from sklearn.cluster import DBSCAN
        
        # If no obstacle points, return empty list
        if len(obstacles[0]) == 0:
            rospy.logwarn("No obstacle points detected")
            return []
        else:
            rospy.loginfo("Detected %d obstacle points", len(obstacles[0]))
            
        # Convert obstacle points to coordinate list
        points = np.column_stack([obstacles[1], obstacles[0]])  # x corresponds to columns, y to rows
        
        # Use DBSCAN for clustering
        clustering = DBSCAN(eps=Config.CLUSTERING['EPS'], min_samples=Config.CLUSTERING['MIN_SAMPLES']).fit(points)
        labels = clustering.labels_
        
        # Calculate center point for each cluster
        box_positions = []
        unique_labels = set(labels)

        for label in unique_labels:
            # Skip noise points (label -1)
            if label == -1:
                continue
                
            # Get all points in current cluster
            cluster_points = points[labels == label]
            
            # Calculate cluster size
            cluster_size = len(cluster_points)
            
            # Calculate cluster bounding box
            min_x, min_y = np.min(cluster_points, axis=0)
            max_x, max_y = np.max(cluster_points, axis=0)
            width = max_x - min_x
            height = max_y - min_y
            
            # Filter conditions: boxes should be small compact clusters
            # 1. Cluster point count should not be too many (adjust based on actual box size)
            # 2. Shape should be close to square
            # 3. Size should be within reasonable range
            if (Config.CLUSTERING['MIN_POINTS'] <= cluster_size <= Config.CLUSTERING['MAX_POINTS'] and  # Point count range
                width < Config.CLUSTERING['width'] and height < Config.CLUSTERING['height']):  # Size limits (in grid cells)
                
                # Calculate cluster center
                center_x = np.mean(cluster_points[:, 0])
                center_y = np.mean(cluster_points[:, 1])
                
                # Convert center point to map coordinates
                map_x = center_x * resolution + origin_x
                map_y = center_y * resolution + origin_y
            
                # Create pose message
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = map_x
                pose.pose.position.y = map_y
                pose.pose.position.z = 0.0
                # Default orientation, facing forward
                pose.pose.orientation.w = 1.0
                
                # Only detect boxes in exploration area
                rospy.loginfo('Box position: x=%.2f, y=%.2f', map_x, map_y)
            
                # Publish exploration area
                self.publish_explore_area()

                # Check if box is within exploration area
                rospy.loginfo('Checking if box is within exploration area...')
                if (Config.EXPLORE_MAP_BOUNDS['X_MIN'] <= map_x <= Config.EXPLORE_MAP_BOUNDS['X_MAX'] and
                    Config.EXPLORE_MAP_BOUNDS['Y_MIN'] <= map_y <= Config.EXPLORE_MAP_BOUNDS['Y_MAX']):
                    box_positions.append(pose)
                    rospy.loginfo(f"Detected box from costmap: x={map_x:.2f}, y={map_y:.2f}")
                else:
                    rospy.loginfo(
                        f"Box position x={map_x:.2f}, y={map_y:.2f} is outside exploration area range {Config.EXPLORE_MAP_BOUNDS['X_MIN']:.2f}~{Config.EXPLORE_MAP_BOUNDS['X_MAX']:.2f}, {Config.EXPLORE_MAP_BOUNDS['Y_MIN']:.2f}~{Config.EXPLORE_MAP_BOUNDS['Y_MAX']:.2f}")
        return box_positions

    def publish_explore_area(self):
        """Publish visualization markers for exploration area"""
        from visualization_msgs.msg import Marker, MarkerArray
        from std_msgs.msg import ColorRGBA
        from geometry_msgs.msg import Point
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Create area boundary marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "explore_area"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Line width
        
        # Set marker color to green, alpha 0
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.0)  # R, G, B, A
        
        # Add four corner points of exploration area, forming a closed loop
        points = []
        # Bottom left
        p1 = Point()
        p1.x = Config.EXPLORE_MAP_BOUNDS['X_MIN']
        p1.y = Config.EXPLORE_MAP_BOUNDS['Y_MIN']
        p1.z = 0.1  # Slightly above ground
        points.append(p1)
        
        # Bottom right
        p2 = Point()
        p2.x = Config.EXPLORE_MAP_BOUNDS['X_MAX']
        p2.y = Config.EXPLORE_MAP_BOUNDS['Y_MIN']
        p2.z = 0.1
        points.append(p2)
        
        # Top right
        p3 = Point()
        p3.x = Config.EXPLORE_MAP_BOUNDS['X_MAX']
        p3.y = Config.EXPLORE_MAP_BOUNDS['Y_MAX']
        p3.z = 0.1
        points.append(p3)
        
        # Top left
        p4 = Point()
        p4.x = Config.EXPLORE_MAP_BOUNDS['X_MIN']
        p4.y = Config.EXPLORE_MAP_BOUNDS['Y_MAX']
        p4.z = 0.1
        points.append(p4)
        
        # Back to start point to close the loop
        points.append(p1)
        
        marker.points = points
        marker.lifetime = rospy.Duration(5.0)  # Marker displayed for 5 seconds
        
        # Create filled area marker
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
        
        # Set fill color to light green, semi-transparent
        fill_marker.color = ColorRGBA(0.0, 0.8, 0.2, 0.3)  # R, G, B, A
        
        # Create two triangles to fill rectangular area
        fill_points = []
        # Triangle 1: Bottom left - Bottom right - Top left
        fill_points.extend([p1, p2, p4])
        # Triangle 2: Bottom right - Top right - Top left
        fill_points.extend([p2, p3, p4])
        
        fill_marker.points = fill_points
        fill_marker.lifetime = rospy.Duration(5.0)
        
        # Add markers to array
        marker_array.markers.append(marker)
        marker_array.markers.append(fill_marker)
        
        # Publish marker array
        self.box_area_publisher.publish(marker_array)
        rospy.loginfo("Exploration area visualization published")

# Define state: Navigate to each box and start OCR
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
        rospy.loginfo('Navigation client connected')
        self.navigation_timeout = Config.TIMEOUTS['NAVIGATION']
        self.ocr_timeout = Config.TIMEOUTS['OCR_PROCESSING']
        self.ocr_result = None  # For storing OCR result
  
    def execute(self, userdata):
        # Get box positions from userdata
        self.box_positions = userdata.box_positions_in

        wait_timeout = rospy.Duration(10.0)  # Set reasonable timeout
        start_time = rospy.Time.now()
        
        while not self.box_positions and (rospy.Time.now() - start_time) < wait_timeout:
            rospy.loginfo("Waiting to receive box position data...")
            rospy.sleep(0.5)  # Short sleep to avoid high CPU usage
        
        if not self.box_positions:
            rospy.logwarn('No box positions available for navigation')
            return 'failed'

        rospy.loginfo('Navigating to %d boxes and starting OCR...', len(self.box_positions.poses))
        
        # Create PoseArray to store extracted poses
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()

        # Navigate and perform OCR for each box
        for i, pose in enumerate(self.box_positions.poses):
            try:
                self.navigate_to_best_viewing_positions(pose)
                
            except Exception as e:
                rospy.logerr('Error processing box: %s', str(e))
                
        return 'succeeded'

    def box_positions_callback(self, msg):
        """Process box position subscription"""
        # rospy.loginfo('Received box position message, count: %d', len(msg.markers))
        # Save positions
        self.box_positions = msg

    def navigate_to_best_viewing_positions(self, box_pose):
        """Calculate and navigate to best viewing positions around box"""     
        # Calculate viewing positions
        viewing_positions = self.calculate_box_viewing_positions(box_pose)
        
        # Publish visualization markers
        self.visualize_box_viewing_positions(box_pose, viewing_positions)
               
        for i, viewing_position in enumerate(viewing_positions):

            # Check if position is near the cliff
            if viewing_position['position']['x'] < Config.EXPLORE_MAP_BOUNDS['X_MIN'] + 1:
                rospy.logwarn('Viewing position is too close to the cliff, skipping...')
                continue

            # Create pose facing the box
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = viewing_position['position']['x']
            goal.target_pose.pose.position.y = viewing_position['position']['y']
            goal.target_pose.pose.position.z = 0.0
            
            # Convert angle to quaternion
            goal.target_pose.pose.orientation.z = viewing_position['orientation']['z']
            goal.target_pose.pose.orientation.w = viewing_position['orientation']['w']
            
            rospy.loginfo('Navigating to box viewing position [%d/4]: x=%.2f, y=%.2f, angle=%.2f°', 
                        i+1, viewing_position['position']['x'],
                        viewing_position['position']['y'],
                        math.degrees(viewing_position['angle']))
                        
            self.client.send_goal(goal)
            
            # Wait for navigation result, with timeout
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('Navigation to viewing position timed out, trying next position')
                continue
            
            if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('Navigation to viewing position failed, trying next position')
                continue
            
            # After reaching viewing position, trigger OCR
            ocr_trigger_msg = Bool()
            ocr_trigger_msg.data = True
            self.ocr_trigger_publisher.publish(ocr_trigger_msg)
            rospy.loginfo('Triggering OCR at viewing position...')
            rospy.sleep(1.0)  # Give OCR time to process

            # If OCR processing completes, topic /recognized_digit(Int32) will publish result
            # If recognition succeeds, break loop
            if self.ocr_result is not None:  # Modified condition to check for any result
                rospy.loginfo('OCR processing successful, recognized digit: %d', self.ocr_result)
                self.ocr_result = None  # Reset result for next recognition
                break
            else:
                rospy.logwarn('OCR processing failed, trying next viewing position')
                continue

    def calculate_box_viewing_positions(self, box_pose):
        """
        Calculate best viewing positions around a box
        
        Parameters:
            box_pose: Box pose, containing position and orientation
            
        Returns:
            viewing_positions: List of viewing positions, each a dictionary containing:
                - position: Viewing position coordinates (x, y, z)
                - orientation: Viewing position orientation (quaternion)
                - angle: Viewing angle (radians)
        """
        # Define 4 best viewing positions (around the box)
        viewing_angles = Config.VIEWING_ANGLES  # 0°, 90°, 180°, 270°
        viewing_distance = Config.VIEWING_DISTANCE  # Best viewing distance
        
        viewing_positions = []
        
        for i, angle in enumerate(viewing_angles):
            # Calculate viewing position
            view_x = box_pose.position.x + viewing_distance * math.cos(angle)
            view_y = box_pose.position.y + viewing_distance * math.sin(angle)
            
            # Calculate angle facing the box
            dx = box_pose.position.x - view_x
            dy = box_pose.position.y - view_y
            facing_angle = math.atan2(dy, dx)  # Correct parameter order for atan2 is (y, x)
            
            # Create quaternion for orientation
            orientation = {
                'z': math.sin(facing_angle / 2.0),
                'w': math.cos(facing_angle / 2.0)
            }
            
            # Save viewing position information
            position = {
                'x': view_x,
                'y': view_y,
                'z': 0.0
            }
            
            # Add to results list
            viewing_positions.append({
                'position': position,
                'orientation': orientation,
                'angle': facing_angle
            })
        
        return viewing_positions
    
    def visualize_box_viewing_positions(self, box_pose, viewing_positions):
        """Visualize box position best viewing position markers"""
        # Create marker array
        marker_array = MarkerArray()

        # Create box marker
        box_marker = Marker()
        box_marker.header.frame_id = "map"
        box_marker.header.stamp = rospy.Time.now()
        box_marker.ns = "box_visualization"
        box_marker.id = 0
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        
        # Set box position and size
        box_marker.pose = box_pose
        box_marker.scale.x = Config.BOX_SIZE
        box_marker.scale.y = Config.BOX_SIZE
        box_marker.scale.z = Config.BOX_SIZE
        
        # Set box color to red
        box_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.0)  # R, G, B, A
        box_marker.lifetime = rospy.Duration(10.0)  # Display for 10 seconds
        
        marker_array.markers.append(box_marker)
        
        # Create viewing position markers
        for i, viewing_position in enumerate(viewing_positions):            
            # Create viewing position marker
            view_marker = Marker()
            view_marker.header.frame_id = "map"
            view_marker.header.stamp = rospy.Time.now()
            view_marker.ns = "view_position"
            view_marker.id = i + 1  # Start from 1, 0 is the box
            view_marker.type = Marker.ARROW  # Use arrow to represent robot orientation
            view_marker.action = Marker.ADD
            
            # Set viewing position
            view_marker.pose.position.x = viewing_position['position']['x']
            view_marker.pose.position.y = viewing_position['position']['y']
            view_marker.pose.position.z = 0.1  # Slightly above ground
            
            # Set orientation (quaternion)
            view_marker.pose.orientation.z = viewing_position['orientation']['z']
            view_marker.pose.orientation.w = viewing_position['orientation']['w']
            
            # Set arrow size
            view_marker.scale.x = 0.3  # Arrow length
            view_marker.scale.y = 0.1  # Arrow width
            view_marker.scale.z = 0.1  # Arrow height
            
            # Set color, use different colors to distinguish four positions
            colors = [
                ColorRGBA(0.0, 0.8, 0.0, 0.8),  # Green - 0°
                ColorRGBA(0.0, 0.0, 0.8, 0.8),  # Blue - 90°
                ColorRGBA(0.8, 0.8, 0.0, 0.8),  # Yellow - 180°
                ColorRGBA(0.8, 0.0, 0.8, 0.8)   # Purple - 270°
            ]
            view_marker.color = colors[i]
            view_marker.lifetime = rospy.Duration(10.0)  # Display for 10 seconds
            
            marker_array.markers.append(view_marker)
            
            # Add connecting line, from viewing position pointing to box
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "view_lines"
            line_marker.id = i + 5  # Start from 5, avoid ID conflict
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Add line start and end points
            start_point = Point()
            start_point.x = viewing_position['position']['x']
            start_point.y = viewing_position['position']['y']
            start_point.z = 0.1
            
            end_point = Point()
            end_point.x = box_pose.position.x
            end_point.y = box_pose.position.y
            end_point.z = 0.1
            
            line_marker.points = [start_point, end_point]
            
            # Set line thickness and color
            line_marker.scale.x = 0.03  # Line width
            line_marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.5)  # Semi-transparent gray
            line_marker.lifetime = rospy.Duration(10.0)  # Display for 10 seconds
            
            marker_array.markers.append(line_marker)
        
        # Create text markers, showing viewing position numbers
        for i, viewing_position in enumerate(viewing_positions):
            # Create text marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "view_text"
            text_marker.id = i + 9  # Avoid ID conflict
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set text position (slightly higher)
            text_marker.pose.position.x = viewing_position['position']['x']
            text_marker.pose.position.y = viewing_position['position']['y']
            text_marker.pose.position.z = 0.3  # Text height
            
            # Set text content and appearance
            text_marker.text = f"Position {i+1}"
            text_marker.scale.z = 0.2  # Text size
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 0.8)  # White text
            text_marker.lifetime = rospy.Duration(10.0)  # Display for 10 seconds
            
            marker_array.markers.append(text_marker)
        
        # Publish marker array    
        self.view_positions_publisher.publish(marker_array)
        rospy.loginfo("Box and its viewing positions visualization published")

    def ocr_result_callback(self, msg):
        self.ocr_result = msg.data

# Define state: Bridge detection task  
class DetectBridge(smach.State):
    def __init__(self):
        # Add output keys bridge_entrance and bridge_exit
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
        self.bridge_entrance = None  # Bridge entrance coordinates
        self.detection_complete = False  # Detection complete flag
        self.detection_timeout = Config.TIMEOUTS['BRIDGE_DETECTION']
        
    def execute(self, userdata):
        # Reset detection status
        self.bridge_entrance = None
        self.detection_complete = False
        
        rospy.loginfo('Executing bridge detection task...')
        try:
            # Publish bridge detection trigger message
            bridge_detection_trigger_msg = Bool()
            bridge_detection_trigger_msg.data = True
            self.bridge_detection_trigger_publisher.publish(bridge_detection_trigger_msg)
            rospy.loginfo('Bridge detection trigger message published')
            
            # Wait for bridge detection result
            start_time = rospy.Time.now()
            
            rate = rospy.Rate(2)  # 2Hz check frequency
            while not self.detection_complete and (rospy.Time.now() - start_time).to_sec() < self.detection_timeout:
                rate.sleep()
                
            if self.bridge_entrance is None:
                rospy.logwarn('Bridge not detected or detection timed out')
                return 'failed'
            
            # Calculate exit coordinates (entrance coordinates y value plus offset)
            bridge_exit = PoseStamped()
            bridge_exit.header = self.bridge_entrance.header
            bridge_exit.pose.position.x = self.bridge_entrance.pose.position.x
            bridge_exit.pose.position.y = self.bridge_entrance.pose.position.y + Config.BRIDGE['EXIT_Y_OFFSET']
            bridge_exit.pose.orientation = self.bridge_entrance.pose.orientation
            
            # Save entrance and exit coordinates to userdata to pass to next state
            userdata.bridge_entrance_out = self.bridge_entrance
            userdata.bridge_exit_out = bridge_exit
            
            rospy.loginfo('Bridge detection successful')
            rospy.loginfo('Bridge entrance coordinates: x=%.2f, y=%.2f', 
                          self.bridge_entrance.pose.position.x, 
                          self.bridge_entrance.pose.position.y)
            rospy.loginfo('Bridge exit coordinates: x=%.2f, y=%.2f', 
                          bridge_exit.pose.position.x, 
                          bridge_exit.pose.position.y)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('Bridge detection error: %s', str(e))
            return 'failed'
    
    def bridge_callback(self, msg):
        rospy.loginfo('Bridge detected, position: x=%.2f, y=%.2f', msg.pose.position.x, msg.pose.position.y)
        self.bridge_entrance = msg
        self.detection_complete = True

# Define state: Navigate to bridge entrance
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
        # Get bridge entrance coordinates from userdata
        bridge_entrance = userdata.bridge_entrance_in
        
        if bridge_entrance is None:
            rospy.logwarn('Missing bridge entrance coordinate information')
            return 'failed'
        
        rospy.loginfo('Starting navigation to bridge entrance...')
        
        try:
            # Navigate to bridge entrance
            rospy.loginfo('Navigating to bridge entrance: x=%.2f, y=%.2f', 
                         bridge_entrance.pose.position.x, 
                         bridge_entrance.pose.position.y)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = bridge_entrance.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = bridge_entrance.pose
            
            self.client.send_goal(goal)
            
            # Add timeout
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('Navigation to bridge entrance timed out')
                return 'failed'
                
            result = self.client.get_state()
            
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('Navigation to bridge entrance failed')
                return 'failed'
            
            rospy.loginfo('Successfully reached bridge entrance')
            # Pass coordinates to next state
            userdata.bridge_entrance_out = bridge_entrance
            userdata.bridge_exit_out = userdata.bridge_exit_in
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('Error navigating to bridge entrance: %s', str(e))
            return 'failed'

# Define state: Open bridge and navigate across
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
        # Get bridge exit coordinates from userdata
        bridge_exit = userdata.bridge_exit_in
        
        if bridge_exit is None:
            rospy.logwarn('Missing bridge exit coordinate information')
            return 'failed'
        
        try:
            # Send open bridge command
            open_bridge_msg = Bool()
            open_bridge_msg.data = True
            self.open_bridge_publisher.publish(open_bridge_msg)
            rospy.loginfo('Open bridge command sent')
            
            # Give bridge enough time to open
            rospy.sleep(self.bridge_open_wait_time)
            
            # Navigate to bridge exit (cross bridge)
            rospy.loginfo('Starting to cross bridge, navigating to bridge exit: x=%.2f, y=%.2f', 
                         bridge_exit.pose.position.x, 
                         bridge_exit.pose.position.y)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = bridge_exit.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = bridge_exit.pose
            
            self.client.send_goal(goal)
            
            # Add timeout
            if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                rospy.logwarn('Bridge crossing navigation timed out')
                return 'failed'
                
            result = self.client.get_state()
            
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logwarn('Bridge crossing failed')
                return 'failed'
            
            rospy.loginfo('Successfully crossed bridge')
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
            for i, goal_position in enumerate(self.goals):
                self.goal.target_pose.pose.position.x = goal_position[0]
                self.goal.target_pose.pose.position.y = goal_position[1]
                self.goal.target_pose.pose.orientation.w = goal_position[2]
                
                rospy.loginfo('[%d/%d] Navigating to target position: x=%.2f, y=%.2f', 
                              i+1, len(self.goals),
                              goal_position[0], goal_position[1])
                              
                self.client.send_goal(self.goal)
                
                # Add timeout
                if not self.client.wait_for_result(rospy.Duration(self.navigation_timeout)):
                    rospy.logwarn('Navigation to target position timed out, trying next target')
                    continue
                    
                result = self.client.get_state()
                
                if result == actionlib.GoalStatus.SUCCEEDED:
                    # Start OCR processing
                    ocr_trigger_msg = Bool()
                    ocr_trigger_msg.data = True
                    self.ocr_trigger_publisher.publish(ocr_trigger_msg)
                    rospy.loginfo('OCR trigger message published, waiting for processing...')
                    
                    # Wait for OCR processing to complete
                    wait_start_time = rospy.Time.now()
                    rate = rospy.Rate(2)  # 2Hz check frequency
                    
                    while (rospy.Time.now() - wait_start_time).to_sec() < self.ocr_timeout:
                        if self.cmd_stop_received:
                            rospy.loginfo('OCR task complete, stopping navigation')
                            self.cmd_stop_received = False
                            return 'succeeded'
                        rate.sleep()
                        
                    rospy.loginfo('OCR processing timed out, continuing to next target')
                else:
                    rospy.logwarn('Navigation to target position failed, trying next target')

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
    
    # Add states to state machine
    with sm:
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'initialized':'EXPLORE_FRONTIER', # Temporarily skip exploration
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
    
    # Create state machine visualization tool
    sis = smach_ros.IntrospectionServer('task_coordinator', sm, '/TASK_COORDINATOR')
    sis.start()
    
    # Execute state machine
    outcome = sm.execute()
    
    rospy.loginfo('State machine execution complete, result: %s', outcome)
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()