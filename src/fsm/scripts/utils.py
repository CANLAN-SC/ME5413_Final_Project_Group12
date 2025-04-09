import os
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import PoseStamped, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseGoal
import actionlib
# =============================== Utility Functions =========================
def detect_object_from_costmap(costmap, eps, min_samples, 
                               min_cluster_size, max_cluster_size, 
                               width_object, height_object, description,
                               area_min_x_threshold, area_max_x_threshold,
                               area_min_y_threshold, area_max_y_threshold, obstacle_threshold=99):
    """
    Detect objects from costmap using clustering algorithm
    Parameters:
        costmap: Costmap message
        eps: DBSCAN eps parameter
        min_samples: DBSCAN min_samples parameter
        min_cluster_size: Minimum cluster size
        max_cluster_size: Maximum cluster size
        width_object: Width of the object in meters
        height_object: Height of the object in meters
        description: Description of the object
        area_min_x_threshold: Minimum x coordinate of the area
        area_max_x_threshold: Maximum x coordinate of the area
        area_min_y_threshold: Minimum y coordinate of the area
        area_max_y_threshold: Maximum y coordinate of the area
        obstacle_threshold: Costmap threshold for detecting obstacles
    Returns:
        positions: List of Pose messages for detected object positions\n
        bounding_boxes: List of Marker messages for detected object bounding boxes
    """
    # Check if costmap is None
    if costmap is None:
        rospy.logwarn("Cannot detect boxes from costmap: costmap not received")
        return []
        
    # Convert costmap to numpy array for processing
    rospy.loginfo(f'Processing costmap to detect {description}...')
    width = costmap.info.width
    height = costmap.info.height
    resolution = costmap.info.resolution
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y
    rospy.loginfo('Costmap resolution: %.2f m/pixel', resolution)
    rospy.loginfo('Costmap origin: (%.2f, %.2f)', origin_x, origin_y)
    
    # Convert 1D array to 2D grid
    grid = np.array(costmap.data).reshape((height, width))

    # Save costmap image
    timestamp = rospy.Time.now().to_sec()
    costmap_path = './costmap/'
    if not os.path.exists(costmap_path):
        os.makedirs(costmap_path)
    filename = f"{costmap_path}/costmap_for_{description}_{timestamp:.0f}.png"
    
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='hot', origin='lower')
    plt.colorbar(label='cost')
    plt.title(f'costmap for {description} ({width}x{height}, resolution: {resolution:.2f}m/pixel)')
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.savefig(filename)
    plt.close()
    rospy.loginfo(f'Costmap for clustering saved to: {filename}, starting clustering...')
        
    # Detect high occupancy value areas (obstacles)
    obstacles = np.where(grid > obstacle_threshold)
    
    # Clustering - using simple distance-based clustering
    
    # If no obstacle points, return empty list
    if len(obstacles[0]) == 0:
        rospy.logwarn("No obstacle points detected")
        return []
    else:
        rospy.loginfo("Detected %d obstacle points", len(obstacles[0]))
        
    # Convert obstacle points to coordinate list
    points = np.column_stack([obstacles[1], obstacles[0]])  # x corresponds to columns, y to rows
    
    # Use DBSCAN for clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)    
    labels = clustering.labels_
    
    # Calculate center point for each cluster
    positions = []
    bounding_boxes = []
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
        cluster_min_x, cluster_min_y = np.min(cluster_points, axis=0)
        cluster_max_x, cluster_max_y = np.max(cluster_points, axis=0)
        min_width_threshold = cluster_max_x - cluster_min_x
        min_height_threshold = cluster_max_y - cluster_min_y
        
        # Filter conditions: boxes should be small compact clusters
        # 1. Cluster point count should not be too many (adjust based on actual box size)
        # 2. Shape should be close to square
        # 3. Size should be within reasonable range
        if (min_cluster_size <= cluster_size <= max_cluster_size and  # Point count range
            min_width_threshold < width_object and min_height_threshold < height_object):  # Size limits (in grid cells)
            
            # Calculate cluster center
            center_x = (cluster_min_x + cluster_max_x) / 2
            center_y = (cluster_min_y + cluster_max_y) / 2
            
            # Convert center point to map coordinates
            center_x_in_map_coordinate = center_x * resolution + origin_x
            center_y_in_map_coordinate = center_y * resolution + origin_y
            width_in_map = width * resolution
            height_in_map = height * resolution
        
            # Create pose message
            pose = Pose()
            pose.position.x = center_x_in_map_coordinate
            pose.position.y = center_y_in_map_coordinate
            pose.position.z = 0.0
            # Default orientation, facing forward
            pose.orientation.w = 1.0

            # Create marker message
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "bounding_boxes"
            marker.id = int(label)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Line width
            # Set marker color to red, alpha 0
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # R, G, B, A

            # Add four corner points of exploration area, forming a closed loop
            corner_points = []
            # Bottom left
            p1 = Point()
            p1.x = center_x_in_map_coordinate - width_in_map/2
            p1.y = center_y_in_map_coordinate - height_in_map/2
            p1.z = 0.1  # Slightly above ground
            corner_points.append(p1)
    
            # Bottom right
            p2 = Point()
            p2.x = center_x_in_map_coordinate + width_in_map/2
            p2.y = center_y_in_map_coordinate - height_in_map/2
            p2.z = 0.1
            corner_points.append(p2)
    
            # Top right
            p3 = Point()
            p3.x = center_x_in_map_coordinate + width_in_map/2
            p3.y = center_y_in_map_coordinate + height_in_map/2
            p3.z = 0.1
            corner_points.append(p3)
    
            # Top left
            p4 = Point()
            p4.x = center_x_in_map_coordinate - width_in_map/2
            p4.y = center_y_in_map_coordinate + height_in_map/2
            p4.z = 0.1
            corner_points.append(p4)
    
            # Back to start point to close the loop
            corner_points.append(p1)
            marker.points = corner_points
            
            # Only detect boxes in exploration area
            rospy.loginfo('Box position: x=%.2f, y=%.2f', center_x_in_map_coordinate, center_y_in_map_coordinate)

            # Check if box is within area
            rospy.loginfo(f'Checking if {description} is within the area...')
            if (area_min_x_threshold <= center_x_in_map_coordinate <= area_max_x_threshold and
                area_min_y_threshold <= center_y_in_map_coordinate <= area_max_y_threshold):
                positions.append(pose)
                bounding_boxes.append(marker)
                rospy.loginfo(f"Detected box from costmap: x={center_x_in_map_coordinate:.2f}, y={center_y_in_map_coordinate:.2f}")
            else:
                rospy.loginfo(
                    f"Object position x={center_x_in_map_coordinate:.2f}, y={center_y_in_map_coordinate:.2f} is outside exploration area range {area_min_x_threshold:.2f}~{area_max_x_threshold:.2f}, {area_min_y_threshold:.2f}~{area_max_y_threshold:.2f}")
    return positions, bounding_boxes

def visualize_area(x_min, y_min, x_max, y_max, area_publisher, description):
    """Publish visualization markers for river area
    Parameters:
        x_min: Minimum x coordinate of the area
        y_min: Minimum y coordinate of the area
        x_max: Maximum x coordinate of the area
        y_max: Maximum y coordinate of the area
        area_publisher: Publisher for visualization markers
        description: Description of the area
    Returns:
        None
    """
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
    marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # R, G, B, A
    
    # Add four corner points of exploration area, forming a closed loop
    points = []
    # Bottom left
    p1 = Point()
    p1.x = x_min
    p1.y = y_min
    p1.z = 0.05  # Slightly above ground
    points.append(p1)
    
    # Bottom right
    p2 = Point()
    p2.x = x_max
    p2.y = y_min
    p2.z = 0.05
    points.append(p2)
    
    # Top right
    p3 = Point()
    p3.x = x_max
    p3.y = y_max
    p3.z = 0.05
    points.append(p3)
    
    # Top left
    p4 = Point()
    p4.x = x_min
    p4.y = y_max
    p4.z = 0.05
    points.append(p4)
    
    # Back to start point to close the loop
    points.append(p1)
    
    marker.points = points
    marker.lifetime = rospy.Duration(5.0)  # Marker displayed for 5 seconds
    
    # Create filled area marker
    fill_marker = Marker()
    fill_marker.header.frame_id = "map"
    fill_marker.header.stamp = rospy.Time.now()
    fill_marker.ns = "river_fill"
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
    area_publisher.publish(marker_array)
    rospy.loginfo(f"{description} area visualization published")

def calculate_box_viewing_positions(viewing_angles, viewing_distance, box_pose):
    """
    Calculate the best viewing positions around a box
    
    Parameters:
        box_pose: Box pose, containing position and orientation
        
    Returns:
        viewing_positions: List of viewing positions, each containing:
            - position: Viewing position coordinates (x, y, z)
            - orientation: Viewing position orientation (quaternion)
            - angle: Viewing angle (radians)
    """
    viewing_positions = []
    
    for i, angle in enumerate(viewing_angles):
        # Calculate viewing position
        view_x = box_pose.position.x + viewing_distance * math.cos(angle)
        view_y = box_pose.position.y + viewing_distance * math.sin(angle)
        
        # Calculate angle facing the box
        dx = box_pose.position.x - view_x
        dy = box_pose.position.y - view_y
        facing_angle = math.atan2(dy, dx)  # Correct parameter order for atan2 is (y, x)
        
        # Create quaternion representing orientation
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
        
        # Add to result list
        viewing_positions.append({
            'position': position,
            'orientation': orientation,
            'angle': facing_angle
        })
    
    return viewing_positions

def visualize_box_positions_and_viewing_positions(box_pose, box_size, viewing_positions, view_positions_publisher):
    """Visualize box position and best viewing position markers"""
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
    box_marker.scale.x = box_size
    box_marker.scale.y = box_size
    box_marker.scale.z = box_size
    
    # Set box color to red
    box_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)  # R, G, B, A
    box_marker.lifetime = rospy.Duration(10.0)  # Display for 10 seconds
    
    marker_array.markers.append(box_marker)
    
    # Create viewing position markers
    for i, viewing_position in enumerate(viewing_positions):            
        # Create viewing position marker
        view_marker = Marker()
        view_marker.header.frame_id = "map"
        view_marker.header.stamp = rospy.Time.now()
        view_marker.ns = "view_position"
        view_marker.id = i + 1  # Start at 1, 0 is for the box
        view_marker.type = Marker.ARROW  # Use arrow to represent robot direction
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
        
        # Set color, using different colors to distinguish the four positions
        colors = [
            ColorRGBA(0.0, 0.8, 0.0, 0.8),  # Green - 0°
            ColorRGBA(0.0, 0.0, 0.8, 0.8),  # Blue - 90°
            ColorRGBA(0.8, 0.8, 0.0, 0.8),  # Yellow - 180°
            ColorRGBA(0.8, 0.0, 0.8, 0.8)   # Purple - 270°
        ]
        view_marker.color = colors[i]
        view_marker.lifetime = rospy.Duration(10.0)  # Display for 10 seconds
        
        marker_array.markers.append(view_marker)
        
        # Add connection line, from viewing position to box
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "view_lines"
        line_marker.id = i + 5  # Start at 5 to avoid ID conflict
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
    view_positions_publisher.publish(marker_array)
    rospy.loginfo("Box and viewing positions visualization published")

def navigate_to_best_viewing_positions_and_visualize_and_ocr(viewing_angles, viewing_distance, box_pose, box_size, 
                                       x_min, y_min, x_max, y_max, 
                                       client, navigation_timeout, ocr_trigger_publisher, view_positions_publisher):
    """Calculate and navigate to best viewing positions around one box
    Parameters:
        viewing_angles: List of viewing angles
        viewing_distance: Distance from the box to the viewing position
        box_pose: Pose of the box (geometry_msgs/Pose)
        box_size: Size of the box (meters)
        x_min: Minimum x coordinate of the area
        y_min: Minimum y coordinate of the area
        x_max: Maximum x coordinate of the area
        y_max: Maximum y coordinate of the area
        client: MoveBase ActionClient
        navigation_timeout: Navigation timeout (seconds)
        ocr_trigger_publisher: Publisher for OCR trigger messages
        view_positions_publisher: Publisher for visualization markers
    Returns:
        ocr_result: Result of OCR processing (if any)
    """     
    # Calculate viewing positions
    viewing_positions = calculate_box_viewing_positions(viewing_angles, viewing_distance, box_pose)
    
    # Publish visualization markers
    visualize_box_positions_and_viewing_positions(box_pose, box_size, viewing_positions, view_positions_publisher)
    
    global latest_ocr_result
    latest_ocr_result = None
    navigation_result = None
            
    for i, viewing_position in enumerate(viewing_positions):
        # Check if position is near the cliff
        if (viewing_position['position']['x'] < x_min or
            viewing_position['position']['x'] > x_max or
            viewing_position['position']['y'] < y_min or
            viewing_position['position']['y'] > y_max):
            # Skip this position if too close to the map boundary
            rospy.logwarn('Viewing position is too close to the map boundary, skipping...')
            continue

        # Create pose facing the box
        navigation_result = navigate_to_goal(client, viewing_position['position']['x'], viewing_position['position']['y'], viewing_position['position']['z'], 
                         viewing_position['orientation']['w'], viewing_position['orientation']['z'],
                         navigation_timeout, f"viewing position {i+1}")
        
        if navigation_result != 'Arrived':
            rospy.logwarn(f'Failed to navigate to viewing position {i+1}, skipping...')
            continue
        # After reaching viewing position, trigger OCR
        ocr_trigger_msg = Bool()
        ocr_trigger_msg.data = True
        ocr_trigger_publisher.publish(ocr_trigger_msg)
        rospy.loginfo('Triggering OCR at viewing position...')
        rospy.sleep(2.0)  # Give OCR time to process

        # Check if OCR result is available
        if latest_ocr_result is not None:
            rospy.loginfo(f'OCR result: {latest_ocr_result}')
        else:
            rospy.logwarn('OCR result not available, skipping...')

def navigate_to_goal(client, x, y, z=0.0, orientation_w=1.0, orientation_z=0.0, timeout=60.0, description="goal"):
    """
    导航到指定目标点

    参数:
        client: MoveBase ActionClient
        x: 目标x坐标
        y: 目标y坐标
        z: 目标z坐标，默认为0.0
        orientation_w: 四元数w分量，默认为1.0
        orientation_z: 四元数z分量，默认为0.0
        timeout: 导航超时时间（秒）
        description: 目标点描述，用于日志

    返回:
        成功返回True，失败返回False
    """
    try:
        # 创建导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = orientation_w
        goal.target_pose.pose.orientation.z = orientation_z
        
        # 记录日志
        rospy.loginfo(f'navigating to {description}: x={x:.2f}, y={y:.2f}')
        
        # 发送目标
        client.send_goal(goal)
        
        # 等待结果，带超时
        if not client.wait_for_result(rospy.Duration(timeout)):
            rospy.logwarn(f'导航到{description}超时')
            return 'Navigation timeout'
            
        # 检查结果
        result = client.get_state()
        if result == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f'Arrive at {description}')
            return 'Arrived'
        else:
            rospy.logwarn(f'Navigate to {description} failed, code: {result}')
            return 'Not arrived'
            
    except Exception as e:
        rospy.logerr(f'导航到{description}出错: {str(e)}')
        return 'Error'

# example usage
# from utils import detect_object_from_costmap, visualize_area, calculate_box_viewing_positions, navigate_to_best_viewing_positions