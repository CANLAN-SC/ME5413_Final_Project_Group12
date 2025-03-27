#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN

class BoxDetector:
    def __init__(self):
        rospy.init_node("box_detector", anonymous=True)

        # Set the frequency for publishing messages (10 Hz)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscribe to ROS topics
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)

        # Publish detected 3D object center points
        self.pose_pub = rospy.Publisher("/detected_boxes", PoseArray, queue_size=10)

        # Publish bounding box visualization
        self.marker_pub = rospy.Publisher("/bounding_boxes", MarkerArray, queue_size=10)

        # Define maximum size thresholds for the bounding box
        self.max_length = 1.2  # Maximum length (x direction)
        self.max_width = 1.2   # Maximum width (y direction)
        self.max_height = 1.2  # Maximum height (z direction)

        # Define minimum size thresholds for the bounding box
        self.min_length = 0.4  # Maximum length (x direction)
        self.min_width = 0.4   # Maximum width (y direction)
        self.min_height = 0.4  # Maximum height (z direction)

        # Define box area
        # self.min_x_coord = 2.0
        # self.min_y_coord = 11.0
        # self.max_x_coord = 22.0
        # self.max_y_coord = 19.0

        # Define alpha for filtering ground points
        self.alpha = 0.05

        rospy.loginfo("BoxDetector has started, waiting for point cloud data...")

    def pointcloud_callback(self, msg):
        # Parse PointCloud2 data
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)))

        if points.shape[0] == 0:
            rospy.logwarn("Point cloud data is empty")
            return
        
        # Compute z value of the ground points
        min_z_global = np.min(points[:, 2])

        # Filter ground points
        filtered_points = points[(points[:, 2] >= min_z_global + self.alpha)]
        
        # Filter points based on x, y coordinates
        #filtered_points = points[
            #(points[:, 0] >= self.min_x_coord) & (points[:, 0] <= self.max_x_coord) &
            #(points[:, 1] >= self.min_y_coord) & (points[:, 1] <= self.max_y_coord)
        #]

        # Check if there are points left after filtering
        #if filtered_points.shape[0] == 0:
            #return
        
        # Use DBSCAN clustering to detect boxes
        clustering = DBSCAN(eps=0.3, min_samples=10).fit(filtered_points)
        labels = clustering.labels_

        # Get unique cluster IDs (-1 represents noise points, ignore them)
        unique_labels = set(labels) - {-1}

        pose_array = PoseArray()
        marker_array = MarkerArray()

        for label in unique_labels:
            cluster_points = filtered_points[labels == label]

            # Compute the bounding box
            cluster_points_2d = cluster_points[:, :2]
            min_x = np.min(cluster_points_2d[:, 0])
            max_x = np.max(cluster_points_2d[:, 0])
            min_y = np.min(cluster_points_2d[:, 1])
            max_y = np.max(cluster_points_2d[:, 1])
            min_z = np.min(cluster_points[:, 2])
            max_z = np.max(cluster_points[:, 2])

            # Compute the length, width, and height of the bounding box
            length = max_x - min_x
            width = max_y - min_y
            height = max_z - min_z

            # Check if the bounding box size is within the allowed range
            if not (self.min_height <= height <= self.max_height):
                continue  # Skip this bounding box
            if not (self.min_length <= length <= self.max_length or 
                    self.min_width <= width <=self.max_width):
                continue  # Skip this bounding box

            # Create Pose message (box center)
            pose = Pose()
            pose.position.x = (min_x + max_x) / 2
            pose.position.y = (min_y + max_y) / 2
            pose.position.z = (min_z + max_z) / 2
            pose_array.poses.append(pose)

            # Create Bounding Box visualization Marker
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "boxes"
            marker.id = int(label)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (min_x + max_x) / 2
            marker.pose.position.y = (min_y + max_y) / 2
            marker.pose.position.z = (min_z + max_z) / 2
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale of the bounding box
            marker.scale.x = length + 0.1
            marker.scale.y = width + 0.1
            marker.scale.z = height + 0.1

            # Set the color and transparency of the bounding box
            marker.color.a = 0.5  # Transparency
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            # Create a text marker to display bounding box size
            text_marker = Marker()
            text_marker.header.frame_id = msg.header.frame_id
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "box_text"
            text_marker.id = int(label) + 1000  # Avoid ID conflict
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = (min_x + max_x) / 2
            text_marker.pose.position.y = (min_y + max_y) / 2
            text_marker.pose.position.z = max_z + 0.2  # Position text above the box

            # Set text content (display length, width, and height)
            text_marker.text = f"L:{length:.2f}, W:{width:.2f}, H:{height:.2f}"

            # Set text scale (font size)
            text_marker.scale.z = 0.2  # Text size

            # Set text color (white)
            text_marker.color.a = 1.0  # Fully opaque
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0

            # Add to marker_array
            marker_array.markers.append(text_marker)

        # Publish detected box center points
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = msg.header.frame_id
        self.pose_pub.publish(pose_array)

        # Publish Bounding Box to RViz
        self.marker_pub.publish(marker_array)

        # Sleep to control the rate (10 Hz)
        self.rate.sleep()

if __name__ == "__main__":
    try:
        detector = BoxDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass