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

        # Subscribe to ROS topics
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)

        # Publish detected 3D object center points
        self.pose_pub = rospy.Publisher("/detected_boxes", PoseArray, queue_size=10)

        # Publish bounding box visualization
        self.marker_pub = rospy.Publisher("/bounding_boxes", MarkerArray, queue_size=10)

        # Define maximum size thresholds for the bounding box
        self.max_length = 1.5  # Maximum length (x direction)
        self.max_width = 1.5   # Maximum width (y direction)
        self.max_height = 1.5  # Maximum height (z direction)

        # Define box area
        # self.min_x_coord = 2.0
        # self.min_y_coord = 11.0
        # self.max_x_coord = 22.0
        # self.max_y_coord = 19.0

        rospy.loginfo("BoxDetector has started, waiting for point cloud data...")

    def pointcloud_callback(self, msg):
        # Parse PointCloud2 data
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)))

        if points.shape[0] == 0:
            rospy.logwarn("Point cloud data is empty")
            return
        
        # Filter points based on x, y coordinates
        #filtered_points = points[
            #(points[:, 0] >= self.min_x_coord) & (points[:, 0] <= self.max_x_coord) &
            #(points[:, 1] >= self.min_y_coord) & (points[:, 1] <= self.max_y_coord)
        #]

        # Check if there are points left after filtering
        #if filtered_points.shape[0] == 0:
            #return
        
        # Use DBSCAN clustering to detect boxes
        clustering = DBSCAN(eps=0.3, min_samples=10).fit(points)
        labels = clustering.labels_

        # Get unique cluster IDs (-1 represents noise points, ignore them)
        unique_labels = set(labels) - {-1}

        pose_array = PoseArray()
        marker_array = MarkerArray()

        for label in unique_labels:
            cluster_points = points[labels == label]

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

            # Check if the bounding box size exceeds the maximum allowed dimensions
            if length > self.max_length or width > self.max_width or height > self.max_height:
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

            # Set the scale of the bounding box
            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = height

            # Set the color and transparency of the bounding box
            marker.color.a = 0.5  # Transparency
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        # Publish detected box center points
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = msg.header.frame_id
        self.pose_pub.publish(pose_array)

        # Publish Bounding Box to RViz
        self.marker_pub.publish(marker_array)

if __name__ == "__main__":
    try:
        detector = BoxDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass