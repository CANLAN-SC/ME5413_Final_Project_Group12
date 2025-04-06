#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import time

class RedConeNavigator:
    def __init__(self):
        rospy.init_node('red_cone_navigator', anonymous=True)

        # Subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback, queue_size=1)
        self.pc_sub = rospy.Subscriber("/mid/points", PointCloud2, self.pc_callback, queue_size=1)

        # Publishers
        self.bridge_cmd_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # TF tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.latest_image = None
        self.latest_pointcloud = None
        self.task_started = False

        # Collect multiple positions
        self.detected_positions = []
        self.required_samples = 10

        rospy.loginfo("Red cone navigator node started.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"CVBridge error: {e}")

    def pc_callback(self, msg):
        self.latest_pointcloud = msg
        self.process()

    def publish_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        rospy.loginfo(f" Publishing goal to: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        self.goal_pub.publish(goal)

    def process(self):
        if self.latest_image is None or self.latest_pointcloud is None or self.task_started:
            return

        image = self.latest_image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])

        selected_points = []
        for p in point_cloud2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True):
            if 1.5 < p[0] < 5.0 and abs(p[1]) < 1.0 and 0 < p[2] < 3.0:
                selected_points.append(p)

        if not selected_points:
            return

        pts = np.array(selected_points)
        median_point = np.median(pts, axis=0)

        pt = PointStamped()
        pt.header.frame_id = self.latest_pointcloud.header.frame_id
        pt.header.stamp = rospy.Time.now()
        pt.point.x, pt.point.y, pt.point.z = median_point

        try:
            pt_out = self.tf_buffer.transform(pt, "map", rospy.Duration(1.0))
            self.detected_positions.append([pt_out.point.x, pt_out.point.y])

            if len(self.detected_positions) < self.required_samples:
                rospy.loginfo(f" Collecting red cone positions: {len(self.detected_positions)}/{self.required_samples}")
                return

            avg_position = np.mean(np.array(self.detected_positions), axis=0)
            cone_x, cone_y = avg_position[0], avg_position[1]

            rospy.loginfo(f" Averaged red cone position: ({cone_x:.2f}, {cone_y:.2f})")
            self.task_started = True

            # 1. Move to 1.5m in front of the cone
            self.publish_goal(cone_x + 1.5, cone_y, 3.14)
            rospy.sleep(5)

            # 2. Clear the cone
            rospy.loginfo(" Clearing the red cone via /cmd_open_bridge")
            self.bridge_cmd_pub.publish(Bool(data=True))


            # 3. Move 1.5m past the cone
            self.publish_goal(cone_x - 1.5, cone_y, 3.14)
            rospy.sleep(2)
            rospy.loginfo(" Published goal to 1.5m past the cone.")

            self.publish_goal(cone_x - 3.0, cone_y, 3.14)
            rospy.loginfo(" Published goal to 3.0m past the cone.")


            # 4. (removed extra forward movement)

        except Exception as e:
            rospy.logwarn(f"TF transform failed: {e}")

if __name__ == '__main__':
    try:
        RedConeNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
