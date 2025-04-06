#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import sys

def publish_goal(x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = "map"  # 根据你的 TF 坐标系，如 "map" 或 "odom"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    # 将 yaw (弧度) 转换为四元数
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    rospy.loginfo("Publishing goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
    pub.publish(goal)

if __name__ == '__main__':
    rospy.init_node('interactive_goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # 等待发布者注册

    rospy.loginfo("please enter x, y, yaw，such as：1.0 0.9 0")
    rospy.loginfo("enter 'exit' or 'quit' exit program")

    # 主循环：不断从终端读取输入
    while not rospy.is_shutdown():
        try:
            # 根据 Python 版本选择合适的输入函数
            if sys.version_info[0] < 3:
                line = raw_input("please enter x y yaw (or 'exit' ):")
            else:
                line = input("please enter x y yaw (or 'exit' ): ")
        except EOFError:
            break

        if line.lower() in ['exit', 'quit']:
            break

        parts = line.split()
        if len(parts) != 3:
            rospy.logwarn("make sure enter 3 number: x y yaw")
            continue

        try:
            x, y, yaw = map(float, parts)
            publish_goal(x, y, yaw)
        except Exception as e:
            rospy.logwarn("error: %s", e)
    
    rospy.loginfo("exit move_base_simple/goal publish")
