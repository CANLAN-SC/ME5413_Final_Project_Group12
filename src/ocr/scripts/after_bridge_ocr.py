#!/usr/bin/env python
import rospy
import cv2
import pytesseract
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import numpy as np

class PostBridgeOCRNode:
    def __init__(self):
        rospy.init_node("post_bridge_ocr_node")
        self.bridge = CvBridge()
        # 标记是否允许OCR（由3D LiDAR检测节点发出触发指令控制）
        self.ocr_enabled = False
        # 标记任务是否已经完成
        self.task_complete = False
        # 存储从预先阶段获得的目标数字（出现次数最少的数字）
        self.target_digit = None

        # 订阅图像话题，参数化配置（默认为 /camera/image_raw）
        image_topic = rospy.get_param("~image_topic", "/front/image_raw")
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        # 订阅OCR触发指令（由3D LiDAR检测节点发出）
        self.ocr_trigger_sub = rospy.Subscriber("/ocr_trigger", Bool, self.ocr_trigger_callback)
        # 订阅预先阶段发布的目标数字（最少出现的数字），类型为Int32
        self.mode_digit_sub = rospy.Subscriber("/mode_digit", Int32, self.mode_digit_callback)

        # 发布识别到的数字（调试用）
        self.recognized_digit_pub = rospy.Publisher("/recognized_digit_post", Int32, queue_size=1)
        # 发布停靠指令，当识别数字与目标数字匹配时，发布True，表示机器人停止（任务完成）
        self.cmd_stop_pub = rospy.Publisher("/cmd_stop", Bool, queue_size=1)
        
        rospy.loginfo("Post-bridge OCR node started, waiting for OCR triggers and target digit.")

    def ocr_trigger_callback(self, msg):
        """
        当收到3D LiDAR检测节点发出的OCR触发指令时，设置ocr_enabled为True，
        后续在图像回调中处理OCR识别。
        """
        if not self.task_complete and msg.data:
            self.ocr_enabled = True
            rospy.loginfo("Post-bridge OCR trigger received, will process next image.")

    def mode_digit_callback(self, msg):
        """
        接收到预先阶段发布的目标数字（出现次数最少的数字），保存到self.target_digit。
        """
        self.target_digit = msg.data
        rospy.loginfo("Received target digit: %d", self.target_digit)

    def image_callback(self, msg):
        """
        图像回调函数。当ocr_enabled为True且任务未完成时，对当前图像进行OCR识别，
        并判断识别结果是否与目标数字一致。如果一致，则发布停靠指令，表示任务完成。
        """
        if not self.ocr_enabled or self.task_complete:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        # 图像预处理：转灰度、二值化，提取图像中心区域作为ROI（可根据实际情况调整）
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)
        h, w = thresh.shape
        roi = thresh[int(0.21*h):int(0.79*h), int(0.34*w):int(0.66*w)]
        # 显示ROI便于调试（可选）
        cv2.imshow("Post-bridge OCR ROI", roi)
        cv2.waitKey(1)

        # 使用Tesseract OCR识别数字
        # 参数 --psm 10 表示识别单个字符，同时仅允许识别数字0~9
        custom_config = r'--psm 10 -c tessedit_char_whitelist=0123456789'
        ocr_text = pytesseract.image_to_string(roi, config=custom_config)
        ocr_text = ocr_text.strip()

        if ocr_text and ocr_text.isdigit():
            digit = int(ocr_text)
            rospy.loginfo("Post-bridge recognized digit: %d", digit)
            self.recognized_digit_pub.publish(digit)
            # 如果识别数字与目标数字一致，则发布停靠指令
            if self.target_digit is not None and digit == self.target_digit:
                rospy.loginfo("Target digit matched! Stopping the robot.")
                self.cmd_stop_pub.publish(True)
                self.task_complete = True
        else:
            rospy.loginfo("No valid digit recognized in this post-bridge frame. OCR result: '%s'", ocr_text)

        # 重置OCR触发标志，等待下一次触发
        self.ocr_enabled = False

if __name__ == '__main__':
    try:
        node = PostBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
