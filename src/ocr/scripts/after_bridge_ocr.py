#!/usr/bin/env python
import rospy
import cv2
import pytesseract
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge

class PostBridgeOCRNode:
    def __init__(self):
        rospy.init_node("post_bridge_ocr_node")
        self.bridge = CvBridge()
        self.ocr_enabled = False
        self.task_complete = False
        self.target_digit = None

        # 参数配置
        image_topic = rospy.get_param("~image_topic", "/front/image_raw")
        self.min_contour_area = rospy.get_param("~min_contour_area", 100)
        self.target_height = rospy.get_param("~target_height", 64)

        # 订阅/发布设置
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.ocr_trigger_sub = rospy.Subscriber("/ocr_trigger", Bool, self.ocr_trigger_callback)
        self.mode_digit_sub = rospy.Subscriber("/mode_digit", Int32, self.mode_digit_callback)
        self.recognized_digit_pub = rospy.Publisher("/recognized_digit_post", Int32, queue_size=1)
        self.cmd_stop_pub = rospy.Publisher("/cmd_stop", Bool, queue_size=1)


        rospy.loginfo("Post-bridge OCR node initialized")

    def ocr_trigger_callback(self, msg):
        if msg.data and not self.task_complete:
            self.ocr_enabled = True
            rospy.logdebug("OCR trigger received")

    def mode_digit_callback(self, msg):
        self.target_digit = msg.data
        rospy.loginfo(f"Received target digit: {self.target_digit}")

    def image_callback(self, msg):
        if not self.ocr_enabled or self.task_complete:
            return

        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CvBridge error: {str(e)}")
            return

        # 增强的图像预处理流程
        processed_roi = self.process_image(cv_image)
        
        if processed_roi is None:
            rospy.logwarn("No valid ROI detected")
            self.ocr_enabled = False
            return

        # OCR识别
        digit = self.ocr_recognition(processed_roi)
        
        if digit is not None:
            self.handle_recognition_result(digit)
        
        self.ocr_enabled = False

    def process_image(self, cv_image):
        """图像处理流水线，返回预处理后的ROI"""
        # 转换为灰度图
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 自适应阈值二值化
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 51, 6
        )
        
        # 形态学操作增强特征
        kernel = np.ones((3,3), np.uint8)
        processed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # 查找轮廓
        contours, _ = cv2.findContours(
            processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # 筛选有效轮廓
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)
            
            # 根据实际场景调整参数
            if (area > self.min_contour_area and 
                w > 15 and h > 30 and 
                0.3 < aspect_ratio < 2.0):
                valid_contours.append(cnt)
        
        if not valid_contours:
            rospy.logdebug("No valid contours found")
            return None
        
        # 选择最大轮廓
        largest_cnt = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_cnt)
        
        # 扩展边界
        pad = 10
        x = max(0, x - pad)
        y = max(0, y - pad)
        w = min(processed.shape[1]-x, w + 2*pad)
        h = min(processed.shape[0]-y, h + 2*pad)
        
        roi = processed[y:y+h, x:x+w]
        
        # 标准化处理
        scale = self.target_height / roi.shape[0]
        resized = cv2.resize(roi, 
            (int(roi.shape[1]*scale), self.target_height),
            interpolation=cv2.INTER_CUBIC
        )
        
        # 后处理
        filtered = cv2.medianBlur(resized, 3)
        bordered = cv2.copyMakeBorder(
            filtered, 20, 20, 20, 20,
            cv2.BORDER_CONSTANT, value=0
        )
        
        return bordered

    def ocr_recognition(self, processed_image):
        """执行OCR识别"""
        custom_config = r'--oem 3 --psm 7 -c tessedit_char_whitelist=0123456789'
        try:
            text = pytesseract.image_to_string(
                processed_image,
                config=custom_config,
                lang='eng'
            ).strip()
            
            if text.isdigit():
                return int(text)
        except Exception as e:
            rospy.logwarn(f"OCR error: {str(e)}")
        return None

    def handle_recognition_result(self, digit):
        """处理识别结果"""
        rospy.loginfo(f"Recognized digit: {digit}")
        self.recognized_digit_pub.publish(digit)
        
        if self.target_digit is not None and digit == self.target_digit:
            rospy.loginfo("Target matched! Sending stop command")
            self.cmd_stop_pub.publish(True)
            self.task_complete = True

if __name__ == '__main__':
    try:
        node = PostBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass