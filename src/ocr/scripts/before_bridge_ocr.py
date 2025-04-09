#!/usr/bin/env python
import rospy
import cv2
import pytesseract
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
from collections import Counter
import numpy as np

class PreBridgeOCRNode:
    def __init__(self):
        rospy.init_node("pre_bridge_ocr_node")
        self.bridge = CvBridge()
        # 存储所有箱子识别得到的数字
        self.ocr_results = []
        # 标记是否完成过桥前OCR采集（即接收到桥梁解锁信号后停止采集）
        self.pre_bridge_complete = False
        # OCR触发标志，由3D LiDAR检测节点发布触发指令控制
        self.ocr_enabled = False
        
        # 参数配置：图像话题名称（可在launch中配置）
        image_topic = rospy.get_param("~image_topic", "/front/image_raw")
        
        # 订阅摄像头图像
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        # 订阅OCR触发指令（由3D LiDAR检测节点发出）
        self.ocr_trigger_sub = rospy.Subscriber("/ocr_trigger", Bool, self.ocr_trigger_callback)
        # 订阅桥梁解锁指令（收到True消息后，统计OCR结果）
        self.bridge_unlock_sub = rospy.Subscriber("/cmd_open_bridge", Bool, self.bridge_unlock_callback)
        
        # 发布识别到的单个数字（用于调试）
        self.digit_pub = rospy.Publisher("/recognized_digit", Int32, queue_size=1)
        # 发布统计得到的出现次数最少的数字（后续过桥后比对时使用）
        self.mode_digit_pub = rospy.Publisher("/mode_digit", Int32, queue_size=1)
        
        rospy.loginfo("Pre-bridge OCR node started, waiting for OCR triggers and bridge unlock signal.")
    
    def ocr_trigger_callback(self, msg):
        """
        收到3D LiDAR检测节点发出的OCR触发指令。
        仅在过桥前阶段有效，触发后设置标志，在下一帧图像中执行OCR识别。
        """
        if not self.pre_bridge_complete and msg.data:
            self.ocr_enabled = True
            rospy.loginfo("OCR trigger received, will process next image for OCR.")
    
    def bridge_unlock_callback(self, msg):
        """
        收到桥梁解锁指令（True消息），此时统计所有已采集的OCR结果，
        计算出现次数最少的数字，并发布统计结果。
        """
        if not self.pre_bridge_complete and msg.data:
            rospy.loginfo("Bridge unlock signal received, calculating min frequency digit.")
            if self.ocr_results:
                counter = Counter(self.ocr_results)
                # 选择出现次数最少的数字，如果有多个数字，取其中一个（按字典序）
                min_digit = min(counter, key=counter.get)
                rospy.loginfo("Min frequency digit is: %d", min_digit)
                self.mode_digit_pub.publish(min_digit)
            else:
                rospy.logwarn("No OCR results collected, cannot compute min frequency digit.")
            # 标记过桥前OCR任务完成，后续不再采集
            self.pre_bridge_complete = True
    
    def image_callback(self, msg):
        """
        图像回调函数。仅当OCR触发标志有效且过桥前阶段未完成时，
        对当前图像进行预处理、OCR识别，并保存结果。
        """
        if not self.ocr_enabled or self.pre_bridge_complete:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: %s", e)
            return
        
        # 图像预处理：
        # 1. 转为灰度图
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 2. 自适应阈值二值化，反色处理使数字区域突出
        # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        #                                cv2.THRESH_BINARY_INV, 11, 2)
        h, w = gray.shape
        roi = gray[int(0.21*h):int(0.79*h), int(0.34*w):int(0.66*w)]
        
        # 显示ROI便于调试
        #cv2.imshow("OCR ROI", roi)
        #cv2.waitKey(1)
        
        # 使用Tesseract OCR进行识别：
        # --psm 10表示识别单个字符，设置白名单仅限数字0～9
        custom_config = r'--psm 7 -c tessedit_char_whitelist=0123456789'
        ocr_text = pytesseract.image_to_string(roi, config=custom_config)
        ocr_text = ocr_text.strip()
        
        if ocr_text and ocr_text.isdigit():
            digit = int(ocr_text)
            rospy.loginfo("Recognized digit: %d", digit)
            self.digit_pub.publish(digit)
            self.ocr_results.append(digit)
        else:
            rospy.loginfo("No valid digit recognized in this frame. OCR result: '%s'", ocr_text)
        
        # 重置OCR触发标志，等待下一次触发
        self.ocr_enabled = False

if __name__ == '__main__':
    try:
        node = PreBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
