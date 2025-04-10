#!/usr/bin/env python
import rospy
import cv2
import pytesseract
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
from collections import Counter

class PreBridgeOCRNode:
    def __init__(self):
        rospy.init_node("pre_bridge_ocr_node")
        self.bridge = CvBridge()
        self.ocr_results = []
        self.pre_bridge_complete = False
        self.ocr_enabled = False

        image_topic = rospy.get_param("~image_topic", "/front/image_raw")
        
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.ocr_trigger_sub = rospy.Subscriber("/ocr_trigger", Bool, self.ocr_trigger_callback)
        self.bridge_unlock_sub = rospy.Subscriber("/cmd_open_bridge", Bool, self.bridge_unlock_callback)
        
        self.digit_pub = rospy.Publisher("/recognized_digit", Int32, queue_size=1)
        self.mode_digit_pub = rospy.Publisher("/mode_digit", Int32, queue_size=1)

        # 设置Tesseract路径（根据实际安装位置调整）
        #pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

    def ocr_trigger_callback(self, msg):
        if not self.pre_bridge_complete and msg.data:
            self.ocr_enabled = True

    def bridge_unlock_callback(self, msg):
        if not self.pre_bridge_complete and msg.data:
            if self.ocr_results:
                counter = Counter(self.ocr_results)
                min_digit = min(counter, key=counter.get)
                self.mode_digit_pub.publish(min_digit)
            self.pre_bridge_complete = True

    def image_callback(self, msg):
        if not self.ocr_enabled or self.pre_bridge_complete:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # 增强的预处理流程
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 改进的二值化方法
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 31, 6
        )

        # 通过轮廓检测定位数字区域
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        roi = None
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            # 根据实际场景调整筛选条件
            if h > 30 and w > 15 and 0.5 < w/h < 2.0:
                # 扩展边界确保包含完整字符
                pad = 5
                x = max(0, x - pad)
                y = max(0, y - pad)
                w = min(thresh.shape[1]-x, w + 2*pad)
                h = min(thresh.shape[0]-y, h + 2*pad)
                roi = thresh[y:y+h, x:x+w]
                break

        if roi is None:
            rospy.logwarn("No valid contour found!")
            self.ocr_enabled = False
            return

        # 优化图像尺寸
        target_height = 64
        scale = target_height / roi.shape[0]
        resized_roi = cv2.resize(
            roi, 
            (int(roi.shape[1]*scale), target_height),
            interpolation=cv2.INTER_CUBIC
        )

        # 附加后处理
        processed_roi = cv2.medianBlur(resized_roi, 3)
        processed_roi = cv2.copyMakeBorder(
            processed_roi,
            20, 20, 20, 20,
            cv2.BORDER_CONSTANT, 
            value=0
        )

        # 调试保存图像
        #cv2.imwrite("/tmp/last_ocr_input.png", processed_roi)

        # 优化OCR参数
        custom_config = r'--oem 3 --psm 7 -c tessedit_char_whitelist=0123456789'
        ocr_text = pytesseract.image_to_string(
            processed_roi, 
            config=custom_config,
            lang='eng'
        ).strip()

        if ocr_text and ocr_text.isdigit():
            digit = int(ocr_text)
            rospy.loginfo(f"Recognized digit: {digit}")
            self.ocr_results.append(digit)
            self.digit_pub.publish(digit)
        else:
            rospy.logwarn(f"Recognition failed. Raw output: '{ocr_text}'")

        self.ocr_enabled = False

if __name__ == '__main__':
    try:
        node = PreBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass