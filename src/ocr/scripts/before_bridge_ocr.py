#!/usr/bin/env python
import rospy
import cv2
import pytesseract
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
        
        rospy.loginfo("Pre-bridge OCR node started, waiting for OCR triggers and bridge unlock signal.")
    
    def ocr_trigger_callback(self, msg):
        if not self.pre_bridge_complete and msg.data:
            self.ocr_enabled = True
            rospy.loginfo("OCR trigger received, will process next image for OCR.")
    
    def bridge_unlock_callback(self, msg):
        if not self.pre_bridge_complete and msg.data:
            rospy.loginfo("Bridge unlock signal received, calculating min frequency digit.")
            if self.ocr_results:
                counter = Counter(self.ocr_results)
                min_digit = min(counter, key=counter.get)
                rospy.loginfo("Min frequency digit is: %d", min_digit)
                self.mode_digit_pub.publish(min_digit)
            else:
                rospy.logwarn("No OCR results collected, cannot compute min frequency digit.")
            self.pre_bridge_complete = True
    
    def image_callback(self, msg):
        """
        图像回调函数。仅当OCR触发标志有效且过桥前阶段未完成时，
        对当前图像进行预处理、OCR识别，并保存结果。
        """
        if not self.ocr_enabled or self.pre_bridge_complete:
            return

        try:
            # 将图像转换为 OpenCV 格式并保存到本地
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            save_path = "./ocr_images/frame.jpg"  # 修改为你自己的保存路径
            cv2.imwrite(save_path, cv_image)

            # 从本地重新读取图像
            cv_image = cv2.imread(save_path)
            if cv_image is None:
                rospy.logerr("Failed to read saved image from path: %s", save_path)
                return

        except Exception as e:
            rospy.logerr("Image processing error: %s", e)
            return

        # 图像预处理：
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

        self.ocr_enabled = False

if __name__ == '__main__':
    try:
        node = PreBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
