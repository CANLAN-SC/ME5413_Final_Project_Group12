#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import tensorflow as tf
from PIL import Image as PILImage

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
        
        # 加载预训练的MNIST模型
        self.model = self.load_mnist_model()
        
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
        
        rospy.loginfo("Post-bridge MNIST recognition node started, waiting for triggers and target digit.")
    
    def load_mnist_model(self):
        """
        加载TensorFlow预训练的MNIST模型
        """
        try:
            # 加载MNIST数据集(仅用于获取模型)
            mnist = tf.keras.datasets.mnist
            (_, _), (_, _) = mnist.load_data()
            
            # 创建一个简单的CNN模型
            model = tf.keras.models.Sequential([
                tf.keras.layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)),
                tf.keras.layers.MaxPooling2D((2, 2)),
                tf.keras.layers.Conv2D(64, (3, 3), activation='relu'),
                tf.keras.layers.MaxPooling2D((2, 2)),
                tf.keras.layers.Flatten(),
                tf.keras.layers.Dense(64, activation='relu'),
                tf.keras.layers.Dense(10, activation='softmax')
            ])
            
            # 编译模型
            model.compile(optimizer='adam',
                         loss='sparse_categorical_crossentropy',
                         metrics=['accuracy'])
                         
            # 这里应该加载已训练好的模型权重，如果有的话
            # model.load_weights('mnist_model_weights.h5')
            
            # 由于我们没有实际的权重文件，这里我们训练一个简单的模型
            # 在实际应用中，应该使用预先训练好并保存的模型
            (train_images, train_labels), _ = mnist.load_data()
            train_images = train_images / 255.0
            train_images = train_images.reshape(-1, 28, 28, 1)
            model.fit(train_images, train_labels, epochs=1, batch_size=128, verbose=0)
            
            rospy.loginfo("MNIST model loaded successfully")

            # save the model for future use
            model.save('mnist_model.h5')
            return model
        except Exception as e:
            rospy.logerr("Error loading MNIST model: %s", e)
            return None

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

    def preprocess_image_for_mnist(self, img):
        """
        预处理图像以适应MNIST模型输入
        """
        # 确保图像为灰度图
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 应用阈值处理，将图像二值化
        _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
        
        # 调整大小为MNIST格式 (28x28)
        img = cv2.resize(img, (28, 28))
        
        # 归一化
        img = img / 255.0
        
        # 扩展维度以匹配模型输入要求
        img = np.expand_dims(img, axis=0)  # 添加batch维度
        img = np.expand_dims(img, axis=-1)  # 添加通道维度
        
        return img

    def image_callback(self, msg):
        """
        图像回调函数。当ocr_enabled为True且任务未完成时，对当前图像进行数字识别，
        并判断识别结果是否与目标数字一致。如果一致，则发布停靠指令，表示任务完成。
        """
        if not self.ocr_enabled or self.task_complete:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        # 1. 转换为灰度图
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 2. 裁剪感兴趣区域
        h, w = gray.shape
        # roi = gray[int(0.21*h):int(0.79*h), int(0.25*w):int(0.75*w)]
        # no roi
        roi = gray
        
        # 3. 图像预处理
        processed_img = self.preprocess_image_for_mnist(roi)
        
        # 4. 使用MNIST模型进行预测
        if self.model is not None:
            predictions = self.model.predict(processed_img, verbose=0)
            digit = np.argmax(predictions[0])
            confidence = np.max(predictions[0])
            
            rospy.loginfo(f"Post-bridge recognized digit: {digit} (confidence: {confidence:.2f})")
            self.recognized_digit_pub.publish(int(digit))
            
            # 如果识别数字与目标数字一致，则发布停靠指令
            if self.target_digit is not None and int(digit) == self.target_digit:
                rospy.loginfo("Target digit matched! Stopping the robot.")
                self.cmd_stop_pub.publish(True)
                self.task_complete = True
        else:
            rospy.logerr("MNIST model not available. Cannot perform recognition.")

        # 重置OCR触发标志，等待下一次触发
        self.ocr_enabled = False

if __name__ == '__main__':
    try:
        node = PostBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass