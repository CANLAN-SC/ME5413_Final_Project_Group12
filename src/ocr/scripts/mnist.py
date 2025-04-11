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
        # Flag indicating if OCR is allowed (controlled by trigger command from 3D LiDAR detection node)
        self.ocr_enabled = False
        # Flag indicating if the task is completed
        self.task_complete = False
        # Store the target digit obtained from the preliminary stage (the least frequent digit)
        self.target_digit = None
        
        # Load the pre-trained MNIST model
        self.model = self.load_mnist_model()
        
        # Subscribe to image topic, parameterized configuration (default is /camera/image_raw)
        image_topic = rospy.get_param("~image_topic", "/front/image_raw")
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        # Subscribe to OCR trigger command (from 3D LiDAR detection node)
        self.ocr_trigger_sub = rospy.Subscriber("/ocr_trigger", Bool, self.ocr_trigger_callback)
        # Subscribe to the target digit (least frequent digit) published from preliminary stage, type Int32
        self.mode_digit_sub = rospy.Subscriber("/mode_digit", Int32, self.mode_digit_callback)

        # Publish recognized digit (for debugging)
        self.recognized_digit_pub = rospy.Publisher("/recognized_digit_post", Int32, queue_size=1)
        # Publish docking command, publish True when recognized digit matches target digit, indicating robot to stop (task complete)
        self.cmd_stop_pub = rospy.Publisher("/cmd_stop", Bool, queue_size=1)
        
        rospy.loginfo("Post-bridge MNIST recognition node started, waiting for triggers and target digit.")
    
    def load_mnist_model(self):
        """
        Load pre-trained TensorFlow MNIST model
        """
        try:
            # Load MNIST dataset (only for obtaining model)
            mnist = tf.keras.datasets.mnist
            (_, _), (_, _) = mnist.load_data()
            
            # Create a simple CNN model
            model = tf.keras.models.Sequential([
                tf.keras.layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)),
                tf.keras.layers.MaxPooling2D((2, 2)),
                tf.keras.layers.Conv2D(64, (3, 3), activation='relu'),
                tf.keras.layers.MaxPooling2D((2, 2)),
                tf.keras.layers.Flatten(),
                tf.keras.layers.Dense(64, activation='relu'),
                tf.keras.layers.Dense(10, activation='softmax')
            ])
            
            # Compile the model
            model.compile(optimizer='adam',
                         loss='sparse_categorical_crossentropy',
                         metrics=['accuracy'])
                         
            # Here should load trained model weights if available
            # model.load_weights('mnist_model_weights.h5')
            
            # Since we don't have actual weight files, we train a simple model here
            # In practical applications, a pre-trained and saved model should be used
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
        When receiving OCR trigger command from 3D LiDAR detection node, set ocr_enabled to True,
        and process OCR recognition in subsequent image callbacks.
        """
        if not self.task_complete and msg.data:
            self.ocr_enabled = True
            rospy.loginfo("Post-bridge OCR trigger received, will process next image.")

    def mode_digit_callback(self, msg):
        """
        Receive target digit (least frequent digit) published from preliminary stage, save to self.target_digit.
        """
        self.target_digit = msg.data
        rospy.loginfo("Received target digit: %d", self.target_digit)

    def preprocess_image_for_mnist(self, img):
        """
        Preprocess image to fit MNIST model input
        """
        # Ensure image is grayscale
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Apply threshold processing to binarize the image
        _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
        
        # Resize to MNIST format (28x28)
        img = cv2.resize(img, (28, 28))
        
        # Normalize
        img = img / 255.0
        
        # Expand dimensions to match model input requirements
        img = np.expand_dims(img, axis=0)  # Add batch dimension
        img = np.expand_dims(img, axis=-1)  # Add channel dimension
        
        return img

    def image_callback(self, msg):
        """
        Image callback function. When ocr_enabled is True and task is not completed, perform digit recognition on the current image,
        and determine if the recognition result matches the target digit. If they match, publish docking command, indicating task completion.
        """
        if not self.ocr_enabled or self.task_complete:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        # 1. Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 2. Crop region of interest
        h, w = gray.shape
        # roi = gray[int(0.21*h):int(0.79*h), int(0.25*w):int(0.75*w)]
        # no roi
        roi = gray
        
        # 3. Image preprocessing
        processed_img = self.preprocess_image_for_mnist(roi)
        
        # 4. Use MNIST model for prediction
        if self.model is not None:
            predictions = self.model.predict(processed_img, verbose=0)
            digit = np.argmax(predictions[0])
            confidence = np.max(predictions[0])
            
            rospy.loginfo(f"Post-bridge recognized digit: {digit} (confidence: {confidence:.2f})")
            self.recognized_digit_pub.publish(int(digit))
            
            # If recognized digit matches target digit, publish docking command
            if self.target_digit is not None and int(digit) == self.target_digit:
                rospy.loginfo("Target digit matched! Stopping the robot.")
                self.cmd_stop_pub.publish(True)
                self.task_complete = True
        else:
            rospy.logerr("MNIST model not available. Cannot perform recognition.")

        # Reset OCR trigger flag, wait for next trigger
        self.ocr_enabled = False

if __name__ == '__main__':
    try:
        node = PostBridgeOCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass