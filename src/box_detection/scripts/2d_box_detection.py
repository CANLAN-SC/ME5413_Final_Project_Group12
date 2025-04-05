#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from PIL import Image
import cv2
from geometry_msgs.msg import Pose, PoseStamped

class BoxDetector:
    def __init__(self):
        # 基本参数设置
        self.obstacle_threshold = 50  # 动态调整的初始阈值
        self.eps = 10  # DBSCAN聚类参数 - 邻域距离
        self.min_samples = 5  # DBSCAN聚类参数 - 最小样本数
        self.min_area = 20  # 最小检测区域面积（像素）
        
        # 地图分辨率和原点
        self.resolution = 0.05  # 米/像素
        self.origin_x = -12.0  # 地图原点X坐标
        self.origin_y = -12.0  # 地图原点Y坐标
        
        # 探索区域边界 (米)
        self.explore_bounds = {
            'X_MIN': 11.0,
            'X_MAX': 22.0,
            'Y_MIN': 2.0,
            'Y_MAX': 21.0
        }

    def load_image(self, filepath):
        """从文件加载图像"""
        try:
            image = Image.open(filepath)
            image_array = np.array(image)
            print(f"Loaded image: {filepath}, shape: {image_array.shape}")
            
            # 如果黑色是0，则反转图像
            if np.median(image_array) > 127:
                image_array = 255 - image_array
                print("Image inverted for obstacle detection.")
            
            return image_array
        except Exception as e:
            print(f"Error loading image: {e}")
            return None

    def preprocess_image(self, image_array):
        """对图像进行预处理"""
        # 动态调整阈值
        _, binary_image = cv2.threshold(image_array, self.obstacle_threshold, 255, cv2.THRESH_BINARY)
        
        # 形态学操作：膨胀和腐蚀
        kernel = np.ones((3, 3), np.uint8)
        binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
        
        return binary_image

    def detect_boxes(self, binary_image):
        """从二值化图像中检测盒子位置"""
        # 查找轮廓
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        box_positions = []
        for contour in contours:
            # 计算轮廓面积，过滤掉小区域
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            # 计算轮廓的中心点
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])
            
            # 转换为地图坐标
            map_x = center_x * self.resolution + self.origin_x
            map_y = (binary_image.shape[0] - center_y) * self.resolution + self.origin_y  # 注意Y轴方向
            
            # 检查是否在探索区域内
            if (self.explore_bounds['X_MIN'] <= map_x <= self.explore_bounds['X_MAX'] and
                self.explore_bounds['Y_MIN'] <= map_y <= self.explore_bounds['Y_MAX']):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = map_x
                pose.pose.position.y = map_y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                box_positions.append(pose)
                print(f"Detected box at: x={map_x:.2f}, y={map_y:.2f}, area={area}")
        
        return box_positions

    def visualize_results(self, image_array, binary_image, box_positions):
        """可视化检测结果"""
        plt.figure(figsize=(10, 8))
        plt.imshow(image_array, cmap='gray', interpolation='nearest')
        
        for box in box_positions:
            pixel_x = int((box.pose.position.x - self.origin_x) / self.resolution)
            pixel_y = int(image_array.shape[0] - (box.pose.position.y - self.origin_y) / self.resolution)
            plt.plot(pixel_x, pixel_y, 'ro', markersize=10)
            plt.text(pixel_x + 5, pixel_y, f"({box.pose.position.x:.2f}, {box.pose.position.y:.2f})", color='red')
        
        plt.title("Detected Boxes")
        plt.savefig("/tmp/detected_boxes.png")
        plt.show()

def main():
    detector = BoxDetector()
    
    # 加载图像
    filepath = "../maps/global_costmap_with_boxes.png"  # 替换为你的文件路径
    image_array = detector.load_image(filepath)
    if image_array is None:
        return
    
    # 图像预处理
    binary_image = detector.preprocess_image(image_array)
    
    # 检测盒子
    box_positions = detector.detect_boxes(binary_image)
    
    # 可视化结果
    detector.visualize_results(image_array, binary_image, box_positions)
    print(f"Total boxes detected: {len(box_positions)}")

if __name__ == "__main__":
    main()