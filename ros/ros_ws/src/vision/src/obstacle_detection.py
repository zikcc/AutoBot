#!/usr/bin/env python
# autobot_vision/src/obstacle_dection.py
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped

class ObstacleDetection:
    def __init__(self):
        rospy.init_node('obstacle_detection', anonymous=True)
        
        # 参数配置
        self.min_distance = rospy.get_param('~min_distance', 0.3)  # 最小检测距离 (米)
        self.max_distance = rospy.get_param('~max_distance', 5.0)  # 最大检测距离 (米)
        self.safety_threshold = rospy.get_param('~safety_threshold', 0.8)  # 安全距离阈值
        
        # 初始化CV桥
        self.bridge = CvBridge()
        
        # 发布器和订阅器
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.obstacle_pub = rospy.Publisher('/obstacle/distance', LaserScan, queue_size=10)
        self.visualization_pub = rospy.Publisher('/obstacle/debug_image', Image, queue_size=10)
        self.nearest_obstacle_pub = rospy.Publisher('/obstacle/nearest', PointStamped, queue_size=10)
        
        # 状态变量
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        self.obstacle_direction = 0  # 障碍物方向 (-1: 左, 0: 中, 1: 右)
        
        rospy.loginfo("Obstacle Detection Node initialized")
        
    def depth_callback(self, msg):
        try:
            # 转换深度图像为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # 处理深度数据
            self.process_depth_data(depth_image)
            
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error: %s", e)
    
    def process_depth_data(self, depth_image):
        # 有效距离过滤（去除NaN和无穷大值）
        valid_depth = np.nan_to_num(depth_image, nan=self.max_distance, posinf=self.max_distance)
        valid_depth = np.clip(valid_depth, self.min_distance, self.max_distance)
        
        # 创建障碍物扫描数据（模拟激光雷达）
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "camera_depth_frame"
        
        # 扫描参数设置
        scan_msg.angle_min = -1.57  # -90度
        scan_msg.angle_max = 1.57   # +90度
        scan_msg.angle_increment = 0.1  # 角度增量
        scan_msg.range_min = self.min_distance
        scan_msg.range_max = self.max_distance
        
        # 将深度图像转换为激光扫描数据
        height, width = valid_depth.shape
        scan_ranges = []
        
        # 将图像分成多个扇形区域进行扫描
        num_sectors = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        
        for i in range(num_sectors):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            sector_min = max(0, int(width/2 * (1 + np.tan(angle) * 0.5) - 10))
            sector_max = min(width, int(width/2 * (1 + np.tan(angle) * 0.5) + 10))
            
            if sector_min < sector_max:
                sector_data = valid_depth[:, sector_min:sector_max]
                min_distance_in_sector = np.min(sector_data) if np.any(sector_data > 0) else self.max_distance
            else:
                min_distance_in_sector = self.max_distance
            
            scan_ranges.append(float(min_distance_in_sector))
        
        scan_msg.ranges = scan_ranges
        self.obstacle_pub.publish(scan_msg)
        
        # 检测最近障碍物
        self.detect_nearest_obstacle(valid_depth)
        
        # 创建可视化图像（用于调试）
        self.create_debug_image(valid_depth, scan_ranges)
    
    def detect_nearest_obstacle(self, depth_image):
        # 找到图像中的最小距离（最近障碍物）
        min_val = np.min(depth_image)
        
        if min_val < self.safety_threshold:
            self.obstacle_detected = True
            self.min_obstacle_distance = min_val
            
            # 找到最小距离的位置
            min_loc = np.where(depth_image == min_val)
            if len(min_loc[0]) > 0 and len(min_loc[1]) > 0:
                y, x = min_loc[0][0], min_loc[1][0]
                height, width = depth_image.shape
                
                # 计算障碍物方向（相对于图像中心）
                center_x = width // 2
                self.obstacle_direction = (x - center_x) / center_x
                
                # 发布最近障碍物信息
                obstacle_msg = PointStamped()
                obstacle_msg.header.stamp = rospy.Time.now()
                obstacle_msg.header.frame_id = "camera_depth_frame"
                obstacle_msg.point.x = min_val
                obstacle_msg.point.y = (x - center_x) / 100.0  # 归一化坐标
                obstacle_msg.point.z = 0.0
                self.nearest_obstacle_pub.publish(obstacle_msg)
                
                rospy.loginfo_throttle(1.0, "Obstacle detected at %.2fm, direction: %.2f", 
                                      min_val, self.obstacle_direction)
        else:
            self.obstacle_detected = False
            self.min_obstacle_distance = float('inf')
    
    def create_debug_image(self, depth_image, scan_ranges):
        # 创建彩色可视化图像
        depth_colored = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_colored = np.uint8(depth_colored)
        depth_colored = cv2.applyColorMap(depth_colored, cv2.COLORMAP_JET)
        
        # 标记最近障碍物
        if self.obstacle_detected:
            min_loc = np.where(depth_image == self.min_obstacle_distance)
            if len(min_loc[0]) > 0 and len(min_loc[1]) > 0:
                y, x = min_loc[0][0], min_loc[1][0]
                cv2.circle(depth_colored, (x, y), 10, (0, 0, 255), -1)
                cv2.putText(depth_colored, f"{self.min_obstacle_distance:.2f}m", 
                           (x+15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 添加安全距离阈值线
        height, width = depth_colored.shape[:2]
        safe_pixels = int((self.safety_threshold - self.min_distance) / 
                         (self.max_distance - self.min_distance) * 255)
        cv2.line(depth_colored, (0, safe_pixels), (width, safe_pixels), (0, 255, 0), 2)
        
        # 发布可视化图像
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(depth_colored, "bgr8")
            debug_msg.header.stamp = rospy.Time.now()
            self.visualization_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr("Failed to convert debug image: %s", e)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ObstacleDetection()
        detector.run()
    except rospy.ROSInterruptException:
        pass