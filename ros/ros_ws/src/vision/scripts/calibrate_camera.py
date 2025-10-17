#!/usr/bin/env python
# autobot_vision/scripts/calibrate_camera.py
import rospy
import numpy as np
import cv2
import glob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraCalibration:
    def __init__(self):
        self.bridge = CvBridge()
        self.chessboard_size = (9, 6)  # 棋盘格内角点数量
        self.square_size = 0.024      # 棋盘格方格大小 (米)
        
        # 存储标定数据
        self.obj_points = []   # 3D点
        self.img_points = []   # 2D点
        
        # 准备对象点 (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1,2)
        self.objp *= self.square_size
        
    def calibrate_from_images(self, image_paths):
        """从图像文件进行标定"""
        for fname in image_paths:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            if ret:
                # 精确化角点位置
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                
                self.obj_points.append(self.objp)
                self.img_points.append(corners2)
                
                # 绘制并显示角点
                img = cv2.drawChessboardCorners(img, self.chessboard_size, corners2, ret)
                cv2.imshow('Calibration', img)
                cv2.waitKey(500)
        
        cv2.destroyAllWindows()
        
        # 执行相机标定
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, gray.shape[::-1], None, None)
        
        return ret, mtx, dist
    
    def save_calibration(self, mtx, dist, filename='camera_calibration.yaml'):
        """保存标定结果到文件"""
        calibration_data = {
            'camera_matrix': mtx.tolist(),
            'distortion_coefficients': dist.tolist(),
            'resolution': [640, 480]  # 假设的图像分辨率
        }
        
        import yaml
        with open(filename, 'w') as f:
            yaml.dump(calibration_data, f)
        
        print(f"Calibration saved to {filename}")
    
    def load_calibration(self, filename='camera_calibration.yaml'):
        """从文件加载标定结果"""
        import yaml
        with open(filename, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        
        mtx = np.array(data['camera_matrix'])
        dist = np.array(data['distortion_coefficients'])
        
        return mtx, dist

def main():
    # 查找所有标定图像
    images = glob.glob('/path/to/calibration/images/*.jpg')
    
    if not images:
        print("No calibration images found!")
        return
    
    calibrator = CameraCalibration()
    ret, mtx, dist = calibrator.calibrate_from_images(images)
    
    if ret:
        print("Calibration successful!")
        print("Camera matrix:")
        print(mtx)
        print("Distortion coefficients:")
        print(dist)
        
        # 保存标定结果
        calibrator.save_calibration(mtx, dist)
    else:
        print("Calibration failed!")

if __name__ == '__main__':
    main()