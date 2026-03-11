#!/usr/bin/env python3
"""
水下机器人视觉定位节点 (Visual Localization relative to USV)
绕过 ROS Bridge，直接从 Gazebo transport 抓取原生图像。
使用 OpenCV ArUco 检测目标，并解算出全局位姿送到 EKF。
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image

# OpenCV ArUco 常量字典 (视 opencv 版本而定)
try:
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
except AttributeError:
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# USV_ARUCO 的实际边长
MARKER_SIZE_METERS = 0.5

# 假设相机的内参 (我们按 SDF 模型中的 1.5 FOV 和 640x480 分辨率逆推)
FOV = 1.5
IMG_W = 640
IMG_H = 480
FL = (IMG_W / 2.0) / math.tan(FOV / 2.0)
CAMERA_MATRIX = np.array([
    [FL,   0.0, IMG_W / 2.0],
    [0.0,   FL, IMG_H / 2.0],
    [0.0,  0.0,         1.0]
], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1))

class VisualEKFNode(Node):
    def __init__(self):
        super().__init__('visual_ekf_node')
        
        # 将视觉定位结果发给 EKF 节点
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/sensor/aruco_pose', 10)
        
        # 订阅 Gazebo 原生图像
        self._gz_node = GzNode()
        self._gz_node.subscribe(Image, '/camera_up/image', self._gz_image_cb)
        
        # 初始化一个用于显示的独立窗口线程，方便我们看效果
        cv2.namedWindow("ROV Upward View", cv2.WINDOW_AUTOSIZE)
        # 用 timer 机制防止 cv2.imshow 卡死
        self.create_timer(0.05, self._cv_refresh_cb)
        self.current_frame = None
        
        self.get_logger().info("🔥 视觉定位节点启动中...正在扫描上方水面 ArUco")

    def _cv_refresh_cb(self):
        if self.current_frame is not None:
            cv2.imshow("ROV Upward View", self.current_frame)
            cv2.waitKey(1)

    def _gz_image_cb(self, msg: Image):
        """处理 Gazebo 传来的二进制 protobuf 图像"""
        # Gazebo的图像通常是 RGB8格式
        np_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        
        # 将 RGB 转换为 BGR 给 OpenCV 使用
        frame = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ArUco 检测
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        if ids is not None:
            # 画框供我们调试可视化
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 位姿估计 (输出旋转向量、平移向量)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_METERS, CAMERA_MATRIX, DIST_COEFFS)
            
            for i in range(len(ids)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]  # [x, y, z] 的相机坐标系偏移
                
                # 画出坐标轴 (X红, Y绿, Z蓝)
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, 0.25)
                
                # --- [神之一手：坐标系大逆转] ---
                # 相机的 Z轴 指向天空 (识别到的目标高度), X向右, Y向下
                # 我们要知道 ROV 相对于 USV (0,0,0) 的投影位置
                cam_z = tvec[2]  # ROV 到 USV 的高度差
                cam_x = tvec[0]
                cam_y = tvec[1]
                
                # 通过高度动态调整 EKF 信任度（协方差）
                # 水面(Z小)图像极其清晰，深水(Z大)图像模糊
                base_cov = 0.01 + (cam_z * 0.02) ** 2 
                
                # 发送给 ROS 2 EKF
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'odom'
                
                # 我们假设 USV 在原点, 我们算出了相机看 USV 的相对方向
                # 为了简化计算，我们这里的偏移做粗略解算发送给 X, Y 进行强制拉扯
                pose_msg.pose.pose.position.x = -cam_y  # 匹配 NED 的北方
                pose_msg.pose.pose.position.y = cam_x   # 匹配 NED 的东方
                
                # 给极小的方差，逼 EKF 去相信它！
                pose_msg.pose.covariance[0] = base_cov     # X方差
                pose_msg.pose.covariance[7] = base_cov     # Y方差
                
                self.pub_pose.publish(pose_msg)
                
                # 打印到终端看它是否抓到了！
                cv_text = f"USV Locked! Z:{cam_z:.1f}m, dX:{-cam_y:.2f}m, dY:{cam_x:.2f}m"
                cv2.putText(frame, cv_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Searching USV...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 保存为实例变量给 imshow
        self.current_frame = frame

def main(args=None):
    rclpy.init(args=args)
    node = VisualEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()