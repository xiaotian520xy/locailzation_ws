#!/usr/bin/env python3
# coding=utf-8

import copy
import threading

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
try:
    import tf_transformations
except ImportError:
    from tf2_ros import transformations as tf_transformations

from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from scipy.ndimage import gaussian_filter1d

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        self.FREQ_PUB_LOCALIZATION = 50.0  # Hz

        # 滤波参数
        self.use_kalman_filter = True
        self.use_lowpass_filter = True
        
        # 低通滤波参数
        self.alpha = 0.1  # 滤波系数，越小越平滑
        self.filtered_pos = None
        self.filtered_quat = None

        self.lock = threading.Lock()
        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        # 구독자
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)

        # 퍼블리셔, 브로드캐스터
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        self.tf_broadcaster    = tf2_ros.TransformBroadcaster(self)

        # 주기 타이머
        period = 1.0 / self.FREQ_PUB_LOCALIZATION
        self.create_timer(period, self.timer_callback)
        self.get_logger().info('Transform Fusion Node Initialized')

    def cb_save_cur_odom(self, msg: Odometry):
        with self.lock:
            self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg: Odometry):
        with self.lock:
            self.cur_map_to_odom = msg

    def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot   = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        return trans @ rot


    
    def quaternion_slerp(self, q1, q2, t):
        """四元数球面线性插值"""
        # 确保四元数单位化
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        dot = np.dot(q1, q2)
        
        # 如果点积为负，取反以保证最短路径
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # 如果非常接近，直接线性插值
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # 计算角度和插值
        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        sin_theta_0 = np.sin(theta_0)
        
        s1 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s2 = sin_theta / sin_theta_0
        
        return s1 * q1 + s2 * q2

    def timer_callback(self):
        with self.lock:
            odom    = copy.deepcopy(self.cur_odom_to_baselink)
            map2odom = copy.deepcopy(self.cur_map_to_odom)
        if odom is None:
            return

        # map -> odom
        T_map_to_odom = self.pose_to_mat(map2odom) if map2odom is not None else np.eye(4)
        trans = tf_transformations.translation_from_matrix(T_map_to_odom)
        quat  = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        # tf 브로드캐스트
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id    = 'map'
        t_msg.child_frame_id     = 'camera_init'
        t_msg.transform.translation.x = trans[0]
        t_msg.transform.translation.y = trans[1]
        t_msg.transform.translation.z = trans[2]
        t_msg.transform.rotation.x    = quat[0]
        t_msg.transform.rotation.y    = quat[1]
        t_msg.transform.rotation.z    = quat[2]
        t_msg.transform.rotation.w    = quat[3]
        self.tf_broadcaster.sendTransform(t_msg)

        # fused localization
        T_odom_to_base = self.pose_to_mat(odom)
        T_map_to_base  = T_map_to_odom @ T_odom_to_base
        xyz   = tf_transformations.translation_from_matrix(T_map_to_base)
        quat2 = tf_transformations.quaternion_from_matrix(T_map_to_base)

        
        # 2. 低通滤波
        if self.use_lowpass_filter:
            if self.filtered_pos is None:
                self.filtered_pos = xyz
                self.filtered_quat = quat2
            else:
                self.filtered_pos = self.alpha * xyz + (1 - self.alpha) * self.filtered_pos
                # 四元数需要特殊处理
                self.filtered_quat = self.quaternion_slerp(self.filtered_quat, quat2, self.alpha)
            xyz = self.filtered_pos
            quat2 = self.filtered_quat

        loc_msg = Odometry()
        loc_msg.header.stamp          = odom.header.stamp
        loc_msg.header.frame_id       = 'map'
        loc_msg.child_frame_id        = 'body'
        loc_msg.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        loc_msg.pose.pose.orientation = Quaternion(
            x=quat2[0], y=quat2[1], z=quat2[2], w=quat2[3]
        )
        loc_msg.twist = odom.twist
        self.pub_localization.publish(loc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
