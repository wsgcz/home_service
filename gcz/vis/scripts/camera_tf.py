#! /usr/bin/env python
"""  
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 
    实现流程:
        1.导包
        2.初始化 ROS 节点
        3.创建 静态坐标广播器
        4.创建并组织被广播的消息
        5.广播器发送消息
        6.spin
"""
# 1.导包
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from math import atan2,pi,sqrt,sin,cos

def rpy2quaternion(roll, pitch, yaw):
    x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    return [x, y, z, w]

if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("static_camera_tf_pub")
    # 3.创建 静态坐标广播器
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # 4.创建并组织被广播的消息
    tfs = TransformStamped()
    # --- 头信息
    tfs.header.frame_id = "base_footprint"
    tfs.header.stamp = rospy.Time.now()
    tfs.header.seq = 101
    # --- 子坐标系
    tfs.child_frame_id = "camera_link"
    # --- 坐标系相对信息
    # ------ 偏移量
    tfs.transform.translation.x = 0.19
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.98
    # ------ 四元数
    qtn = rpy2quaternion(-1.92,0,-1.57)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]


    # 5.广播器发送消息
    broadcaster.sendTransform(tfs)
    # 6.spin
    rospy.spin()
