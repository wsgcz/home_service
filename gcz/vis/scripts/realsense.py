# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from math import atan2,pi,sqrt,sin,cos
import actionlib
import tf2_ros
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
 
''' 
设置
'''
width = 1040
height = 560

pipeline = rs.pipeline()  # 定义流程pipeline，创建一个管道
config = rs.config()  # 定义配置config

config.enable_stream(rs.stream.depth,  1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
 
pipe_profile = pipeline.start(config)  # streaming流开始
 
# 创建对齐对象与color流对齐
align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)  # rs.align 执行深度帧与其他帧的对齐

frame = 0

def rpy2quaternion(roll, pitch, yaw):
    x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    return [x, y, z, w]
 
 
def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧，获取颜色和深度的框架集
    aligned_frames = align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
 
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
    aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧
 
    #### 获取相机参数 ####
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
 
    #### 将images转为numpy arrays ####
    img_color = np.asanyarray(aligned_color_frame.get_data())  # RGB图
    img_depth = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
 
    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame
 
 
''' 
获取随机点三维坐标
'''
 
def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
    # print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return dis, camera_coordinate
 
 
if __name__ == "__main__":
    rospy.init_node("waypoint")
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    #while (not ac.wait_for_server(rospy.Duration(5))):
        #rospy.loginfo("wait for server to come up")

    if (frame == 0):
        ''' 
        获取对齐图像帧与相机参数
        '''
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()  # 获取对齐图像与相机参数
 
        ''' 
        获取随机点三维坐标
        '''
        depth_pixel = [width, height]  # 设置随机点，以相机中心点为例
        dis, camera_coordinate = get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)
        print('depth: ', dis)  # 深度单位是mm
        print('camera_coordinate: ', camera_coordinate)
 
        ''' 
        显示图像与标注
        '''
        #### 在图中标记随机点及其坐标 ####
        cv2.circle(img_color, (width, height), 8, [255, 0, 255], thickness=-1)
        cv2.putText(img_color, "Dis:" + str(dis) + " m", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, [0, 0, 255])
        cv2.putText(img_color, "X:" + str(camera_coordinate[0]) + " m", (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        cv2.putText(img_color, "Y:" + str(camera_coordinate[1]) + " m", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])
        cv2.putText(img_color, "Z:" + str(camera_coordinate[2]) + " m", (80, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    [255, 0, 0])

        point_camera = PointStamped()
        point_camera.header.frame_id = "camera_link"
        point_camera.header.stamp = rospy.Time.now()
        point_camera.point.x = camera_coordinate[0]
        point_camera.point.y = camera_coordinate[1]
        point_camera.point.z = camera_coordinate[2]
        try:
    #     5.调研订阅对象的 API 将 4 中的点坐标转换成相对于 base_footprint 的坐标
            point_target = buffer.transform(point_camera,"base_footprint")
            rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                            point_target.point.x,
                            point_target.point.y,
                            point_target.point.z)
        except Exception as e:
            rospy.logerr("异常:%s",e)

        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = 'base_footprint'
        # goal.target_pose.pose.position.x = point_target.point.x - 1
        # goal.target_pose.pose.position.y = point_target.point.y
        # goal.target_pose.pose.position.z = 0
        # xyzw = rpy2quaternion(0.0, 0.0, 0.0)
        # goal.target_pose.pose.orientation.x = xyzw[0]
        # goal.target_pose.pose.orientation.y = xyzw[1]
        # goal.target_pose.pose.orientation.z = xyzw[2]
        # goal.target_pose.pose.orientation.w = xyzw[3]
        # goal.target_pose.header.stamp = rospy.Time.now()
        # print("----------i will send a goal----------")
        # ac.send_goal(goal)

        #### 显示画面 ####
        cv2.imshow('RealSence', img_color)
        frame = 1
        key = cv2.waitKey(0)