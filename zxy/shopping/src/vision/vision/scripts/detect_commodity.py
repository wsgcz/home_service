#! /usr/bin/env python3

import rospy
import numpy as np
import torch
import cv2 as cv
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float64MultiArray
from math import pi, sqrt, cos, sin
import sys
sys.path.insert(0, "/home/gcz/shopping/src/vision/vision/scripts")
from shopping_vision import Vision, VisionTimer

class DetectCommodity(Vision):

    # RealSense parameter
    image_width = 640
    image_height = 480
    image_fps = 6

    # 检测时间计时器
    detect_timer = VisionTimer(max_wait_time=10) # 每次检测的时间为5s

    # 物品名称列表
    name_list = [
        'biscuit',
        'chip', 
        'lays',
        'bread',
        'cookie',
        'handwash',
        'dishsoap',
        'water',
        'sprite',
        'cola',
        'orange juice'
    ]
    # 投票列表
    name_vote_list = np.zeros(11) # 11为物品种类数

    # 语音播报publisher
    speak_str = String()
    voice_pub = rospy.Publisher(
        name="/ourspeak",
        data_class=String,
        queue_size=1
    )
    
    def __init__(self) -> None:
        self.camera_connect()
        super().__init__(
            yolo_path='/home/gcz/shopping/model/models/best.pt',
            vision_show_topic="detect_vision_show",
            vision_finish_topic="detect_finish",
            detect_class=range(11),
            confidence=0.8,
            max_wait_time=20
        )

    def __call__(self) -> None:

        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() 
        color_frame = aligned_frames.get_color_frame()

        if color_frame and aligned_depth_frame:
            super().__call__(color_frame, aligned_depth_frame)

    def image_processing(self, raw_rgb_image, raw_depth_image) -> None:

        depth_map = np.asanyarray(raw_depth_image.get_data())*self.depth_scale
        bgr_img = np.asanyarray(raw_rgb_image.get_data())
        rgb_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2RGB)

        return rgb_img, depth_map

    def detect_result_postprocessing(self,result):
        '''
        根据视觉任务是识别、分割还是其他而各有不同
        '''
        boxes = result.boxes

        return boxes

    def get_xys(self, detect_result) -> torch.Tensor:
        '''
        将检测到的数据整理成合适的格式
        '''
        xyxys = detect_result.xyxy
        xys = torch.reshape(input=xyxys, shape=(xyxys.shape[0], 2, 2))
        return xys

    def move(self, target, rgb_image, depth_image) -> None:
        '''
        检测阶段,机器人底盘移动,将物体置于realsense视野中心
        '''
        u, v = target
        self.name_vote_list[self.target_class] += 1

        if np.abs(u - self.image_width/2) > 50:
            self.vel_cmd.linear.y = 0.1 * 0.002 * (self.image_width/2 - u)
        if np.abs(v - self.image_height/2) > 50:
            self.vel_cmd.linear.x = 0.1 * 0.002 * (self.image_height/2 - v)
        if np.abs(u - self.image_width/2) < 50 and np.abs(v - self.image_height/2) < 50:
            if not self.is_finish.data:
                self.speak()
                self.is_finish.data = True
            else:
                self.finish_pub.publish(self.is_finish)
                rospy.loginfo(f"************ finish publish {self.is_finish}")

    def speak(self) -> None:
        '''
        语音播报检测到的物体
        '''
        min_commodity_index = np.argmax(self.name_vote_list)
        self.name_vote_list = np.zeros(11) # 将投票列表清空，便于下一个物品的检测
        rospy.loginfo(f"min_commodity_index: {min_commodity_index}")
        commodity_name = self.name_list[min_commodity_index]

        self.speak_str.data = "I find " + commodity_name

        for i in range(3):
            self.voice_pub.publish(self.speak_str)
        
        if commodity_name == "orange juice":
            # 参数服务器无法命名中带有空格
            rospy.set_param("item_name", "orangejuice")
        else:
            rospy.set_param("item_name", commodity_name)
    
    def state_switch(self) -> None:
        pass
    
    # ******************************* camera related method (need to be rewrited) *****************************************
    def camera_connect(self):
        # Start and configure
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, self.image_width, self.image_height, rs.format.z16, self.image_fps)
        config.enable_stream(rs.stream.color, self.image_width, self.image_height, rs.format.rgb8, self.image_fps)

        cfg = self.pipeline.start(config)

        # Determine intrinsics
        rgb_profile = cfg.get_stream(rs.stream.color)
        self.intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()

        # Determine depth scale
        self.depth_scale = cfg.get_device().first_depth_sensor().get_depth_scale()

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
    
    def get_camera_para(self):
        
        self.f_x = self.intrinsics.fx
        self.f_y = self.intrinsics.fy
        self.P_x = self.intrinsics.ppx
        self.P_y = self.intrinsics.ppy
        
        # get intrinsic matrix
        self.intrinsic_matrix = np.array([
            [self.f_x ,   0.     , self.P_x ,   0.     ],
            [  0.     , self.f_y , self.P_y ,   0.     ],
            [  0.     ,   0.     ,   1.     ,   0.     ],
            [  0.     ,   0.     ,   0.     ,   1.     ]
        ])
        self.intrinsic_matrix_inv = np.linalg.inv(self.intrinsic_matrix)        

    def depth_to_Zc(self, u, v, depth) -> float:

        return round(depth, 3)

    def pixel2camera(self, u, v, depth) -> tuple:
        Zc = self.depth_to_Zc(u, v, depth)
        Xc, Yc, _, _= self.intrinsic_matrix_inv @ np.array([u, v, 1, 1]) * Zc

        return np.around(
            a=np.array([Xc, Yc, Zc]),
            decimals=2
        )
    
    def world_coordinate_frame(self, Xc, Yc, Zc) -> tuple:
        Xw, Yw, Zw, _ = self.extrinsic_matrix_inv @ np.array([Xc, Yc, Zc, 1])
        return np.around(
            a=np.array([Xw, Yw, Zw]),
            decimals=2
        )
    
    def deal_lost(self):
        # self.vel_cmd.linear.x = 0.05
        pass

import threading
class switchThread(threading.Thread):

    def __init__(self, vision:Vision, switch_topic:str) -> None:

        super().__init__()

        easy_grasp_sub = rospy.Subscriber(
            name=switch_topic,
            data_class=Bool,
            callback=vision.switch,
            queue_size=10
        )

    def run(self) -> None:
        rospy.spin()

class visionThread(threading.Thread):

    def __init__(self, vision) -> None:

        super().__init__()

        self.vision = vision

    def run(self) -> None:

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            r.sleep()

            self.vision()

if __name__=='__main__':
    
    rospy.init_node('detect_commodity')

    detect_commodity = DetectCommodity()

    switch_thread = switchThread(detect_commodity, 'detect_switch')
    vision_thread = visionThread(detect_commodity)

    switch_thread.start()
    vision_thread.start()