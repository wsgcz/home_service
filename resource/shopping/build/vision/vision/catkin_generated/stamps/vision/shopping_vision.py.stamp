#！ /usr/bin/env python3

import rospy
import numpy as np
import torch
import cv2 as cv
import json
from typing import Any
from time import perf_counter
from ultralytics import YOLO
from cv_bridge import CvBridge
from math import sqrt, pi
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from abc import ABC, abstractmethod

# ************************************ 相机参数已经不适用了 *************************************************************

class Vision(ABC):

    # 速度控制
    vel_cmd:Twist = Twist()
    vel_cmd.linear.z, vel_cmd.angular.x, vel_cmd.angular.y = 0, 0, 0
    vel_pub = rospy.Publisher(
        name='/cmd_vel',
        data_class=Twist,
        queue_size=10
    )

    __is_exec:bool = False
    lost_rotate_direction = -1
    cvbridge:CvBridge = CvBridge()

    def __init__(self, 
                yolo_path:str, 
                vision_show_topic:str, 
                vision_finish_topic:str,
                confidence:float=0.5, 
                max_wait_time:float=3.0, 
                detect_class:int=0
                ) -> None:

        self.__conf = confidence

        self.__model = self.load_model(yolo_path)

        self.vision_timer = VisionTimer(max_wait_time)
        # self.__max_wait_time = max_wait_time
        
        self.__class = detect_class

        self.get_camera_para() 

        # 图像展示
        self.drawed_image = None
        self.info_dict:dict = dict()
        self.img_pub = rospy.Publisher(
            name=vision_show_topic,
            data_class=Image,
            queue_size=10
        )
        # 模块运行结束要通知其他结点
        self.is_finish:Bool = Bool()
        self.is_finish.data = False
        self.finish_pub = rospy.Publisher(
            name=vision_finish_topic,
            data_class=Bool,
            queue_size=10
        )

    def __call__(self, row_rgb_image:Any, row_depth_image:Any) -> Any:

        self.info_dict.clear()
        self.info_dict['start detect'] = self.__is_exec

        self.reset_vel() # 速度清零

        if self.__is_exec:

            rgb, depth = self.image_processing(row_rgb_image, row_depth_image)

            # 检测模块
            t1 = rospy.Time.now().nsecs
            detect_result = self.detect(rgb)
            detect_result = self.detect_result_postprocessing(detect_result) # 根据不同的任务对结果要有不同的预处理
            t2 = rospy.Time.now().nsecs
            # rospy.logwarn("detect time: {}".format(t2 - t1))

            # 决策模块
            t1 = rospy.Time.now().nsecs
            target = self.decide(
                rgb_image=rgb,
                depth_image=depth,
                detect_result=detect_result
            ) 
            t2 = rospy.Time.now().nsecs
            # rospy.logwarn("decide time: {}".format(t2 - t1))

            # 运动模块
            t1 = rospy.Time.now().nsecs
            if target is None:
                lost_time = self.vision_timer(self.deal_lost)
                self.info_dict['lost time'] = round(lost_time, 2)
            else:
                cv.circle(
                    img=self.drawed_image,
                    center=target,
                    radius=10,
                    color=(0, 0, 0),
                    thickness=2
                )
                self.vision_timer.time(reset=True)
                # 运动模块
                self.move(target, rgb, depth)

            self.info_dict['robot velocity'] = (
                round(self.vel_cmd.linear.x, 2), 
                round(self.vel_cmd.linear.y, 2), 
                round(self.vel_cmd.angular.z, 2)
            )
        
            self.image_show()
            self.vel_pub.publish(self.vel_cmd)

            t2 = rospy.Time.now().nsecs
            # rospy.logwarn("motion time: {}".format(t2 - t1))
            # rospy.logwarn("\n")

    def load_model(self, yolo_path) -> YOLO:
        return YOLO(
            model=yolo_path
        ) # 加载模型

    def detect(self, rgb_image) -> Any:

        result = self.__model.predict(
            source=rgb_image,
            conf=self.__conf,
            verbose=False,
            classes=self.__class,
            half=True
        )[0]
        self.drawed_image = result.plot()

        return result

    def decide(self, rgb_image, depth_image, detect_result) -> Any:
        '''
        根据检测结果，决定目标物体
        return:
            若返回值为None 则没有检测到物体，或检测到的不符合要求，丢失目标
        '''
        if len(detect_result) == 0:
            # 没有检测到物体
            target = None
        else:
            # 检测到多个物体
            target = self.select_objects(detect_result, depth_image)
        
        return target
    
    def select_objects(self, detect_result, depth_image) -> Any:
        '''
        当机器人视野内有多个物体时，从中挑选出一个最适合的
        默认情况下，选择最近的物体
        '''
        xys = self.get_xys(detect_result)
        xys_avg = torch.sum(xys, dim=1) / xys.shape[1]

        xys_avg = xys_avg.cpu().numpy().astype("int32")
        classes_detected = detect_result.cls.cpu().numpy().astype("int32")

        dists = depth_image[xys_avg[:, 1], xys_avg[:, 0]] * self.depth_scale

        min_dist_idx = np.argmin(dists)
        self.target_class = classes_detected[min_dist_idx] # 存储目标的类别

        return xys_avg[min_dist_idx]
    
    def get_object_position(self, target, depth_image) -> tuple:
        '''
        根据目标的像素坐标系坐标，
        返回要抓取的物体在相机坐标系下的坐标
        '''
        u, v = int(target[0]), int(target[1])
        center_depth = depth_image[v, u] * self.depth_scale
        Xc, Yc, Zc = self.real_pose(u, v, center_depth)

        self.info_dict["target coordinates in pixel frame"] = (u, v)
        cv.circle(self.drawed_image, center=(u, v), radius=5, color=(0,0,0))

        return (Xc, Yc, Zc)
    
    def reset_vel(self) -> None:

        self.vel_cmd.linear.x = 0
        self.vel_cmd.linear.y = 0
        self.vel_cmd.angular.z = 0

    def image_show(self) -> None:

        with open('/home/gcz/shopping/src/vision/vision/vision_info.json', 'w') as json_file:
            json.dump(self.info_dict, json_file)
        
        self.img_pub.publish(
            self.cvbridge.cv2_to_imgmsg(self.drawed_image)
        )
    
    def deal_lost(self) -> None:
        '''
        机器人确认已经丢失目标后的处理函数
        '''
        self.vel_cmd.angular.z = self.lost_rotate_direction * pi/180 * 12
        for i in range(3):
            self.vel_pub.publish(self.vel_cmd)

    def switch(self, p) -> None:
        '''
        整个视觉程序的开关
        '''
        if self.__is_exec != p.data:
            self.state_switch()
            self.is_finish.data = False

        self.__is_exec = p.data

    # ********************************************** 以下为自定义函数 ********************************
    def image_processing(self, row_rgb_image:Any, row_depth_image:Any) -> tuple:
        '''
        处理第一手获取到的图像数据
        并将其转换为cv.mat

        默认情况下, 将imgmsg 转换为 cv.mat
        '''
        rgb = self.cvbridge.imgmsg_to_cv2(row_rgb_image)
        depth = self.cvbridge.imgmsg_to_cv2(row_depth_image)

        return rgb, depth

    @abstractmethod
    def get_xys(self, detect_result):
        '''
        获得像素坐标系下检测点的坐标
        返回的array shape = (object number, point number, 2/3)
        '''
        pass

    @abstractmethod
    def detect_result_postprocessing(self,result) -> Any:
        '''
        根据视觉任务是识别、分割还是其他而各有不同
        '''
        pass

    @abstractmethod
    def state_switch(self) -> Any:
        '''
        状态切换时执行
        '''
        pass
        
    @abstractmethod
    def move(self, target, rgb_image, depth_image) -> None:
        '''
        根据检测到的target来决定接下来的运动情况
        默认情况下，机器人原地不动
        '''    
        pass


    # **************************** camera related method (need be rewrited) **************************************
    def get_camera_para(self):

        try:
            self.f_x = 540.68603515625
            self.f_y = 540.68603515625
            self.P_x = 479.75
            self.P_y = 269.75
            self.depth_scale = 0.001
        except KeyError:
            rospy.logfatal("Vision didn't get camera parameter")

    def real_pose(self, u_img, v_img, d):
        '''
        根据图像上的坐标获取其相对机器人的真实坐标
        '''
        z = d / sqrt( 1 + (u_img-self.P_x)**2/self.f_x**2 + (v_img-self.P_y)**2/self.f_y**2 )
        x = (u_img-self.P_x)*z/self.f_x
        y = (v_img-self.P_y)*z/self.f_y

        return x, y, z

class VisionTimer:

    __lost_start = 0
    __lost_time = 0

    def __init__(self, max_wait_time) -> None:
        self.__max_wait_time = max_wait_time

    def __call__(self, func, wait_time=None) -> float:
        # 未检测到物体，开始计时

        if self.__lost_start == 0:
            self.time(init=True)

        if not self.time(wait_time):
            # 超时，启动超时处理程序
            if func != None:
                func()
        
        return round(self.lost_time, 1)

    def time(self, wait_time=None, reset:bool=False, init:bool=False) -> bool:

        if reset and init:
            raise ValueError("reset and init can't be True in the same time")

        if reset:
            self.__lost_start = 0
            self.__lost_time = 0
            return True

        elif init:
            self.__lost_start = perf_counter()
            return True
        
        if wait_time is None:
            return self.lost_time < self.__max_wait_time
        else:
            return self.lost_time < wait_time

    @property
    def lost_time(self) -> float:
        return self.__lost_time
    
    @lost_time.getter
    def lost_time(self) -> float:
        return perf_counter() - self.__lost_start