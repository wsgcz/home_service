#! /usr/bin/env python3

import rospy
import sys
import numpy as np
import torch
from math import pi, atan, sqrt
from time import perf_counter
from std_msgs.msg import Bool
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from vision.srv import LeaderDist, LeaderDistResponse
sys.path.insert(0, "/home/gcz/shopping/src/vision/vision/scripts")
from shopping_vision import Vision

class Follow(Vision):

    # 跟随模块控制参数
    __history_direction = 1
    __max_dist = 5
    __min_dist = 0.7

    leader_dist = -1

    def __init__(self):

        super().__init__(
            yolo_path='/home/gcz/shopping/model/models/yolov8n.pt', 
            vision_show_topic="follow_vision_show",
            vision_finish_topic="follow_finish",
            confidence=0.6,
            max_wait_time=3
            )
        
    def detect_result_postprocessing(self, result):
        return result.boxes


    def get_xys(self, detect_result) -> torch.Tensor:

        xyxys = detect_result.xyxy
        xys = torch.reshape(input=xyxys, shape=(xyxys.shape[0], 2, 2))
        return xys
    
    def select_objects(self, detect_result, depth_image):
        '''
        当机器人视野内有多个物体时，从中挑选出一个最适合的
        选择检测框最大的人
        '''
        xys = self.get_xys(detect_result)
        xys_avg = torch.sum(xys, dim=1) / xys.shape[1]

        xy_diff = xys[:, 0, :] - xys[:, 1, :]
        areas = xy_diff[:, 0] * xy_diff[:, 1]

        max_area_idx = torch.argmax(areas)

        target = xys_avg[max_area_idx].cpu().numpy().astype("int32")

        self.leader_dist = depth_image[target[1], target[0]] * self.depth_scale # 记录leader的距离

        return target
    
    def state_switch(self) -> None:
        '''
        跟随模块只能处于两个状态：休眠或启动
        在这两种模式切换时，要求机器人停止
        '''
        self.reset_vel()
        for i in range(3):
            # 让机器人停止运动,连续发三次,防止通信异常
            self.vel_pub.publish(self.vel_cmd)

    def move(self, target, rgb_image, depth_image) -> None:
        '''
        根据检测到的target来决定接下来的运动情况

        默认情况下，机器人原地不动
        '''
        Xc, Yc, Zc = self.get_object_position(target, depth_image)

        self.update_history_direction(target) # 更新历史方向的记录，以防丢失目标

        self.info_dict['is following'] = True if Zc < self.__max_dist and Zc > self.__min_dist else False

        self.follow_vel(Xc, Yc, Zc)
    
    # ************* 以下为Follow自定义方法 *****************************

    def return_leader_dist(self, request):

        rospy.loginfo(f"leader dist: {self.leader_dist}")

        resp = LeaderDistResponse(distance=self.leader_dist)

        return resp
    
    def update_history_direction(self, target) -> None:

        u, _ = target
        self.__history_direction = 1 if u > self.P_x else 0
        self.lost_rotate_direction = self.__history_direction
    
    def follow_vel(self, X, Y, Z) -> None:

        FOLLOW_DIST = 0.5 # 跟随距离，单位：米（m）
        LINEAR_RATE = 0.4 
        ANGLE_RATE = 1.4 # 真实旋转速度比例
        MAX_LINEAR = 0.5
        MAX_ANGULAR = 2.5

        if Z > 0:
            human_x = Z
            human_y = -X
            angle = atan(human_y/human_x)

            flw_dist = sqrt(human_x**2 + human_y**2)
            diff_dist = flw_dist-FOLLOW_DIST

            self.info_dict['follow distance'] = str(round(flw_dist, 2)) + ' m'
            self.info_dict['difference distance'] = str(round(diff_dist, 2)) + ' m'
            self.info_dict['follow angle'] = str(round(angle, 2)) + ' rad'

            self.vel_cmd.angular.z = angle*ANGLE_RATE

            if self.vel_cmd.angular.z < -MAX_ANGULAR:
               self.vel_cmd.angular.z = -MAX_ANGULAR
            if self.vel_cmd.angular.z > MAX_ANGULAR:
               self.vel_cmd.angular.z = MAX_ANGULAR

            if flw_dist > self.__min_dist and flw_dist < self.__max_dist: 
                self.vel_cmd.linear.x = diff_dist*LINEAR_RATE
                if self.vel_cmd.linear.x < 0:
                    self.vel_cmd.linear.x *= 0.5

                if self.vel_cmd.linear.x < -MAX_LINEAR:
                    self.vel_cmd.linear.x = -MAX_LINEAR
                if self.vel_cmd.linear.x > MAX_LINEAR:
                    self.vel_cmd.linear.x = MAX_LINEAR


if __name__=='__main__':

    rospy.init_node(name="follow")

    follow = Follow()

    switch_sub = rospy.Subscriber(
        name='follow_switch',
        data_class=Bool,
        callback=follow.switch,
        queue_size=10
    )

    leader_dist_server = rospy.Service("/vision/LeaderDist", LeaderDist, follow.return_leader_dist)

    rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color", Image)
    depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(follow)

    if not rospy.is_shutdown():
        rospy.spin()