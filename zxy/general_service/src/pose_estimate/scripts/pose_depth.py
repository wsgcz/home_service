# from ctypes.wintypes import tagMSG
from importlib.resources import path
# from itertools import count
import queue
from turtle import width
import tf2_ros
from inspect import stack
import re
from select import select
# import matplotlib.pyplot as plt
import torch
import cv2
from torchvision import transforms
import numpy as np
import sys
sys.path.insert(0,'/home/linxi/ServiceRobot-General/src/pose_estimate')
sys.path.insert(1,'/home/linxi/ServiceRobot-General/src/pose_estimate/scripts')
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts,plot_one_box
from face_detect import FaceRecognition
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist
import time
import os
import threading
from tf2_geometry_msgs import PointStamped,PoseStamped
from queue import Queue
# 加载模型
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import message_filters
import mediapipe as mp
import rospy
from math import atan2,pi,sin,cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
bridge = CvBridge()
count=0
def image_callback(image_rgb,image_depth):
    # global count
    # if count<=100:
    #     count+=1
    #     pass
    # else:
        image = bridge.imgmsg_to_cv2(
                image_rgb, desired_encoding='passthrough')
        depth = bridge.imgmsg_to_cv2(
                image_depth, desired_encoding='passthrough')
        # print(np.sum(depth!=0))
        cv2.imshow("thisone ",image)
        cv2.waitKey(1)
    # print(111111111111)q
if __name__=="__main__":
    # time.sleep(4)
    rospy.init_node("pose")
    rgb_sub = message_filters.Subscriber('/kinect2/hd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/hd/image_depth_rect', Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)
    tfBuffer = tf2_ros.Buffer    
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # task2()

    rospy.spin()
    
 
 
 