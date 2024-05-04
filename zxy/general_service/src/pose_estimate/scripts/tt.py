from ast import arg
import os
# from ctypes.wintypes import tagMSG
# from ctypes.wintypes import tagMSG
from importlib.resources import path
import queue
# from tokenize import String
from std_msgs.msg import String
from turtle import width
from xml.dom.expatbuilder import theDOMImplementation
import tf2_ros
from inspect import stack
import re
from select import select
import matplotlib.pyplot as plt
import torch
import cv2
from torchvision import transforms
import numpy as np
import sys
from nav_msgs.msg import Odometry
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
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

mutex1=threading.Lock()
mutex2=threading.Lock()
mutex_quene=threading.Lock()
now_flag=0
person_index_count=0
person_index=0
mediapipe_detect_count=0
all_work_count=0
empty_list=[]
empty_list_index=0
machine_odom_now_y=0
machine_odom_now_x=0
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=1,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)
# skeleton = mp.solutions.pose.skeleton()
# paralle_calculator = mp.solutions.drawing_utils.ParallelSkeletonDrawer(skeleton)
choose_face_or_pose_flag=0
face_recognitio = FaceRecognition()
loc_quene=Queue(10)
people_can_get_count=0
face_db_store_name=[]
path1='/home/linxi/ServiceRobot-General/src/pose_estimate/data_face'
name=['0.jpg','1.jpg','2.jpg']

def mm(now_path):
    global mutex1
    
    image=cv2.imread(now_path)
    mutex1.acquire()
    results=mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    mutex1.release()
    time.sleep(1)
    print(results.pose_world_landmarks.landmark[11])
    # result=paralle_calculator.process(image)
    print("now time pass is ",time.time()-t1)
t1=time.time()
for i in name:
    now_path=os.path.join(path1,i)
    task=threading.Thread(target=mm,args=(now_path,))
    task.start()
    
t2=time.time()
print("continue task time ", t2-t1)

    