# from ast import main
# from importlib.resources import path
# from msilib.schema import Class
import queue
from tkinter.messagebox import YESNO
from turtle import width
from unicodedata import name
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
w_img = 960  # 图像宽度
h_img = 540  # 图像高度
p_x = 479 
p_y = 269
f_x = 540.68603515625
f_y = 540.68603515625
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
# weigths = torch.load('/home/linxi/ServiceRobot-General/src/pose_estimate/scripts/yolov7-w6-pose.pt')
weights=torch.load("/home/linxi/yolo_pose/yolov7/yolov7.pt")
model = weights['model']
model = model.half().to(device)
_ = model.eval()
class YoloV7:
    def __init__(self) -> None:
        pass
    def func(self,image):
        image = letterbox(image, 960, stride=64, auto=True)[0]
        image_ = image.copy()
        image = transforms.ToTensor()(image)
        image = torch.tensor(np.array([image.numpy()]))
        image = image.to(device)
        image = image.half()

    # 姿势识别
        with torch.no_grad():
            output, _ = model(image)
            output = non_max_suppression_kpt(output, 0.35, 0.65, nc=model.yaml['nc'], nkpt=model.yaml['nkpt'], kpt_label=True)
            output = output_to_keypoint(output)
            print(output)
        nimg = image[0].permute(1, 2, 0) * 255
        nimg = nimg.cpu().numpy().astype(np.uint8)
        nimg = cv2.cvtColor(nimg, cv2.COLOR_RGB2BGR)    
        cv2.imshow("show",nimg)
        cv2.waitKey(1)
if __name__=="__main__":
    cap=cv2.VideoCapture(0)
    now_module=YoloV7()
    while (True):
        ret, image = cap.read()
        now_module.func(image)
        # func(image)
