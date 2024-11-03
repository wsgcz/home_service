#!/usr/bin/python3
import os,cv2,sys,time,torch,rospy,tf2_ros,threading,insightface,message_filters
import numpy as np
import mediapipe as mp
import actionlib
import random
import matplotlib.pyplot as plt
from tqdm import tqdm
import numpy as np
from ultralytics import YOLO

from sklearn import preprocessing
from PIL import Image, ImageFont, ImageDraw
from math import atan2,pi,sqrt,sin,cos
from queue import Queue
from nav_msgs.msg import Odometry
from sklearn import preprocessing
from cv_bridge import CvBridge
from torchvision import transforms
from tf import transformations
from tf2_geometry_msgs import PointStamped,PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Twist
image_count = 0

cb_mutex = threading.Lock()

def image_callback(image_rgb):
    cb_mutex.acquire()
    if (image_count < 180):
        print("-------- i am taking photoes")
        if (image_count % 3 == 0):
            cv2.imwrite(f"/home/lzh/bin/{good}_{image_count}.jpg",image_rgb)
    cb_mutex.release()


if __name__ == "__main__":
    rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image,image_callback)
    good = "handwash"
    rospy.spin()
