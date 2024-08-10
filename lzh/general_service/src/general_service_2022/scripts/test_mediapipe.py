from ast import Str
from gettext import find
from unittest import result
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi, atan
import json
import time
import numpy as np


mp_drawing = mp.solutions.drawing_utils  # type: ignore
mp_drawing_styles = mp.solutions.drawing_styles  # type: ignore
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)