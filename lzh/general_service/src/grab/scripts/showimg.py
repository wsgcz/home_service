#! /home/linxi/anaconda3/envs/ros/bin/python
# -*- coding: UTF-8 -*-


import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def show_image(image):
    # rospy.loginfo("hello")
    cv2.imshow("MediaPipe", _cv_bridge.imgmsg_to_cv2(image))
    cv2.waitKey(1)


if __name__ == '__main__':
    _cv_bridge = CvBridge()
    rospy.init_node("show_mediapipe_processed_image", anonymous=True)
    rospy.Subscriber("Follow_image", Image, show_image, queue_size=1)
    rospy.spin()