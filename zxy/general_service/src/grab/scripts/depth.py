#! /usr/bin/env python3
import rospy
import message_filters
import cv_bridge
import cv2
from sensor_msgs.msg import Image
import math

W_img = 960 # 图像宽度
H_img = 540 # 图像高度
P_x = W_img//2-1 # 图像中心点x坐标
P_y = H_img//2-1 # 图像中心点y坐标
f_x = 540.68603515625
f_y = 540.68603515625
WIDTH = 960
HEIGHT = 540

point_args = [0.3, 0.5, 0.7]
_cv_bridge = cv_bridge.CvBridge()

def image_cb(image_rgb, image_depth):
    image = _cv_bridge.imgmsg_to_cv2(image_rgb, desired_encoding='passthrough')
    # cv2.imshow("thiswindow",image)
    depth = _cv_bridge.imgmsg_to_cv2(image_depth, desired_encoding='passthrough')
    # print(depth.shape)
    rospy.loginfo("oneceassssssssss assssssssssss")

    cv2.imshow("this_window",image)
    cv2.waitKey(10)
    # cv2.destroyAllWindows()
def real_pose(u_img, v_img, d):
    global p_x, p_y, f_x, f_y
    Z = d/math.sqrt(1+(u_img-p_x)**2/f_x**2+(v_img-p_y)**2/f_y**2)
    X = (u_img-p_x)*Z/f_x
    Y = (v_img-p_y)*Z/f_y
    y=abs(Y/1000)
    zzz= d/1000*math.cos(0.4)-y*math.sin(0.4)+0.04
    yyy=1.58-d/1000*math.sin(0.4)-y*math.cos(0.4)
    return X/1000+0.015, yyy, zzz
    
if __name__ == '__main__':
    rospy.init_node("depth_check")
    rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color", Image)
    depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_cb)
    rospy.spin()
