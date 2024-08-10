#! /usr/bin/env python3
from signal import siginterrupt
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
imgae_count=0
def image_cb(image_rgb, image_depth):
    global imgae_count
    # if imgae_count<10:
    #     imgae_count+=1
    #     return
    image = _cv_bridge.imgmsg_to_cv2(image_rgb, desired_encoding='passthrough')
    # cv2.imshow("thiswindow",image)
    depth = _cv_bridge.imgmsg_to_cv2(image_depth, desired_encoding='passthrough')
    # print(depth.shape)
    rospy.loginfo("oneceassssssssss assssssssssss")
    for u_arg in point_args:
        for v_arg in point_args:
            u_img = int(WIDTH*u_arg)
            v_img = int(HEIGHT*v_arg)
            cv2.circle(image, (u_img, v_img), 10, (0, 50, 255), -1)
            real_z,real_y,real_z1=real_pose(u_img,v_img,depth[v_img][u_img])
            # rospy.loginfo("this is real_z:%f,this is real_z1:%f",real_z,real_z1)
            cv2.putText(image, ("D=%.2fm" % (real_z)), (u_img-40, v_img-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # cv2.putText(image, ("y=%.2fm" % (real_y)), (u_img-60, v_img-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
        # cv2.putText(image, ("D=%dcm" % (depth[v_img][u_img]//10)), (u_img-40, v_img-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("this_window",image)
    cv2.waitKey(10)
    # cv2.destroyAllWindows()
def real_pose(u_img, v_img, d):
    # global p_x, p_y, f_x, f_y
    other_angle=(v_img-270)*43/540
    this_angle=0.4*180/math.pi
    now_angle=abs(this_angle+other_angle)*math.pi/180
    now_angle1=abs(other_angle)*math.pi/180
    
    
    Z = d/math.sqrt(1+(u_img-P_x)**2/f_x**2+(v_img-P_y)**2/f_y**2)
    X = (u_img-P_x)*Z/f_x
    Y = (v_img-P_y)*Z/f_y
    y=(Y/1000)
    zzz= d/1000*math.cos(now_angle)+0.24-y*math.sin(now_angle1)
    # yyy=1.62-d/1000*math.sin(0.4)+y*math.cos(0.4)
    return X/1000+0.015, y, zzz
def real_pose1(u_img, v_img, d):
    global p_x, p_y, f_x, f_y
    this_angle=0.4*180/math.pi
    now_angle=abs(this_angle+other_angle)*math.pi/180
    now_angle1=abs(other_angle)*math.pi/180
    
    
    Z = d/math.sqrt(1+(u_img-P_x)**2/f_x**2+(v_img-P_y)**2/f_y**2)
    X = (u_img-P_x)*Z/f_x
    Y = (v_img-P_y)*Z/f_y
    y=abs(Y/1000)
    if v_img<270:
        other_angle=math.atan(d/1000)
        now_angle=1
    print("this is ",y)
    zzz= d/1000*math.cos(now_angle)+0.04
    # yyy=1.62-d/1000*math.sin(0.4)+y*math.cos(0.4)
    return X/1000+0.015, y, zzz

    
if __name__ == '__main__':
    rospy.init_node("depth_check")
    rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color", Image)
    depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_cb)
    rospy.spin()
