#! /usr/bin/env python3
# from configparser import RawConfigParser
# from enum import global_enum_repr
# from ctypes.wintypes import PINT
from multiprocessing import Process,Pool
from random import randrange
from re import S
from selectors import SelectorKey
import string
from turtle import left
from typing import final
from urllib import robotparser
from xml.dom.expatbuilder import theDOMImplementation
from xml.dom.minidom import ReadOnlySequentialNamedNodeMap
import xxlimited
# from pose_estimate.scripts.pose import task2
import rospy
import json
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math
import message_filters
import time
import cv_bridge
import cv2
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from waterplus_map_tools.srv import GetWaypointByName, GetWaypointByNameRequest, GetWaypointByNameResponse
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import asyncio
image,depth=None,None
import threading
import tf
import tf.transformations as transformations
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32

sys.path.append('/home/linxi/ServiceRobot-General/src/yolov5-in-ros/scripts')
for i in range(len(sys.path)):
    if "2.7" in sys.path[i]:
        sys.path.append(sys.path.pop(i))
        break
from libyolo import YoloV5s
yolo = YoloV5s(weight_path=None)
bridge = CvBridge()
machine_angle_set=0
Y=-1
xmin=2
p=Pool(10)
mutrex=threading.Lock()
mutrex1=threading.Lock()
now_map_pose_x=0
now_map_pose_y=0
draw_image=None
W_img = 960 # 图像宽度
H_img = 540 # 图像高度
P_x =W_img//2-1 # 图像中心点x坐标
P_y =H_img//2-1 # 图像中心点y坐标
f_x = 540.68603515625
f_y = 540.68603515625
color_image=None
aligned_depth_frame=None
depth_intrin=None
depth_image=None
grab_corner={"kitchen":[11.945,-1.018],"bedroom":[0.707,2.7]}
grab_room=None
sayword_number=0
# aligned_depth_frame,rs,depth_intrin,color_image,depth_image,yolo_mutex,

def ori_to_rpy(x, y, z, w):
    from tf import transformations
    (r, p, y) = transformations.euler_from_quaternion([x, y, z, w])
    return [r, p, y]
def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw
def rpy2quaternion(roll, pitch, yaw):
    x = math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    y = math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    z = math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    w = math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    return [x, y, z, w]
 
object_need_to_grab=None
odom_px=None
odom_py=None
odom_pz=None
odom_ox=None
odom_oy=None
odom_oz=None
odom_ow=None
last_time_postion_x=0
last_time_postion_y=0
last_time_postion_z=0
other_sayword_number=0
now_time_postion_x=0.16
now_time_postion_y=0
now_time_postion_z=0
machine_speed=Twist()
now_ahead_speed=0
now_angle_speed=0
begin_to_confirm_all_position_data=0
control_arm_data=JointState()
arm_postion_init=[0,0]
control_arm_data.name.append("lift")
control_arm_data.name.append("gripper")
control_arm_data.position.extend(arm_postion_init)
control_arm_data.velocity.extend(arm_postion_init)
have_known_the_angle=0
have_set_the_height=0
have_set_everything=0
final_position_y=0
d_x=0
d_z=0
now_pixel_x=0
last_pixel_x=0
d_pixel_x=0
begin_ahead_function=0
can_not_know_the_deepth=0
angle=20/180*math.pi
have_left=0.56
# yolo_mutex=threading.Lock()
machine_odom_now_x=0
machine_odom_now_y=0
machine_odom_last_x=0
machine_odom_last_y=0
machine_odom_distance=1.5
target_position_z=0
begin_back=0
begin_everything=0
over_yolo=0
not_have_catch_target=1
sleep_count=0
def control_angle_speed():
    global now_angle_speed,now_time_postion_x,d_x,can_not_know_the_deepth,d_pixel_x
    # while not rospy.is_shutdown() :
    if can_not_know_the_deepth:
        adjusted_step=d_pixel_x/20
        now_angle_speed=-0.2*now_time_postion_x
        if now_angle_speed>0.6:
            now_angle_speed=0.6
        elif now_angle_speed<-0.6:
            now_angle_speed=-0.6
    else:
        # tidu_xishu=d_
        # step_scale=tidu_xishu
        # adjusted_step=0.5-abs(step_scale)*10
        # now_angle_speed=-0.2*now_time_postion_x
        
        if now_time_postion_x>0.8:
            now_angle_speed=-0.6
        elif 0.4<abs(now_time_postion_x)<0.8:
            now_angle_speed=-now_time_postion_x*0.4
        elif 0.1<abs(now_time_postion_x)<0.4:
            now_angle_speed=-now_time_postion_x*0.5
        elif 0<abs(now_time_postion_x)<0.1:
            now_angle_speed=-now_time_postion_x
        elif now_time_postion_x<-0.8:
            now_angle_speed=0.6

        if now_angle_speed>0.6:
            now_angle_speed=0.6
        elif now_angle_speed<-0.6:
            now_angle_speed=-0.6
    # time.sleep(0.02)
def reset_parameter():
    global have_left,machine_odom_last_x,machine_odom_last_y,machine_odom_now_x,machine_odom_now_y,machine_speed
    global machine_odom_distance,target_position_z,begin_back,have_known_the_angle,have_set_the_height,have_set_everything
    global final_position_y,d_x,d_y,now_pixel_x,last_pixel_x,d_z,d_pixel_x,begin_ahead_function,can_not_know_the_deepth
    global object_need_to_grab,odom_px,odom_py,odom_pz,odom_ox,odom_ow,odom_oz,odom_ow,last_time_postion_x,last_time_postion_y,last_time_postion_z
    global now_time_postion_x,now_time_postion_y,now_time_postion_z,now_ahead_speed,now_angle_speed,color_image,depth_image,depth_intrin,aligned_depth_frame,begin_everything
    global over_yolo,not_have_catch_target,odom_oy,begin_to_confirm_all_position_data,sleep_count
    not_have_catch_target=1
    begin_to_confirm_all_position_data=1
    have_left=0.56
    machine_odom_now_x=0
    machine_odom_now_y=0
    machine_odom_last_x=0
    machine_odom_last_y=0
    machine_odom_distance=1.5
    target_position_z=0
    begin_back=0
    have_known_the_angle=0
    have_set_the_height=0
    have_set_everything=0
    final_position_y=0
    d_x=0
    d_z=0
    now_pixel_x=0
    last_pixel_x=0
    d_pixel_x=0
    begin_ahead_function=0
    can_not_know_the_deepth=0
    object_need_to_grab=None
    odom_px=None
    odom_py=None
    odom_pz=None
    odom_ox=None
    odom_oy=None
    odom_oz=None
    odom_ow=None
    last_time_postion_x=0
    last_time_postion_y=0
    last_time_postion_z=0

    now_time_postion_x=0.16
    now_time_postion_y=0
    now_time_postion_z=0
    machine_speed=Twist()
    now_ahead_speed=0
    now_angle_speed=0
    color_image=None
    aligned_depth_frame=None
    depth_intrin=None
    depth_image=None
    over_yolo=0
    sleep_count=0
    # begin_everything=1
def reset_speed():
    global machine_speed
    machine_speed.linear.x=0
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=0
    for i in range(5):
        speed_of_pub.publish(machine_speed)
        rate=rospy.Rate(10)
        rate.sleep()
def grab_the_object():
    global now_time_postion_y,have_set_everything,begin_back,over_yolo,object_need_to_grab
    speak_data=String()
    speak_data.data="ok i have find the "+object_need_to_grab
    pub_speak.publish(speak_data)
    control_arm_data.position[0]=final_position_y
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=0.5
    control_arm_data.velocity[0]=0.5
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(3)
    # time_to_sleep=rospy.Rate()
    # time_to_sleep.sleep()
    have_set_everything=0
    over_yolo=1
    begin_back=1
def know_target_and_go_ahead():
    global machine_speed,have_known_the_angle,begin_to_confirm_all_position_data,final_position_y
    machine_speed.linear.x=0
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=now_angle_speed
    print("this is machine_speed.z %f,now x %f",machine_speed.angular.z,now_time_postion_x)
    if (abs(now_time_postion_x)>0.015):
        speed_of_pub.publish(machine_speed)
    else :
        reset_speed()
        speed_of_pub.publish(machine_speed)
        time.sleep(0.5)
        print("over!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        final_position_y=now_time_postion_y
        begin_to_confirm_all_position_data=0
        have_known_the_angle=1
def reset_control_arm_position():
    global final_position_y
    control_arm_data.position[0]=final_position_y
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=0
    control_arm_data.velocity[0]=0
    arm_action_pub.publish(control_arm_data)
def set_the_speed_of_hegiht_action():
    global now_time_postion_y,final_position_y,control_arm_data,have_known_the_angle,have_set_the_height,begin_ahead_function
    # if abs(final_position_y)<0.015:
    global final_position_y
    time.sleep(0.5)
    
    final_position_y=0.9
    control_arm_data.position[0]=0.5
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0.5
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=0.5
    arm_action_pub.publish(control_arm_data)
    # time.sleep(3)
    rate=rospy.Rate(0.333)
    rate.sleep()
    control_arm_data.position[0]=final_position_y
    arm_action_pub.publish(control_arm_data)
    # time_to_sleep=final_position_y*2+10
    # time_to_sleep=max(time_to_sleep,1)
    rate=rospy.Rate(0.08)
    rate.sleep()
    reset_control_arm_position()
    # final_position_y=now_time_postion_y
    have_known_the_angle=0
    have_set_the_height=1   
    begin_ahead_function=1
def set_begin_odom_x_and_y():
    global machine_odom_last_x,machine_odom_last_y,machine_odom_now_x,machine_odom_now_y,target_position_z,machine_odom_now_x,machine_odom_now_y
    machine_odom_last_x=machine_odom_now_x
    machine_odom_last_y=machine_odom_now_y
    target_position_z=now_time_postion_z
def calculate_ahead_speed():
    global now_time_postion_z,now_ahead_speed,have_lefthave_known_the_angle
    # have_left=now_time_postion_z-0.5
    rospy.loginfo("this is left %f",have_left)
    
    if have_left>0.8:
         now_ahead_speed=0.35
    elif 0.4<have_left<0.8:
        now_ahead_speed=have_left*0.4
    elif 0.1<abs(have_left)<0.4:
        now_ahead_speed=have_left*0.3
    elif 0.03<have_left<0.1:
        now_ahead_speed=have_left*0.5
    elif 0.01<have_left<0.03:
        now_ahead_speed=0.01
    elif 0<have_left<0.01:
        now_ahead_speed=0  
    print("this is pose_z ",now_time_postion_z)
def set_the_speed_of_ahead_action():
    global machine_speed,have_known_the_angle,have_set_everything,have_set_the_height,have_left
    machine_speed.linear.x=now_ahead_speed
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=0
    # rospy.loginfo("this is left %f",)
    if (abs(have_left)<0.015):
        rospy.loginfo("this is over")
        reset_speed()
        time.sleep(0.5)
        have_set_the_height=0
        have_set_everything=1
    else :
        rospy.loginfo("continue pub message ")
        speed_of_pub.publish(machine_speed)

# 这个函数用以获取此刻的odom位置信息
def control_angle_reset():
    global now_map_pose_x,now_map_pose_y,machine_odom_now_x,machine_odom_now_y,grab_corner,grab_room,machine_angle_set
    global machine_speed
    now_map_pose_y,now_map_pose_y=get_map_pose(machine_odom_now_x,machine_odom_now_y)
    theta=(grab_corner[grab_room][0]-now_map_pose_x)/(now_map_pose_y-grab_corner[grab_room][1])
    theta=math.atan(theta)
    # mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow]) 
    change_theta=theta-machine_angle_set
    if change_theta>0:
        speed=-0.3
    else :
        speed=0.3
    machine_speed.angular.z=speed
    speed_of_pub.publish(machine_speed)
    time.sleep(2)
    rospy.loginfo("control angle and reset")
    reset_speed()
    machine_speed.linear.x=0.1
    speed_of_pub.publish(machine_speed)
    time.sleep(2)
    reset_speed()
    
    
def to_get_odom_position_data(odom:Odometry):
    global odom_px,odom_py,odom_pz,odom_ow,odom_oz,odom_oy,odom_ox
    odom_px=odom.pose.pose.position.x
    odom_py=odom.pose.pose.position.y
    odom_pz=odom.pose.pose.position.z
    odom_ox=odom.pose.pose.orientation.x
    odom_oy=odom.pose.pose.orientation.y
    odom_oz=odom.pose.pose.orientation.z
    odom_ow=odom.pose.pose.orientation.w   
# 这个函数用以获取需要抓取的物品
def get_the_need_object(msg:String):
    global object_need_to_grab,begin_to_confirm_all_position_data,begin_everything,begin_back
    reset_parameter()
    object_need_to_grab=msg.data
    begin_to_confirm_all_position_data=1
    # begin_everything=1
def intel_real_pose(y,z):
    angle=20/180*math.pi
    real_z=z*math.cos(angle)+y*math.sin(angle)
    real_y=-z*math.sin(angle)+y*math.cos(angle)+1.3
    return real_y,real_z
def control_all_the_action_of_machine():
    global begin_to_confirm_all_position_data,have_known_the_angle,have_set_everything,have_set_the_height,begin_ahead_function,begin_back,begin_everything
    # print(rospy.is_shutdown() )
    global begin_back,not_have_catch_target,machine_speed,sleep_count
    while not rospy.is_shutdown() :
        if begin_everything==0:
            rospy.sleep(0.02)
            continue
        # print("this is object ",now_time_postion_x,"and this is D_x ",d_x)        
        # rospy.loginfo("   finale_postino_y  %f",final_position_y)
        # if not_have_catch_target==1:
        #     if sleep_count==0:
        #         rospy.sleep(2)
        #     sleep_count+=1
        #     machine_speed.linear.x=0
        #     machine_speed.linear.y=0
        #     machine_speed.linear.z=0
        #     machine_speed.angular.x=0
        #     machine_speed.angular.y=0
        #     machine_speed.angular.z=0.1
        #     speed_of_pub.publish(machine_speed)
        
        elif begin_to_confirm_all_position_data==1:
            print("11111111111")
            control_angle_speed()
            know_target_and_go_ahead()

            # continue
        elif have_known_the_angle==1:
            # have_known_the_angle=0
            # have_set_the_height=1
            # time.sleep(3)
            # begin_ahead_function=1
            set_the_speed_of_hegiht_action()
            set_begin_odom_x_and_y()

            # continue
        elif have_set_the_height==1:
            rospy.loginfo("111111111111111111111")
            calculate_ahead_speed()
            set_the_speed_of_ahead_action()

            # continue
        elif have_set_everything==1:
            grab_the_object()
            # begin_everything=0
            # continue
            begin_back=1

        elif begin_back==1:
            rospy.sleep(1)
            back_and_reset()
            # control_angle_reset()
            #黄维肖说，抓取完成返回时要发一个“1”给主函数
            first_put_down()
            the_get=String()
            the_get.data="1"
            pub_get_over.publish(the_get)
            # reset_parameter()
            begin_everything=0

        time.sleep(0.02)
        
def first_put_down():
    # time.sleep(2)
    global control_arm_data,final_position_y
    control_arm_data.position[0]=0.5
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=3
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    
    rospy.sleep(4)
    rospy.loginfo("now it is in low")
    control_arm_data.position[0]=0
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    # control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=3
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    
def say_word_task(yolo_message:dict,img):
    global draw_image
    rospy.loginfo("entered sayword task insight %s",yolo_message)
    for i in yolo_message.keys():
        # i=i[0]
        rospy.loginfo("entered final insight %s",i)
        x=int(yolo_message[i][0][0])
        y=int(yolo_message[i][0][1])
        w=int(yolo_message[i][0][2]/2)
        h=int(yolo_message[i][0][3]/2)
        path="/home/linxi/ServiceRobot-General/src/grab/image_data/"
        path=path+i+".jpg"
        img=img[y-h:y+h,x-w:x+w]
        # cv2.imwrite()
        rospy.loginfo("have entered sayword task")
        data=String()
        data.data=i
        pub_speak.publish(data)
        cv2.imwrite(path,img)
        time.sleep(1.5)
    path_image="/home/linxi/ServiceRobot-General/src/grab/image_data/all.jpg"
    cv2.imwrite(path_image,draw_image)
def calculate_postion_data(mutex):
    global object_need_to_grab,last_time_postion_x,last_time_postion_y,last_time_postion_z,now_time_postion_x,now_time_postion_y,now_time_postion_z,d_x,d_z,can_not_know_the_deepth
    global last_pixel_x,now_pixel_x,d_pixel_x,begin_ahead_function,angle,have_left,yolo
    global aligned_depth_frame,depth_intrin,color_image,depth_image,machine_odom_distance,not_have_catch_target,begin_to_confirm_all_position_data
    global sayword_number,draw_image,have_known_the_angle,other_sayword_number
    mutex.acquire()
    yolo_result_message, draw_image = yolo.inference(img=color_image, if_show=True)
    # if cv2.waitKey(1)!=27:
        # cv2.imshow("yolo_intermediate", draw_image)
        # pass
    # print(yolo_result_message)
    rospy.loginfo("2222222222222222222222222222222")
    if len(yolo_result_message)!=0  and sayword_number==0 and  have_known_the_angle==1:
        rospy.loginfo("this is sayword task")
        rospy.loginfo("yolo message is %s",yolo_result_message)
        sayword_number+=1
        task=threading.Thread(target=say_word_task,args=(yolo_result_message,color_image,))
        task.start()
    if len(yolo_result_message)!=0  and other_sayword_number==0 :
        rospy.loginfo("this is sayword task")
        rospy.loginfo("yolo message is %s",yolo_result_message)
        other_sayword_number+=1
        task=threading.Thread(target=say_word_task,args=(yolo_result_message,color_image,))
        task.start()
    if object_need_to_grab not in list(yolo_result_message.keys()):
        rospy.loginfo("use odom message______________________________________________________________ ")
        rospy.loginfo("this is machine odom distance %f",machine_odom_distance)
        if (machine_odom_distance<have_left and begin_ahead_function==1):
            rospy.loginfo("5555555555555555555555555555555555555555555555555555555")
            have_left=machine_odom_distance
        return
    try:
        person=yolo_result_message[f'{object_need_to_grab}']
        rospy.loginfo("11111111111111")
        use_machine_odom_distance_flag=0
        for person_i in person:
            # not_have_catch_target=0
            # begin_to_confirm_all_position_data=1
            u_img=int(person_i[0])
            v_img=int(person_i[1])
            # print(person_i)
            d = aligned_depth_frame.get_distance(u_img, v_img)
            rospy.loginfo("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
            if d>0.000000001:
            # print(d)
                rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                use_machine_odom_distance_flag=1
                real_x,real_y,real_z=rs.rs2_deproject_pixel_to_point(intrin=depth_intrin, pixel=[u_img, v_img], depth=d)
                # real_y,real_z=intel_real_pose(real_y,real_z)
                real_x-=0.07
                z=real_z
                y=real_y
                fuy=-real_y
                real_z=z*math.cos(angle)+fuy*math.sin(angle)+0.12
                rospy.loginfo("this is real_z %f" ,real_z)
                real_y=-z*math.sin(angle)+fuy*math.cos(angle)+1.18
                rospy.loginfo ("this is real_y  %f",real_y)
                last_time_postion_x=now_time_postion_x
                last_time_postion_y=now_time_postion_y
                now_time_postion_x=real_x
                now_time_postion_y=real_y
                if begin_ahead_function==0:
                    last_time_postion_z=now_time_postion_z
                    now_time_postion_z=real_z
                else :
                    if (real_z<last_time_postion_z):
                        if abs(now_time_postion_z-real_z>0.1):
                            continue
                        last_time_postion_z=now_time_postion_z
                        now_time_postion_z=real_z
                        have_left=now_time_postion_z-0.5
                d_x=now_time_postion_x-last_time_postion_x
                d_z=now_time_postion_z-last_time_postion_z
                last_pixel_x=now_pixel_x
                now_pixel_x=u_img
                d_pixel_x=now_pixel_x-last_pixel_x
                # print("this is real_x ",now_time_postion_x," real_y ",now_time_postion_y," real_z " ,now_time_postion_z," ",begin_to_confirm_all_position_data )
                can_not_know_the_deepth=0
            else :
                last_pixel_x=now_pixel_x
                now_pixel_x=u_img
                d_pixel_x=now_pixel_x-last_pixel_x
                can_not_know_the_deepth=1
                # time.sleep(0.01)
        if use_machine_odom_distance_flag:
            if (machine_odom_distance<have_left and begin_ahead_function==1):
                rospy.loginfo("5555555555555555555555555555555555555555555555555555555")
                have_left=machine_odom_distance 
    except BaseException as e:
        rospy.loginfo("error have token place")
        pass   
    mutex.release()
def put_down(msg:String):
    global control_arm_data,final_position_y
    control_arm_data.position[0]=0.5
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(1.5)
    control_arm_data.position[0]=0.85
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(3.5)
    control_arm_data.position[0]=0.85
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0.5
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(3.5)
    control_arm_data.position[0]=0.5
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(3.5)
    control_arm_data.position[0]=0
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4.5)
    ok_msg=String()
    ok_msg.data="1"
    pub_put_down_rslt_pub.publish(ok_msg)
    
        
def back_and_reset():
    
    global begin_back,machine_speed
    reset_speed()
    rospy.loginfo("11111111111111111111111111111111111111")
    machine_speed.linear.x=-0.1
    speed_of_pub.publish(machine_speed)
    rospy.sleep(rospy.Duration(4.0))
    reset_speed()
    begin_back=0
def intel_realsense_image_and_depth():

    global object_need_to_grab,last_time_postion_x,last_time_postion_y,last_time_postion_z,now_time_postion_x,now_time_postion_y,now_time_postion_z,d_x,d_z
    global aligned_depth_frame,depth_intrin,color_image,depth_image,begin_everything
    while True:
        if begin_everything==1:
            break
    pipeline = rs.pipeline()
    config = rs.config()
    width=1280
    height=720
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
    # Start streaming(开启)
    # pipeline.start(config)
    profile = pipeline.start(config)
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    # 获取相机内参
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    camera_parameters = {
        'fx': intr.fx, 'fy': intr.fy,
                        'ppx': intr.ppx, 'ppy': intr.ppy,
                        'height': intr.height, 'width': intr.width,
                        'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                        }
    # 保存内参到本地    

    try:
        while not rospy.is_shutdown() :
            if begin_everything==0:
                rospy.sleep(0.02)
                continue
            if over_yolo==1:
                rospy.sleep(15)
            # Wait for a coherent pair of frames: depth and color(等待连贯的一对帧:深度图和rgb图)
            align_to = rs.stream.color
            align = rs.align(align_to)
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()
        # 深度参数，像素坐标系转相机坐标系用到
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            # Convert images to numpy arrays(转换image格式为numpy数组格式)
    
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
    
            color_image = np.asanyarray(color_frame.get_data())
    
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # 在深度图像上应用颜色映射(图像必须首先转换为每像素8位)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally(水平堆叠两幅图像)
            images = np.hstack((color_image, depth_colormap))
            yolo_mutex=threading.Lock()
            yolo_inferen_task=threading.Thread(target=calculate_postion_data,args=(yolo_mutex,))
            yolo_inferen_task.start()

            show_image()
            # Show images(显示图片)
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense',color_image)
            # # cv2.imshow('RealSense',images)
            
            # key = cv2.waitKey(1)
            # # Press esc or 'q' to close the image window
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break
            # elif key & 0xFF == ord('s'):
            #     cv2.imwrite('savefile.jpg',images)
            time.sleep(0.01)
    finally:
        # Stop streaming
        pipeline.stop()
def image_process_and_control_arm():
    task1=threading.Thread(target=intel_realsense_image_and_depth)
    # task2=threading.Thread(target=control_all_the_action_of_machine)
    # task3=threading.Thread(target=control_angle_speed)
    
    task1.start()
    # task2.start()
    # task3.start()
    control_all_the_action_of_machine()
def grab_room_cb(msg:String):
    global grab_room
    grab_room=msg.data
def start_cb(msg:String):
    global begin_everything
    begin_everything=1   
    rospy.loginfo("ok i have set start_cb value")
def get_odom_message(msg:Odometry):
    global machine_odom_now_y,machine_odom_now_x,machine_odom_last_y,machine_odom_last_x,machine_odom_distance,target_position_z,last_time_postion_z,now_time_postion_z
    # if last_time_postion_z==now_time_postion_z:
    #     target_position_z=last_time_postion_z
    #     machine_odom_last_x=machine_odom_now_x
    #     machine_odom_last_y=machine_odom_now_y
    machine_odom_now_x= msg.pose.pose.position.x
    machine_odom_now_y=msg.pose.pose.position.y
    
    if have_set_the_height:
        machine_odom_distance=math.sqrt((machine_odom_now_x-machine_odom_last_x)**2+(machine_odom_last_y-machine_odom_now_y)**2 )
        machine_odom_distance=target_position_z-machine_odom_distance-0.5
        rospy.loginfo("this is target_distance %f machine_odom_distance %f",target_position_z,machine_odom_distance)
    
    # rospy.loginfo("odom message have accepted ")
def angle_cb(msg:Float32):
    global machine_angle_set
    machine_angle_set=msg.data
def get_map_pose(x, y):
        ps = PointStamped()
        ps.header.frame_id = "odom"
        ps.point.x = x
        ps.point.y = y
        ps.header.stamp = rospy.Time.now()
        ps_new:PointStamped
        ps_new =tfBuffer.transform(ps, "map", rospy.Duration(1))
        return ps_new.point.x, ps_new.point.y
def show_image():
    global draw_image,begin_everything
    # print(draw_image)
    if begin_everything:
        if not (draw_image is None):
            if cv2.waitKey(1)!=27:
                    cv2.imshow("image",draw_image)
if __name__=="__main__":
    rospy.init_node("general_service_grab")
    object_need_to_grab_subsriber=rospy.Subscriber("general_service_need",String,get_the_need_object,queue_size=10)
    odom_grab_position_subscriber=rospy.Subscriber("odom",Odometry,get_odom_message,queue_size=10)
    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30)
    pub_get_over=rospy.Publisher("genenal_service_get_it",String,queue_size=10)
    pub_put_down_sub=rospy.Subscriber("genenal_service_put_down",String,put_down,queue_size=10)
    pub_put_down_rslt_pub=rospy.Publisher("genenal_service_put_down_result",String,queue_size=10)
    start_grab_sub=rospy.Subscriber("start_grab",String,start_cb,queue_size=10)
    pub_speak=rospy.Publisher("/general_service_xfsaywords",String,queue_size=10)
    grab_room_sub=rospy.Subscriber("grab_room",String,grab_room_cb,queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    angle_sub=rospy.Subscriber("/general_service_angle",Float32,angle_cb,queue_size=10)
    # time.sleep(5)
    # mg=machince_grab()
    # intel_realsense_image_and_depth()
    image_process_and_control_arm()
    # mg.pub_loc.publish(mg.twist_msg)
    # rospy.loginfo("ASdadssadsadad")

    rospy.spin()

