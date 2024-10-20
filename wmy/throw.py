#! /home/wmy/anaconda3/envs/grabb/bin/python
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
#import tf2_ros
#from tf2_geometry_msgs import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import asyncio
#image,depth=None,None
#import threading
#import tf
#import tf.transformations as transformations
#import numpy as np
#from std_msgs.msg import Float32

#参数
control_arm_data=JointState()
arm_postion_init=[0,0]
control_arm_data.name.append("lift")
control_arm_data.name.append("gripper")
control_arm_data.position.extend(arm_postion_init)
control_arm_data.velocity.extend(arm_postion_init)

machine_speed=Twist()

have_known_the_angle=0
have_set_everything=0
have_set_height = 0
begin_everything = 0
begin_to_confirm_all_position_data=0


#???huoqu
have_left=0.56
now_time_position_x = 0.31
now_time_position_y = 0.2

def start_find(msg):
    global begin_to_confirm_all_position_data,begin_everything
    begin_to_confirm_all_position_data = 1   
    begin_everything = 1
    rospy.loginfo("ok I will find the trashbase")


#调整角度使x吻合
def control_angle_speed():# 设定旋转角度   ####ke dong tai ping hua gai jin
    global now_angle_speed,now_time_position_x
    if now_time_position_x>0.8:
        now_angle_speed=-0.6
    elif 0.4<abs(now_time_position_x)<0.8:
        now_angle_speed=-now_time_position_x*0.7
    elif 0.1<abs(now_time_position_x)<0.4:
            now_angle_speed=-now_time_position_x*0.5
    elif 0<abs(now_time_position_x)<0.1:
            now_angle_speed=-now_time_position_x*0.3
    elif now_time_position_x<-0.8:
        now_angle_speed=0.6

    if now_angle_speed>0.6:
        now_angle_speed=0.6
    elif now_angle_speed<-0.6:
        now_angle_speed=-0.6
    # time.sleep(0.02)

def know_target_and_go_ahead():
    global machine_speed,have_known_the_angle,begin_to_confirm_all_position_data,now_time_position_x
    machine_speed.linear.x=0
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=now_angle_speed
    # print("this is machine_speed.z %f,now x %f"%(machine_speed.angular.z,now_time_postion_x))
    if (abs(now_time_position_x)>0.02):
        speed_of_pub.publish(machine_speed)
        rospy.sleep(3)
        now_time_position_x = now_time_position_x - 0.1
    else :
        reset_speed()
        speed_of_pub.publish(machine_speed)
        time.sleep(0.5)
        rospy.loginfo("spin succeed!!!!")
        begin_to_confirm_all_position_data=0
        have_known_the_angle=1
# def control_angle_reset():
#     global now_map_pose_x,now_map_pose_y,machine_odom_now_x,machine_odom_now_y,grab_corner,grab_room,machine_angle_set
#     global machine_speed
#     #la ji wei zhi huo qu
#     now_map_pose_x,now_map_pose_y=get_map_pose(machine_odom_now_x,machine_odom_now_y)
#     theta=(1-now_map_pose_x)/(now_map_pose_y-1)  ####### huoqu
#     theta=math.atan(theta)
#     # mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow]) 
#     change_theta=theta-machine_angle_set
#     if change_theta>0:
#         speed=-0.3
#     else :
#         speed=0.3
#     machine_speed.angular.z=speed
#     speed_of_pub.publish(machine_speed)
#     time.sleep(2)
#     rospy.loginfo("control angle and reset")
#     reset_speed()
#     machine_speed.linear.x=0.1
#     speed_of_pub.publish(machine_speed)
#     time.sleep(2)
#     reset_speed()

def set_height():
    global now_time_position_y,control_arm_data,have_set_height,have_known_the_angle
    # rospy.loginfo("this is final_y %f",final_position_y)
    if (now_time_position_y>0.7):
        control_arm_data.position[0]=now_time_position_y + 0.1
        control_arm_data.velocity[1]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(4)
        have_set_height = 1
        have_known_the_angle = 0
        rospy.loginfo("set height over!!!")
    else:
        have_set_height = 1
        have_known_the_angle = 0
    
#移动到目标位置
def calculate_ahead_speed():  ### gaijin
    global now_ahead_speed,have_left,have_known_the_angle
    rospy.loginfo("this is have_left %f",have_left)
    if have_left>0.8:
        now_ahead_speed=0.2
    elif 0.4<have_left<0.8:
        now_ahead_speed=have_left*0.3
    elif 0.1<abs(have_left)<0.4:
        now_ahead_speed=have_left*0.4
    elif 0.03<have_left<0.1:
        now_ahead_speed=have_left*0.6
    elif 0.01<have_left<0.03:
        now_ahead_speed=0.015
    elif have_left<0.01:
        now_ahead_speed=0  
    # print("this is now_time_position_z(正在计算速度时) ",now_time_postion_z)
def set_the_speed_of_ahead_action():
    global machine_speed,have_known_the_angle,have_set_everything,have_set_height,have_left
    machine_speed.linear.x=now_ahead_speed # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=0
    # rospy.loginfo("this is left %f",)
    if (abs(have_left)<0.015):
        rospy.loginfo("arrive!!!")
        reset_speed()
        time.sleep(0.5)
        have_set_height=0
        have_set_everything=1
    else :
        rospy.loginfo("continue move to the destination~")
        speed_of_pub.publish(machine_speed)
        rospy.sleep(1)
        have_left = have_left - 0.186
# def to_get_odom_position_data(odom:Odometry):
#     global odom_px,odom_py,odom_pz,odom_ow,odom_oz,odom_oy,odom_ox
#     odom_px=odom.pose.pose.position.x
#     odom_py=odom.pose.pose.position.y
#     odom_pz=odom.pose.pose.position.z
#     odom_ox=odom.pose.pose.orientation.x
#     odom_oy=odom.pose.pose.orientation.y
#     odom_oz=odom.pose.pose.orientation.z
#     odom_ow=odom.pose.pose.orientation.w   


#抓取（人递送）

# def get_the_need_object(msg:String):
#     global object_need_to_grab,begin_to_confirm_all_position_data,begin_everything,begin_back
    
#     reset_parameter()  ###!!!!!!!!!!!!!
#     object_need_to_grab=msg.data
#     # begin_to_confirm_all_position_data=1
#     begin_everything=1


# def intel_real_pose(y,z):   ###20 shi yao jin xing de zhua qu can shu tiao jie
#     angle=20/180*math.pi
#     real_z=z*math.cos(angle)+y*math.sin(angle)
#     real_y=-z*math.sin(angle)+y*math.cos(angle)+1.3  ##rao x axis xuan zhuan
#     return real_y,real_z


def throw_the_object():
    global have_set_everything,control_arm_data
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0.5
    control_arm_data.velocity[1]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(9)
    machine_speed.linear.x=-0.1 # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=0
    speed_of_pub.publish(machine_speed)
    rospy.sleep(5)
    reset_speed()
    control_arm_data.position[0]= 0 #下降且闭合
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[1]=0
    control_arm_data.velocity[1]=1
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    rospy.loginfo("finish throw the trash!!!")
    rospy.set_param("params_finish", 1)
   
def control_all_the_action_of_machine_throw():
    global begin_to_confirm_all_position_data,have_known_the_angle,have_set_everything,have_set_height,begin_everything,control_arm_data,have_left
    # print(rospy.is_shutdown())
    global machine_speed,now_ahead_speed,now_angle_speed,now_time_position_x,now_time_position_y
    while not rospy.is_shutdown():
        if begin_everything==0:
            rospy.sleep(0.02)
            continue
        elif begin_to_confirm_all_position_data==1:  
            #print("11111111111")
            control_angle_speed()   #计算旋转角度
            know_target_and_go_ahead()          # 进行旋转，使之大概对准物体
            # continue
        elif have_known_the_angle==1:
            set_height()
        elif have_set_height==1:                #是否调整好高度
            rospy.loginfo("angle okkk")
            calculate_ahead_speed()                 #计算机器人前进继续前进的速度
            set_the_speed_of_ahead_action()         #发送给机器人控制节点，控制机器人向前走
            # continue
        elif have_set_everything==1 :     # 是否所有都准备就绪
            #check_pass_state()           #que ren di song cheng gong
            throw_the_object()  # 语音，抓住
            reset_speed()
            begin_everything=0
            # begin_everything=0
            # continue
            # begin_back=1


#重置
# def reset_parameter():
#     global have_left,machine_odom_last_x,machine_odom_last_y,machine_odom_now_x,machine_odom_now_y,machine_speed
#     global machine_odom_distance,target_position_z,begin_back,have_known_the_angle,have_set_height,have_set_everything
#     global final_position_y,d_x,d_y,now_pixel_x,last_pixel_x,d_z,d_pixel_x,begin_ahead_function,can_not_know_the_deepth
#     global object_need_to_grab,odom_px,odom_py,odom_pz,odom_ox,odom_ow,odom_oz,last_time_postion_x,last_time_postion_y,last_time_postion_z
#     global now_time_postion_x,now_time_postion_y,now_time_postion_z,now_ahead_speed,now_angle_speed,color_image,depth_image,depth_intrin,aligned_depth_frame,begin_everything
#     global over_yolo,not_have_catch_target,odom_oy,begin_to_confirm_all_position_data,sleep_count

#     have_left=0.56
#     begin_to_confirm_all_position_data=1
#     have_known_the_angle=0
#     #have_set_the_height=0
#     have_set_everything=0
    
#     machine_speed=Twist()
#     have_set_height = 0
#     # begin_everything=1
def reset_speed():
    global machine_speed
    machine_speed.linear.x=0
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=0
    rate=rospy.Rate(10)
    for i in range(5):
        speed_of_pub.publish(machine_speed)
        rate.sleep()
#def reset_control_arm_position():
 #   global final_position_y
  #  control_arm_data.position[0]=final_position_y
   # control_arm_data.position[1]=0
    #control_arm_data.velocity[1]=0
    #control_arm_data.velocity[0]=0
    #arm_action_pub.publish(control_arm_data)
# def set_begin_odom_x_and_y():
#     global machine_odom_last_x,machine_odom_last_y,machine_odom_now_x,machine_odom_now_y,target_position_z
#     machine_odom_last_x=machine_odom_now_x
#     machine_odom_last_y=machine_odom_now_y
#     target_position_z=now_time_postion_z # target_position_z指的是距离物体的距离

# #转化
# def ori_to_rpy(x, y, z, w):
#     from tf import transformations
#     (r, p, y) = transformations.euler_from_quaternion([x, y, z, w])
#     return [r, p, y]
# def quart_to_rpy(x, y, z, w):
#     roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
#     pitch = math.asin(2 * (w * y - x * z))
#     yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
#     return roll, pitch, yaw
# def rpy2quaternion(roll, pitch, yaw):
#     x = math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
#     y = math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
#     z = math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
#     w = math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
#     return [x, y, z, w]
# def get_map_pose(x, y):
#         ps = PointStamped()
#         ps.header.frame_id = "odom"
#         ps.point.x = x
#         ps.point.y = y
#         ps.header.stamp = rospy.Time.now()
#         ps_new:PointStamped
#         ps_new =tfBuffer.transform(ps, "map", rospy.Duration(1))
#         return ps_new.point.x, ps_new.point.y

    
# #多线程

# def image_process_and_control_arm_throw():   ##duo xian cheng(yong yu you hua) #通过多线程机制，将图像处理和机械臂控制分开运行，以提高执行效率。
#     task1=threading.Thread(target=intel_realsense_image_and_depth)
#     # task2=threading.Thread(target=control_all_the_action_of_machine)
#     # task3=threading.Thread(target=control_angle_speed)
#     task1.start()
#     # task2.start()
#     # task3.start()
#     control_all_the_action_of_machine_throw()  

# def angle_cb(msg:Float32):
#     global machine_angle_set
#     machine_angle_set=msg.data

#主体
if __name__=="__main__":
    rospy.init_node("general_service_throw")

    start_throw_sub=rospy.Subscriber("move_robot_arm",String,start_find,queue_size=10)
    
    #trash_bin_subsriber=rospy.Subscriber("general_service_need",String,get_the_need_object,queue_size=10)
    #odom_grab_position_subscriber=rospy.Subscriber("odom",Odometry,get_odom_message,queue_size=10)
    #angle_sub=rospy.Subscriber("/general_service_angle",Float32,angle_cb,queue_size=10)
    #start_throw_sub=rospy.Subscriber("start_throw",String,start_th,queue_size=10)
    #chuang jian fa bu dui xiang

    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10) #控制机器人线速度和角速度

    #open_sub=rospy.Subscriber("move_robot_arm",String,start_open,queue_size=10)
    #close_sub=rospy.Subscriber("move_robot_arm",String,start_close,queue_size=10)
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    
    # tfBuffer = tf2_ros.Buffer()
    # tfSub = tf2_ros.TransformListener(tfBuffer)
    
    time1=time.time()

    control_all_the_action_of_machine_throw()

    #pub_speak=rospy.Publisher("/general_service_xfsaywords",String,queue_size=10)
    rospy.spin()



























#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data








