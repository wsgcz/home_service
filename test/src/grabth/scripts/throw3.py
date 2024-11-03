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

#???huoqu
# have_left=0.56
begin_throw = 0

# def start_spin(msg):
#     global begin_to_confirm_all_position_data,begin_throw
#     if (msg=="throw"):
#         start_spin_pub.publish("spin")
#     rospy.loginfo("ok I will face the trashbase")

# def start_forward(msg):
#     global begin_to_confirm_all_position_data,begin_throw
#     if (msg=="spin ok"):
#         start_get_distance_pub.publish("forward")
#     rospy.loginfo("ok I will get the diatance")

# def go_ahead(msg):
#     global machine_speed,begin_throw,have_left
#     have_left = msg
#     machine_speed.linear.x=(have_left-0.65)/2 # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向
#     machine_speed.linear.y=0
#     machine_speed.linear.z=0
#     machine_speed.angular.x=0
#     machine_speed.angular.y=0
#     machine_speed.angular.z=0
#     rospy.loginfo("continue move to the destination~")
#     speed_of_pub.publish(machine_speed)
#     rospy.sleep(2)
#     rospy.loginfo("arrive!!!")
#     reset_speed()
#     time.sleep(0.5)
#     begin_throw = 1

def throw_the_object():
    global control_arm_data
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[0]=0.6
    control_arm_data.velocity[1]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(9)
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
    down_state_pub.publish("0")
    #rospy.set_param("params_finish", 1)
   
def control_all_the_action_of_machine_throw():
    global begin_to_confirm_all_position_data,have_known_the_angle,have_set_everything,have_set_height,begin_throw,control_arm_data,have_left
    # print(rospy.is_shutdown())
    global machine_speed,now_ahead_speed,now_angle_speed,now_time_position_x,now_time_position_y
    while not rospy.is_shutdown():
        if begin_throw==0:
            rospy.sleep(0.02)
            continue
        else:
            throw_the_object()  # 语音，抓住
            begin_throw=0
            # begin_throw=0
            # continue
            # begin_back=1

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

#主体
if __name__=="__main__":
    rospy.init_node("general_service_throw")

    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10) #控制机器人线速度和角速度
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    
    start_into_throw_sub=rospy.Subscriber("/home_service/robot_state",String,start_spin,queue_size=10) #throw
    # start_spin_pub=rospy.Publisher("/home_service/robot_spin",String,queue_size=10)#begin spin

    # start_forward_sub=rospy.Subscriber("/home_service/robot_spin_reply",String,start_forward,queue_size=10)#spin ok
    # start_get_distance_pub=rospy.Publisher("/home_service/lidar_distance",String,queue_size=10) #begin to get distance

    get_distance_sub=rospy.Subscriber("/home_service/robot_getdistance",String,go_ahead,queue_size=10)#get distance and move
    
    down_state_pub=rospy.Publisher("/home_service/genenal_service_put_down_result",String,queue_size=10)#grab.py
    
    # tfBuffer = tf2_ros.Buffer()
    # tfSub = tf2_ros.TransformListener(tfBuffer)

    time1=time.time()

    control_all_the_action_of_machine_throw() #throw and reset

    #pub_speak=rospy.Publisher("/general_service_xfsaywords",String,queue_size=10)
    rospy.spin()



























#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data








