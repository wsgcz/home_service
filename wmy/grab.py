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
from sensor_msgs.msg import LaserScan
import asyncio

#参数
control_arm_data=JointState()
arm_postion_init=[0,0]
control_arm_data.name.append("lift")
control_arm_data.name.append("gripper")
control_arm_data.position.extend(arm_postion_init)
control_arm_data.velocity.extend(arm_postion_init)

begin_open = 0
begin_close = 0

#启动
def start_open(msg):
    global begin_open
    if (msg=="open"):
        begin_open = 1   
        rospy.loginfo("ok I will start")

def start_close(msg):
    global begin_close
    if (msg=="close"):
        begin_close = 1   
        rospy.loginfo("ok I will close")

  
#张开夹爪
def open():
    global begin_open,control_arm_data
    if(begin_open == 1):
        rospy.loginfo("Open!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0.7
        control_arm_data.velocity[0]=0.5
        control_arm_data.position[1]=0
        control_arm_data.velocity[1]=0
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(5)
        control_arm_data.position[1]=0.5
        control_arm_data.velocity[1]=2
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(5)
        rospy.set_param("params_finish", 1)
        

#抓取
def close():
    global begin_close,control_arm_data
    if (begin_close == 1):
        control_arm_data.position[1]=0
        control_arm_data.velocity[1]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(10)
        rospy.loginfo("Close!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        rospy.set_param("params_finish", 1)
        # control_arm_data.position[0]=0
        # control_arm_data.velocity[1]=1
        # arm_action_pub.publish(control_arm_data)
        # rospy.sleep(5)


def reset():
    global begin_close,begin_open,control_arm_data
    begin_open = 0
    begin_close = 0
    rospy.loginfo("reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    rospy.sleep(2)


#主体
if __name__=="__main__":
    rospy.init_node("general_service_grab")
    open_sub=rospy.Subscriber("/home_service/move_robot_arm",String,start_open,queue_size=10)
    close_sub=rospy.Subscriber("/home_service/move_robot_arm",String,start_close,queue_size=10)
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    open()
    close()
    reset()
    time1 = time.time()
    rospy.spin()


#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data



