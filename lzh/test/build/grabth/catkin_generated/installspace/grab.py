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
  
#张开夹爪
def grab(msg:String):
    global control_arm_data
    if(msg == "1"):
        rospy.loginfo("Open!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0.6
        control_arm_data.velocity[0]=0.5
        control_arm_data.position[1]=0
        control_arm_data.velocity[1]=0
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(5)
        control_arm_data.position[1]=0.5
        control_arm_data.velocity[1]=2
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(5)
        arm_state_pub.publish("1")
    elif(msg == "0"): 
        control_arm_data.position[1]=0
        control_arm_data.velocity[1]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(7)
        arm_state_pub.publish("1")
        rospy.loginfo("Close!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0
        control_arm_data.velocity[0]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(7)
        get_state_pub.publish("0")
        #rospy.set_param("params_finish", 1)
    
    
        #rospy.set_param("params_finish", 1)
        # control_arm_data.position[0]=0
        # control_arm_data.velocity[1]=1
        # arm_action_pub.publish(control_arm_data)
        # rospy.sleep(5)


# def reset():
#     global begin_close,begin_open,control_arm_data
#     begin_open = 0
#     begin_close = 0
#     rospy.loginfo("reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     rospy.sleep(2)


#主体
if __name__=="__main__":
    rospy.init_node("general_service_grab")
    grab_sub=rospy.Subscriber("/home_service/move_robot_arm",String,grab,queue_size=10)
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    arm_state_pub = rospy.Publisher("/home_service/move_robot_arm_reply",String,queue_size=10)
    get_state_pub=rospy.Publisher("/home_service/genenal_service_get_it",String,queue_size=10)#grab.py
    #reset()
    time1 = time.time()
    rospy.spin()


#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data



