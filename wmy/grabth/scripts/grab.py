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

machine_speed=Twist()

# begin_open = 0
# begin_close = 0

begin_throw = 0
  
#张开夹爪
def grab(msg:String):
    global control_arm_data,machine_speed,begin_throw
    if(msg.data == "1"):
        rospy.loginfo("Open!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0.58
        control_arm_data.velocity[0]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(6)
        arm_state_pub.publish("1")
    elif(msg.data == "2"): 
        control_arm_data.position[1]=0
        control_arm_data.velocity[1]=0.5
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(4)
        control_arm_data.position[0]=0.5
        control_arm_data.velocity[0]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(4)
        arm_state_pub.publish("1")


#主体
if __name__=="__main__":
    rospy.init_node("general_service_grab")
    rospy.loginfo("-----i am grab------")
    time1 = time.time()
    grab_sub=rospy.Subscriber("move_robot_arm",String,grab)


    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    arm_state_pub = rospy.Publisher("move_robot_arm_reply",String,queue_size=10)
    
    rospy.spin()