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
#from pose_estimate.scripts.pose import task2
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
""" 机械臂控制 """
control_arm_data=JointState()#变量的类型
arm_postion_init=[0,0]#初始化
control_arm_data.name.append("lift")#[0]控制机械臂上下
control_arm_data.name.append("gripper")#[1]控制爪夹开和
control_arm_data.position.extend(arm_postion_init)#机械臂位置初始化
control_arm_data.velocity.extend(arm_postion_init)#速度初始化

machine_speed=Twist()#变量类型，控制机器人速度

  
""" 抓取操作 """
def grab(msg:String):
    global control_arm_data,machine_speed
    if(msg.data == "1"):#机械臂上升
        rospy.loginfo("Open!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0.58#机械臂上升到0.58米
        control_arm_data.velocity[0]=1#机械臂移动速度为1米每秒
        arm_action_pub.publish(control_arm_data)#发布机械臂控制信息
        rospy.sleep(5)#等待5s使操作完成
        arm_state_pub.publish("1")#给主函数返回1说明动作完成
    elif(msg.data == "2"): #夹取并放下
        control_arm_data.position[1]=0#爪夹间距为0
        control_arm_data.velocity[1]=0.5#爪夹闭合速度为0.5m/s
        arm_action_pub.publish(control_arm_data)
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(1.5)
        control_arm_data.position[0]=0.5 #下降到0.5m，若低于0.5m则再上升时爪夹会自动打开
        control_arm_data.velocity[0]=1.2
        arm_action_pub.publish(control_arm_data)
        arm_action_pub.publish(control_arm_data)#发布机械臂控制信息
        rospy.sleep(6)
        print("-------finish putdown------------------")
        arm_state_pub.publish("1") #给主函数返回1说明动作完成


#主体
if __name__=="__main__":
    rospy.init_node("general_service_grab") #节点初始化
    rospy.loginfo("-----i am grab------")
    time1 = time.time()
    grab_sub=rospy.Subscriber("move_robot_arm",String,grab) #与主函数通信


    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    arm_state_pub = rospy.Publisher("move_robot_arm_reply",String,queue_size=10) #发布动作完成的消息
    
    rospy.spin()