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
        # control_arm_data.position[1]=0
        # control_arm_data.velocity[1]=0.5
        arm_action_pub.publish(control_arm_data)
        #rospy.loginfo("------ begin grab 1 sleep------------")
        rospy.sleep(4)
        #rospy.loginfo("------ end grab 1 sleep------------")
        # control_arm_data.position[1]=0.5
        # control_arm_data.velocity[1]=0.5
        # arm_action_pub.publish(control_arm_data)
        # rospy.sleep(10)
        arm_state_pub.publish("1")
    elif(msg.data == "2"): 
        control_arm_data.position[1]=0
        control_arm_data.velocity[1]=0.5
        arm_action_pub.publish(control_arm_data)
        #rospy.loginfo("------ begin suo 2 sleep------------")
        rospy.sleep(4)
        #rospy.loginfo("------ end suo 2 sleep------------")
        #rospy.loginfo("Close!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0.5
        control_arm_data.velocity[0]=1
        arm_action_pub.publish(control_arm_data)
        #rospy.loginfo("------ begin sleep 4 second------------")
        rospy.sleep(4)
        #rospy.loginfo("------ end sleep 4 second------------")
        arm_state_pub.publish("1")
    elif(msg.data == "3"):
        # begin_throw = 1
        start_spin_pub.publish("spin")
        
def start_forward(msg):
    global begin_to_confirm_all_position_data,begin_throw
    if (msg=="spin ok"):
        start_get_distance_pub.publish("forward")
    rospy.loginfo("ok I will get the diatance")

def go_ahead(msg):
    global machine_speed,begin_throw,have_left
    have_left = msg
    machine_speed.linear.x=(have_left-0.55)/2 # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向
    machine_speed.linear.y=0
    machine_speed.linear.z=0
    machine_speed.angular.x=0
    machine_speed.angular.y=0
    machine_speed.angular.z=0
    rospy.loginfo("continue move to the destination~")
    speed_of_pub.publish(machine_speed)
    rospy.sleep(2)
    rospy.loginfo("arrive!!!")
    reset_speed()
    time.sleep(0.5)
    begin_throw = 1    
        
        
def throw_the_object():   
    global control_arm_data   
    control_arm_data.position[0]=0.58
    control_arm_data.velocity[0]=1
    control_arm_data.velocity[1]=0
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    control_arm_data.position[1]=0.5
    control_arm_data.velocity[1]=0.5
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    control_arm_data.position[0]= 0.5 #下降
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    rospy.loginfo("finish throw the trash!!!")
    down_state_pub.publish("0")


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



    #rospy.set_param("params_finish", 1)
        # get_state_pub.publish("0")
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
    rospy.loginfo("-----i am grab------")
    time1 = time.time()
    grab_sub=rospy.Subscriber("move_robot_arm",String,grab)
    
    start_spin_pub=rospy.Publisher("/home_service/robot_spin",String,queue_size=10)#begin spin
    start_forward_sub=rospy.Subscriber("/home_service/robot_spin_reply",String,start_forward,queue_size=10)#spin ok
    
    start_get_distance_pub=rospy.Publisher("/home_service/lidar_distance",String,queue_size=10) #begin to get distance
    get_distance_sub=rospy.Subscriber("/home_service/robot_getdistance",String,go_ahead,queue_size=10)#get distance and move
    
    down_state_pub=rospy.Publisher("/home_service/genenal_service_put_down_result",String,queue_size=10)#grab.py
    

    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    arm_state_pub = rospy.Publisher("move_robot_arm_reply",String,queue_size=10)
    get_state_pub=rospy.Publisher("genenal_service_get_it",String,queue_size=10)#grab.py
    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10) #控制机器人线速度和角速度
    down_state_pub=rospy.Publisher("genenal_service_put_down_result",String,queue_size=10)#grab.py
    
    control_all_the_action_of_machine_throw()
    
    #reset()
    
    rospy.spin()


#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data



