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

begin_open = 0
begin_close = 0
  
#张开夹爪
def grab(msg:String):
    global control_arm_data,machine_speed
    if(msg.data == "1"):
        rospy.loginfo("Open!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        control_arm_data.position[0]=0.6
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
        control_arm_data.position[0]=0.6
        control_arm_data.velocity[0]=1
        control_arm_data.velocity[1]=0
        arm_action_pub.publish(control_arm_data)
        #rospy.loginfo("------ begin throw 3 sleep------------")
        rospy.sleep(4)
        #rospy.loginfo("------ end throw 3 sleep------------")
        control_arm_data.position[1]=0.5
        control_arm_data.velocity[1]=0.5
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(4)
        # machine_speed.linear.x=-0.1 # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向
        # machine_speed.linear.y=0
        # machine_speed.linear.z=0
        # machine_speed.angular.x=0
        # machine_speed.angular.y=0
        # machine_speed.angular.z=0
        # speed_of_pub.publish(machine_speed)
        # rospy.sleep(5)
        # reset_speed()
        control_arm_data.position[0]= 0.5 #下降
        # rospy.loginfo("this is final_y %f",final_position_y)
        control_arm_data.velocity[0]=1
        arm_action_pub.publish(control_arm_data)
        rospy.sleep(4)
        rospy.loginfo("finish throw the trash!!!")
        down_state_pub.publish("0")


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
    
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    arm_state_pub = rospy.Publisher("move_robot_arm_reply",String,queue_size=10)
    get_state_pub=rospy.Publisher("genenal_service_get_it",String,queue_size=10)#grab.py
    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10) #控制机器人线速度和角速度
    down_state_pub=rospy.Publisher("genenal_service_put_down_result",String,queue_size=10)#grab.py
    
    #reset()
    
    rospy.spin()


#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data



