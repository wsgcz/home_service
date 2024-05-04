from enum import Flag
from pyexpat.errors import XML_ERROR_DUPLICATE_ATTRIBUTE
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import atan2, pi, sqrt, asin, sin, cos
import json
import time
import math
import numpy as np
from move_base_msgs.msg import MoveBaseGoal
from tf2_geometry_msgs import PointStamped, PoseStamped
import tf2_ros
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from general_service_2022.msg import the_way_out
postion_x=0
postion_y=0
postion_theta=0
def get_map_pose(x, y,frame_id,target,time):
    ps = PointStamped()
    ps.header.frame_id = frame_id
    ps.point.x = x
    ps.point.y = y
    ps.header.stamp =time
    ps_new = tfBuffer.transform(ps, target, rospy.Duration(1))
    return ps_new.point.x, ps_new.point.y

def rpy2quaternion(roll, pitch, yaw):
    x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    return [x, y, z, w]
def get_map_pose_theta():
    ps = PoseStamped()
    ps.header.frame_id = "laser"
    ps.pose.position.x = 0.0
    ps.pose.position.y = 0.0
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = 1.0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = 0.0
    ps.header.stamp = rospy.Time.now()
    ps_new = tfBuffer.transform(ps, "map", rospy.Duration(2))
    return [ps_new.pose.position.x, ps_new.pose.position.y, ps_new.pose.orientation.z, ps_new.pose.orientation.w]

class Mymsg:
    def __init__(self) -> None:
        self.frame_id=None
        self.angle=None
        self.secs_time=None
        self.nsecs_time=None
        self.length=None
        self.now_time=None

def process_msg(msg:Mymsg):
    x=msg.length*cos(msg.angle)
    y=msg.length*sin(msg.angle)
    return x,y,
def domsg(msg:LaserScan):
    now_time1=rospy.Time.now().to_sec()
    now_time2=rospy.Time.now().to_nsec()
    now_time=rospy.Time.now()
    frame_id=msg.header.frame_id
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    angle_increment=msg.angle_increment
    time_increment=msg.time_increment
    scan_time=msg.scan_time
    the_range=msg.ranges
    length=len(the_range)
    # for i in the_range:
    #     rospy.loginfo("%f",i)
    # angle_lis=[]
    Msg_list=[]
    for i in range(length):
        temp=Mymsg()
        if the_range[i]==math.inf:
            # print("1111111111111")
            pass
        else :
            temp.angle=angle_min+i*angle_increment
            temp.frame_id=frame_id
            temp.length=the_range[i]
            temp.secs_time=int(now_time1+i*time_increment)
            temp.nsecs_time=int(now_time2+i*1000*time_increment)
            temp.now_time=now_time+rospy.Duration(0,int(1000*i*time_increment))
            Msg_list.append(temp)
            # if -0.05<temp.angle<0.05:
            #     rospy.loginfo(temp.length)
    real_position_x=[]
    real_position_y=[]
    data=the_way_out()
    data.the_angle_of_max_length=0
    max_length=0
    max_angle=0
    number=0
    for k,i in enumerate(Msg_list):
        # temp=position_data()
        x,y=process_msg(i)
        # try:
        if i.length>0.5:
            data.angle+=1
            number+=k
        real_x,real_y=get_map_pose(x,y,"laser","base_link",i.now_time)
        # real_x,real_y=get_map_pose(x,y,"base_link","map")
        # except BaseException as E:
        theta=math.atan(real_y/real_x)
        # print(i.length)
        # rospy.loginfo(i.length)
        # rospy.loginfo("_______________")
        real_x=i.length*cos(postion_theta+theta)+postion_x
        real_y=postion_y+i.length*cos(math.pi/2-theta-postion_theta)    
        # real_y=real_x
        real_position_x.append(x)
        real_position_y.append(y)
        if i.length>=max_length:
            # rospy.loginfo("yeas")
            max_length=i.length
            max_angle=i.angle
        if -0.05<i.angle<0.05:
            rospy.loginfo("%f,%f\n",real_x,real_y)
        # except BaseException as E:
        #     return
    rospy.loginfo("____________")
    data.angle+=(360-length)
    data.the_angle_of_max_length=int((max_angle/math.pi)*180)
    data.max_length=max_length
    if (not rospy.is_shutdown()):
        pos_pub.publish(data)
        rospy.loginfo("%d,%d,%d,%d",data.angle,data.the_angle_of_max_length,data.max_length,number)
        rospy.loginfo("yeah ,i have pubed")
def pose_react(msg:Pose2D):
    global postion_x,postion_y,postion_theta
    postion_x=msg.x
    postion_y=msg.y
    postion_theta=msg.theta
rospy.init_node("laser_of_map")
tfBuffer = tf2_ros.Buffer()
tfSub = tf2_ros.TransformListener(tfBuffer)
# transformListener=tf2_ros.transform_listener()
rospy.Subscriber("/scan",LaserScan,domsg)
pose_diff=rospy.Subscriber("/wpb_home/pose_diff",Pose2D,pose_react)
pos_pub=rospy.Publisher("/general_service_pose_pub",the_way_out,pose_react,queue_size=10)
rospy.spin()

