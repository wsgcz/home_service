#! /usr/bin/env python3
from ast import arg
from cProfile import label
from configparser import RawConfigParser
import enum
from multiprocessing import Process,Pool
from random import randrange
# from re import S, T
from selectors import SelectorKey
import string
from tkinter import N
from tkinter.messagebox import NO
from urllib import robotparser
from xml.dom.minidom import ReadOnlySequentialNamedNodeMap
import xxlimited
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
from waterplus_map_tools.srv import GetWaypointByName, GetWaypointByNameRequest, GetWaypointByNameResponse
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import asyncio
image,depth=None,None
import threading
import tf
import tf.transformations as transformations
import threading
from judge_arrive.msg import judge_arrive
from std_msgs.msg import Int32
all_count=0
sys.path.append('/home/linxi/ServiceRobot-General/src/yolov5-in-ros/scripts')
for i in range(len(sys.path)):
    if "2.7" in sys.path[i]:
        sys.path.append(sys.path.pop(i))
        break
from libyolo import YoloV5s
start_guide_time=time.time()
Y=-1
xmin=2
p=Pool(10)
number_image=0
now_have_been_middle_place=0
mutrex=threading.Lock()
mutrex1=threading.Lock()
# all_room_corner标准为minx,miny，maxx,maxy
all_room_corner={"kitchen":[4.54,-3.092,10.3121,0.629],"living":[1,1,1,1],"bedroom":[-0.498,0.412,2.103,4.647],"dining_room":[1,1,1,1]}
all_room_y={"kitchen":-1.74,"bedroom":2.487}
# all_room_center={"kitchen":}
all_object_need=[]
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


class machince_grab:
    def __init__(self) -> None:
        self._cv_bridge = cv_bridge.CvBridge()
        self.sub_grab=rospy.Subscriber("general_service_need",String,self.object_need)
        self.yolo=YoloV5s()
        self.pub_loc=rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfSub = tf2_ros.TransformListener(self.tfBuffer)
        # self.rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color", Image)
        # self.depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
        self.to_grab_pub=rospy.Publisher("start_grab",String,queue_size=10)
        # self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.sub_grab_position=rospy.Subscriber("odom",Odometry,self.position_process)
        self.judge_arrive_pub=rospy.Publisher("judge_arrive_target",judge_arrive,queue_size=10)
        self.judge_arrive_sub=rospy.Subscriber("judge_arrive_result",Int32,self.sub_judge_arrive_result,queue_size=10)
        self.to_grab_pub=rospy.Publisher("start_grab",String,queue_size=10)
        self.grab_room_sub=rospy.Subscriber("grab_room",String,self.grab_room_cb,queue_size=10)
        self.all_name_sub=rospy.Subscriber("all_name",String,self.all_name_grab_cb,queue_size=10)
        self.W_img = 960 # 图像宽度
        self.H_img = 540 # 图像高度
        self.P_x = self.W_img//2-1 # 图像中心点x坐标
        self.P_y = self.H_img//2-1 # 图像中心点y坐标
        self.f_x = 540.68603515625
        self.f_y = 540.68603515625
        self.twist_msg=Twist()
        self.label=None
        self.begin_everything=0   
        self.judge_arrive_result=[]
        self.odom_py=0
        self.odom_pz=0
        self.odom_ox=0
        self.odom_oy=0
        self.odom_oz=0
        self.odom_ow=0
        self.mathince_theta=0
        self.togetname=rospy.ServiceProxy("/waterplus/get_waypoint_name",GetWaypointByName)#robot
        self.target_count=0
        self.pianzhi=0
        self.grab_room_name=None
        self.image=None
        self.depth=None
        # self.ts.registerCallback(self.image_cb)
        self.yolo_result=None
        self.rotate_time=time.time()
        self.rotate_count=0
        self.srv_rqs=GetWaypointByNameRequest()
        self.srv_rsp=GetWaypointByNameResponse()
        self.goal=MoveBaseGoal()
        rospy.spin()
    def all_name_grab_cb(self,msg:String):
        global all_object_need
        all_object_need.append(msg.data)
    def sub_judge_arrive_result(self,result):
        self.judge_arrive_result.append(result.data)
        rospy.loginfo("第%d点 %d",len(self.judge_arrive_result),result.data)
    def grab_room_cb(self,msg:String):
        self.grab_room_name=msg.data 
    def real_pose(self,u_img, v_img, d):
        other_angle=(v_img-270)*43/540
        this_angle=0.47*180/math.pi
        now_angle=abs(this_angle+other_angle)*math.pi/180
        now_angle1=abs(other_angle)*math.pi/180
        
        
        Z = d/math.sqrt(1+(u_img-self.P_x)**2/self.f_x**2+(v_img-self.P_y)**2/self.f_y**2)
        X = (u_img-self.P_x)*Z/self.f_x
        Y = (v_img-self.P_y)*Z/self.f_y
        y=(Y/1000)
        print("this is ",y)
        zzz= d/1000*math.cos(now_angle)+0.24-y*math.sin(now_angle1)
        # yyy=1.62-d/1000*math.sin(0.4)+y*math.cos(0.4)
        return X/1000+0.015, y, zzz  
    def real_pose1(self,u_img, v_img, d):
        Z = d/math.sqrt(1+(u_img-self.P_x)**2/self.f_x**2+(v_img-self.P_y)**2/self.f_y**2)
        X = (u_img-self.P_x)*Z/self.f_x
        Y = (v_img-self.P_y)*Z/self.f_y

        y=(Y/1000)
        zzz= d/1000*math.cos(0.4)-y*math.sin(0.4)-0.03
        yyy=1.71-d/1000*math.sin(0.4)-y*math.cos(0.4)
        self.the_bottle_need_y=yyy
        rospy.loginfo("this is xx,yyy,zzz%f,%f,%f",X/1000,yyy,zzz)
        return X/1000, yyy,  zzz     
    def turn_right_or_left(self,z):

        self.twist_msg.linear.x=0
        self.twist_msg.linear.y=0
        self.twist_msg.linear.z=0
        self.twist_msg.angular.x=0
        self.twist_msg.angular.y=0
        self.twist_msg.angular.z=z
        # rospy.loginfo("have reset speed")   
    def position_process(self,odom:Odometry):
        self.odom_px=odom.pose.pose.position.x
        # rospy.loginfo("this is odom_px%f",self.odom_px)
        self.odom_py=odom.pose.pose.position.y
        self.odom_pz=odom.pose.pose.position.z
        self.odom_ox=odom.pose.pose.orientation.x
        self.odom_oy=odom.pose.pose.orientation.y
        self.odom_oz=odom.pose.pose.orientation.z
        self.odom_ow=odom.pose.pose.orientation.w
        # rospy.loginfo("this is odom_msg%f,%f,%f,%f",self.odom_oz,self.odom_ow,self.odom_oy,self.odom_ox)
        
        _,_,self.mathince_theta=transformations.euler_from_quaternion([self.odom_ox,self.odom_oy,self.odom_oz,self.odom_ow])      
    def get_map_pose(self,x, y):
        ps = PointStamped()
        ps.header.frame_id = "odom"
        ps.point.x = x
        ps.point.y = y
        ps.header.stamp = rospy.Time.now()
        ps_new:PointStamped
        ps_new = self.tfBuffer.transform(ps, "map", rospy.Duration(1))
        return ps_new.point.x, ps_new.point.y
    def object_need(self,msg:String):
        global start_guide_time
        self.label=msg.data
        self.begin_everything=1
        self.twist_msg=Twist()
        # self.label=None
        # self.begin_everything=1 
        self.pianzhi=0
        self.mathince_theta=0

        self.target_count=0
        rospy.loginfo("have accepted message")
        start_guide_time=time.time()
        self.rotate_count=0
        self.do_pos()
    def get_map_pose_theta(self):
        ps = PoseStamped()
        ps.header.frame_id = "base_footprint"
        ps.pose.position.x = 0.0
        ps.pose.position.y = 0.0
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.header.stamp = rospy.Time.now()
        ps_new = self.tfBuffer.transform(ps, "map", rospy.Duration(1))
        return [ps_new.pose.position.x, ps_new.pose.position.y, ps_new.pose.orientation.z, ps_new.pose.orientation.w]           

    def stop(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.linear.y = 0
        vel_cmd.linear.z = 0
        vel_cmd.angular.x = 0
        vel_cmd.angular.y = 0
        vel_cmd.angular.z = 0
        self.pub_loc.publish(vel_cmd)    
    def image_cb(self,image_rgb,image_depth):
        global now_have_been_middle_place,all_count
        if self.begin_everything==0:
            return
        if rospy.is_shutdown():
            return
        self.judge_rotate_time=rospy.Time.now().to_sec()
        self.image = self._cv_bridge.imgmsg_to_cv2(image_rgb, desired_encoding='passthrough')
        self.depth = self._cv_bridge.imgmsg_to_cv2(image_depth, desired_encoding='passthrough')
        self.yolo_result,_=self.yolo.inference(img=self.image, if_show=True)
        # cv2.imshow("img",self.image)
        # cv2.waitKey(0)
        # cv2.imwrite("/home/linxi/ServiceRobot-General/src/grab/image/"+str(all_count)+".jpg",self.image)
        all_count+=1
        rospy.loginfo("%s",self.yolo_result)
        
        time.sleep(0.02)
        # task2=threading.Thread(target=self.do_image_task,args=(image_rgb,image_depth,))
        # task2.start()
    def do_image_task(self,image_rgb,image_depth):
        self.judge_rotate_time=rospy.Time.now().to_sec()
        self.image = self._cv_bridge.imgmsg_to_cv2(image_rgb, desired_encoding='passthrough')
        self.depth = self._cv_bridge.imgmsg_to_cv2(image_depth, desired_encoding='passthrough')
        self.yolo_result,_=self.yolo.inference(img=image, if_show=True)
    def do_pos(self):
        if self.grab_room_name=="kitchen":
            self.Gotopoint("kitchen_grab")
        elif self.grab_room_name=="bedroom":
            self.Gotopoint("bedroom_grab")
        data=String("1")
        self.to_grab_pub.publish(data)
    def Gotopoint(self,position):
        # global Ahead,Client
        self.srv_rqs.name=position
        # print(position)
        if (self.togetname.call(self.srv_rqs)):
            if (self.ac.wait_for_server(rospy.Duration(5.0))==False): 
                    rospy.loginfo("The move_base action server is no running. action abort...")
                    return False
            else:
                    self.srv_rsp=self.togetname.call(self.srv_rqs)
                    rospy.loginfo("get_waypoint_name: name = %s (%.2lf,%.2lf),"
                        ,position,
                        self.srv_rsp.pose.position.x,
                        self.srv_rsp.pose.position.y)
                    self.goal.target_pose.header.frame_id = "map"
                    self.goal.target_pose.header.stamp = rospy.Time.now()
                    self.goal.target_pose.pose = self.srv_rsp.pose
                    self.ac.send_goal(self.goal)
                    
                    self.ac.wait_for_result()
                    if self.ac.get_state()==3:
                        rospy.loginfo("Arrived at %s!", position)
                        return True
                    else:
                        print(self.ac.get_state())
                        rospy.loginfo("Failed to get to %s ...", position)
                        return False
        else:
            rospy.logwarn("Failed to call service GetWaypointByName")
            return False

if __name__=="__main__":
    rospy.init_node("general_service_my_navigation")
    # rospy.Subscriber("start_navigate",String,open_navigate,queue_size=10)
    mg=machince_grab()

    # mg.pub_loc.publish(mg.twist_msg)
    rospy.loginfo("ASdadssadsadad")

    rospy.spin()

