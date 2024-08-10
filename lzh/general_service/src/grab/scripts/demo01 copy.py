#! /usr/bin/env python3
from re import S
from selectors import SelectorKey
import string
import rospy
import json
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math
import message_filters
import cv_bridge
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
image,depth=None,None
class machince_grab:
    def __init__(self) -> None:
        # self.sub=rospy.Subscriber("yolo_result",10,self.yolocallback)
        self.canshu=None
        # self.Sever=rospy.Subscriber("objcet_need",String,self.callback_objectneed)
        self.pub_mani_ctrl=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30)
        self.pub_loc=rospy.Publisher('cmd_vel',Twist,queue_size=30)
        # self.camera_sub = rospy.Subscriber("/kinect2/qhd/camera_info", CameraInfo, self.InitCamera)
        self.sub_grab=rospy.Subscriber("generservice_start",String,self.process_grab)
        # self.sub_grab_position=rospy.Subscriber("odom",Odometry,self.position_process)
        self.ctrl_msg=JointState()
        self.ctrl_msg.name.append("lift")
        self.ctrl_msg.name.append("gripper")
        self.position_init=[0,0]
        self.ctrl_msg.position.extend(self.position_init)
        self.ctrl_msg.velocity.extend(self.position_init)
        self.switch_case=0
        self.W_img = 960 # 图像宽度
        self.H_img = 540 # 图像高度
        self.P_x = self.W_img//2-1 # 图像中心点x坐标
        self.P_y = self.H_img//2-1 # 图像中心点y坐标
        self.f_x = 540.68603515625
        self.f_y = 540.68603515625
        self.u_img=None
        self.v_img=None
        self.label=None
        self.twist_msg=Twist()
        self.d=0
        self.X=0
        self.Y=0
        self.Z=0
        self.angle=0
        # rospy.spin()
    def callback_objectneed(self,msg):
        self.label=msg.data
    def grab_control(self):
        while( not rospy.is_shutdown()):
            rospy.loginfo(self.switch_case)
            if self.switch_case==4:
                break
            elif self.switch_case==0:
                self.ctrl_msg.position[0]=0.5
                self.ctrl_msg.velocity[0]=0.5
                self.ctrl_msg.position[1]=0.1
                self.ctrl_msg.velocity[1]=5
                self.switch_case=1
                rate=rospy.Rate(0.2)
                rate.sleep()
                self.pub_mani_ctrl.publish(self.ctrl_msg)
                continue
            elif self.switch_case==1:
                self.ctrl_msg.position[0]=1.1
                self.ctrl_msg.velocity[0]=1.0
                self.ctrl_msg.position[1]=0
                self.ctrl_msg.velocity[1]=5
                self.switch_case=2
                self.pub_mani_ctrl.publish(self.ctrl_msg)
                rate=rospy.Rate(0.1)
                rate.sleep()
                break
            elif self.switch_case==2:
                self.ctrl_msg.position[0]=1.1
                self.ctrl_msg.velocity[0]=0.5
                self.ctrl_msg.position[1]=0
                self.ctrl_msg.velocity[1]=0.5
                self.switch_case=3
                self.pub_mani_ctrl.publish(self.ctrl_msg)
                rate.sleep()
                continue
            elif self.switch_case==3:
                self.ctrl_msg.position[0]=0
                self.ctrl_msg.velocity[0]=1
                self.ctrl_msg.position[1]=0.1
                self.ctrl_msg.position[1]=5
                self.pub_mani_ctrl.publish(self.ctrl_msg)
                self.switch_case=4
            rospy.loginfo("adssasadds")
    def location_angel(self):

        self.twist_msg.linear.x=0
        self.twist_msg.linear.y=0
        self.twist_msg.linear.z=0
        self.twist_msg.angular.x=0
        self.twist_msg.angular.y=0
        self.wist_msg.angular.z=1
        rate=rospy.Rate(3)
        while(not rospy.is_shutdown()):
            self.pub_loc.publish(twist_msg)
            
            rate.sleep()

    def process_grab(self,msg):
        if (msg.data=="start"):
            self.switch_case=1
        elif (msg.data=="over"):
            self.switch_case=3
        self.grab_control()
    def position_process(self,odom):
        grab_x=odom.pose.pose.position.x
        grab_y=odom.pose.pose.position.y
        grab_z=odom.pose.pose.position.z
        print(grab_x,grab_y,grab_z)


rospy.init_node("general_service_grab")
# rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color", Image)
# depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
# ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
# ts.registerCallback(image_cb)
mg=machince_grab()
# mg.grab_control()
# mg.location_processing()
# mg.grab_control()
mg.location_angel()
# yolo_sub=rospy.Subscriber("yolo_result",10,yolocallback)
rospy.spin()
