from concurrent.futures import BrokenExecutor
from tkinter import Y
from unicodedata import name
# from general_service_2022.scripts.main_2_8_18 import Gotogoal
import rospy
from nav_msgs.msg import Odometry
import tf2_ros
import tf.transformations as transformations
from std_msgs.msg import Int32
from tf2_geometry_msgs import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from waterplus_map_tools.srv import GetWaypointByName,GetWaypointByNameRequest,GetWaypointByNameResponse
from judge_arrive.msg import judge_arrive
from std_msgs.msg import String
import time
import threading
start_time=time.time()
count_of_three=0
count_of_two=0

class client:
    def __init__(self) -> None:
        self.go_to_flag=False
        self.togetname=rospy.ServiceProxy("/waterplus/get_waypoint_name",GetWaypointByName)#robot
        self.ac=actionlib.SimpleActionClient("move_base",MoveBaseAction)
class ahead:
   def __init__(self) -> None:
        self.goal=MoveBaseGoal()
        self.goal_renew_flag={}
        self.goals_dit={}
        self.goals_list=[]
        self.index=0
        self.goals_finish=False
        self.current_goals_index=0
        self.srv_rqs=GetWaypointByNameRequest()
        self.srv_rsp=GetWaypointByNameResponse()
        self.new_position={}
        self.msg=String()
def Gotopoint(position):
   global Ahead,Client,count_of_three,count_of_two
   Ahead.srv_rqs.name=position
   print(position)
   if (Client.togetname.call(Ahead.srv_rqs)):
      if (Client.ac.wait_for_server(rospy.Duration(5.0))==False): 
            rospy.loginfo("The move_base action server is no running. action abort...")
            return False
      else:
            Ahead.srv_rsp=Client.togetname.call(Ahead.srv_rqs)
            rospy.loginfo("get_waypoint_name: name = %s (%.2lf,%.2lf),"
                ,position,
                Ahead.srv_rsp.pose.position.x,
                Ahead.srv_rsp.pose.position.y)
            Ahead.goal.target_pose.header.frame_id = "map"
            Ahead.goal.target_pose.header.stamp = rospy.Time.now()
            Ahead.goal.target_pose.pose = Ahead.srv_rsp.pose
            Client.ac.send_goal(Ahead.goal)
            switch_on_pub.publish(Ahead.goal)
            Client.ac.wait_for_result()
            last_time=time.time()
            
            if Client.ac.get_state()==3:
                rospy.loginfo("Arrived at %s!", position)
                data=String()
                data.data="1"
                switch_over_pub.publish(data)
                return True
            else :
                while (over_flag):
                    pass
            # else:
            #     rospy.loginfo("this is state %d",Client.ac.get_state())
            #     rospy.loginfo("Failed to get to %s ...", position)
            #     rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1 %s ...", position)
                
            #     return False
   else:
      rospy.logwarn("Failed to call service GetWaypointByName")
      return False



switch=0
odom_px=0
odom_py=0
odom_pz=0
odom_ox=0
odom_oy=0
odom_oz=0
odom_ow=0
x=0.06
y=0
mathince_theta=0
last_mathince_theta=0

judge_arrive_result=[]
Ahead=ahead()        
Client=client()
start_time=time.time()
over_flag=1
def off_cb(msg:String):
    global over_flag
    over_flag=0
if __name__=="__main__":
    rospy.init_node("eme_pub")
    switch_on_pub=rospy.Publisher("emergency_switch_on_1",MoveBaseGoal,queue_size=10)
    switch_off=rospy.Subscriber("emergency_switch_off_1",String,off_cb,queue_size=10)
    switch_over_pub=rospy.Publisher("emergency_switch_off_2",String,queue_size=10)
    time.sleep(5)
    Gotopoint("targets_kitchen")
    rospy.loginfo("this is over main")
    