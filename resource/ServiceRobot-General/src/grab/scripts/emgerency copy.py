# from general_service_2022.scripts.main_2_8_18 import Ahead
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
   global Ahead,Client
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
            
            if Client.ac.get_state()==3:
                rospy.loginfo("Arrived at %s!", position)
                return True
            else:
                print(Client.ac.get_state())
                rospy.loginfo("Failed to get to %s ...", position)
                return False
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

def rpy2quaternion(roll, pitch, yaw):
    x = math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    y = math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    z = math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    w = math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    return [x, y, z, w]

def get_map_pose(x, y):
        ps = PointStamped()
        ps.header.frame_id = "odom"
        ps.point.x = x
        ps.point.y = y
        ps.header.stamp = rospy.Time.now()
        ps_new = tfBuffer.transform(ps, "map", rospy.Duration(1))
        return ps_new.point.x, ps_new.point.y

def switch_on_cb(target):
    global odom_px,odom_py,switch,x,y,start_time,mathince_theta,last_mathince_theta,start_time
    switch=1
    # start_time=time.time()
    judge_break=0
    rospy.loginfo(1111111111111111)
    while(switch==1):
        # rospy.sleep(5)
        time.sleep(1)
        rospy.loginfo("insight odom_x,y,x,y %f,%f,%f,%f",odom_py,odom_px,x,y)
        if (( abs( x-odom_px ) < 0.05 and abs( y-odom_py ) < 0.05  and abs(mathince_theta-last_mathince_theta)<0.01) and switch ==1 ):
            rospy.loginfo("have entered insight")
            judge_arrive_result=[]
            machine_x,machine_y=get_map_pose(odom_px,odom_py)
            reach_place1=[( machine_x, machine_y+0.4,math.pi/2),( machine_x, machine_y-0.2,-math.pi/2),( machine_x+0.2, machine_y,math.pi),( machine_x-0.2, machine_y,0)]
            for i in range(4):
                (x1,y1,theta1) = reach_place1[i]
                p=judge_arrive()
                p.x=x1
                p.y=y1
                judge_arrive_pub.publish(p)
                
            while(True):
                if(len(judge_arrive_result)!=4):
                    break
                for i in range(4):
                    if judge_arrive_result[i]<253:
                        (x1,y1,theta1) = reach_place1[i]
                        break
                break
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = x1
            goal.target_pose.pose.position.y = y1
            xyzw = rpy2quaternion(0.0, 0.0, theta1)
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = xyzw[2]
            goal.target_pose.pose.orientation.w = xyzw[3]
            goal.target_pose.header.stamp=rospy.Time.now()
            
            try:
                ac.send_goal(goal)
                ac.wait_for_result()
                start_time=time.time()
                if ac.get_state()==actionlib.SimpleGoalState.ACTIVE:
                    rospy.loginfo("active")
                elif ac.get_state()==3:    
                    judge_break=1
                elif ac.get_state()==actionlib.SimpleGoalState.PENDING:
                    rospy.loginfo("now it is pending")
                else :
                    rospy.loginfo("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz454654545644444444444444")
                    pass
            except BaseException as E:
                rospy.loginfo("can not go")
                break
                #前往房间的设定点

        if judge_break:
            rospy.loginfo("have break")
            break   
    if switch!=2:
        task=threading.Thread(target=wait_task,args=(ac,target,))
        task.start()

def wait_task(ac,target):
    global switch
    ac.send_goal(target)
    switch_on_pub.publish(target)
    ac.wait_for_result() 
    if ac.get_state()==3: 
        rospy.loginfo("this is break wait_task")  
        switch=2
        data=String()
        data.data='0'
        switch_off.publish(data)
        
        
def sub_judge_arrive_result(result):
        judge_arrive_result.append(result.data)
        rospy.loginfo("第%d点 %d",len(judge_arrive_result),result.data)

def process(odom):
        global odom_py,odom_px,x,y,start_time,mathince_theta,last_mathince_theta
        now_time=time.time()
        odom_px=odom.pose.pose.position.x
        odom_py=odom.pose.pose.position.y
        odom_pz=odom.pose.pose.position.z
        odom_ox=odom.pose.pose.orientation.x
        odom_oy=odom.pose.pose.orientation.y
        odom_oz=odom.pose.pose.orientation.z
        odom_ow=odom.pose.pose.orientation.w
        # rospy.loginfo("this is odom_msg%f,%f,%f,%f",self.odom_oz,self.odom_ow,self.odom_oy,self.odom_ox)
        _,_,mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow])  
        now_time=time.time()
        if now_time-start_time>15:
            rospy.loginfo("refresh message %f",time.time())
            start_time=now_time
            x=odom_px
            y=odom_py
            last_mathince_theta=mathince_theta
            rospy.loginfo(" odom x_y%f,%f,%f,%f",odom_px,odom_py,x,y)

def over_switch_cb(msg):
    global switch
    switch=2         
if __name__=="__main__":
    rospy.init_node("emergency_1")
    # rospy.sleep(5)
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)   
    sub_odom=rospy.Subscriber("odom",Odometry,process,queue_size=10)
    switch_on_sub=rospy.Subscriber("emergency_switch_on_1",MoveBaseGoal,switch_on_cb,queue_size=10)
    switch_on_pub=rospy.Publisher("emergency_switch_on_1",MoveBaseGoal,queue_size=10)
    switch_off=rospy.Publisher("emergency_switch_off_1",String,queue_size=10)
    judge_arrive_pub=rospy.Publisher("judge_arrive_target",judge_arrive,queue_size=10)
    judge_arrive_sub=rospy.Subscriber("judge_arrive_result",Int32,sub_judge_arrive_result,queue_size=10)
    over_sub=rospy.Subscriber("emergency_switch_off_2",String,over_switch_cb,queue_size=10)
    # Gotopoint("targets_kitchen")
    rospy.spin()