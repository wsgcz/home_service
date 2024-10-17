#!/usr/bin/env python3
# from re import T
# from grab.scripts.t import A
from multiprocessing.connection import wait
from operator import imod
from secrets import randbelow
from time import time
from typing import List
import rospy
import actionlib.simple_action_client 
import actionlib 
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
from waterplus_map_tools.srv import GetWaypointByName,GetWaypointByNameRequest,GetWaypointByNameResponse
#from general_service_2022.msg import Goals_name  # topic msg    contain name/goal
import pandas as pd
import numpy as np
import tf.transformations as transformations
from nav_msgs.msg import Odometry
import math
import time

# from xf_voice.scripts.word import refresh_cb
###################################################################
# 通常情况下我定义了结构体的变量都是为了状态机逻辑服务的如：Face_det.msg--- #
# 节点的消息操作封装在大类中：pub sub client。状态机类型定义为结构体Status #
# PDW:PreDefinedWaypoints 是初始预设的位置信息:厨房/客厅的信息--------- #
###################################################################

#！！！！！！！！！！！！！！！！！！！！！！！！！test=1为测试状态！！！！！！！！！！！！！！！！！！！！！！！！！！！

################
# 状态机类型定义 #
################
begin_time=time.time()
recode_temp=0
delete_flag=0
class Statu:
  def __init__(self) -> None:
        self.Enter = 0,
        self.WaitRoom = 2,
        self.Explore = 4,
        self.Collect = 8,
        self.Grab = 16,
        self.Return = 32,
        self.Put = 64,
        self.Stop = 128,
        self.CallBack = 256,
        self.Error = 512,
        self.Find=1028,
        self.Robbish_Explore=2056,
        self.Find_Robbish=4112,
        self.Collect_Robbish=8224,
        self.Robbish_grab=16448,
        self.Send=32896,
        self.Quit=65792,
################
# 初始的一些参数 #
################
class Params:
   def __init__(self) -> None:
        self.wait_position=""  # enter,wait
        self.wait_time=0        #maybe time after door open
        self.targets_waypoint="" #mubiao /default kitchen
        self.gate_name=""       # /default A
        self.duration=rospy.Duration(2.0)
        self.msg_cache=""       #useless
        # the following is added by lzh
        self.finish=0           #lzh add,for each time recognize finish ,this is 1
        self.people_exist=0      #lzh add,for each time recognize finish ,if exist people or robbish,this is 1
        self.people_sum=0       #now we have recog * people
#####################
# # 用于前往人的一些参数 #
# #####################
class ahead:
   def __init__(self) -> None:
        self.goal=MoveBaseGoal() #a goal maybe useless?
        self.goal_renew_flag={} #dictionary name:1/0
        self.goals_dit={}   #dictionary name:goal
        self.goals_list=[]  #dic value
        self.index=0        #dic/list index
        self.goals_finish=False     #useless
        self.current_goals_index=0      # whats the difference with index????
        self.srv_rqs=GetWaypointByNameRequest() #unknow
        self.srv_rsp=GetWaypointByNameResponse() #unknow
        self.new_position={}    #useless
        self.msg=String()       #useless
#################
# robbish注册的参数 add by lzh#
#################  
class robbish_det:
    def __init__(self) -> None:
        self.recog_msg="None"       #result for robbish recog
        self.pos=MoveBaseGoal()

#################
# 人体注册的参数 #
#################  
class human_det:
   def __init__(self) -> None:
        self.max_n=0        #max human number
        self.msg=String()   #useless
        self.reg_flag=False   #useless

#################
# 人脸注册的参数 #
#################
class face_det:
   def __init__(self) -> None:  
        self.msg=str()  #useless
        self.recog_msg="None"   #string recognize result
        self.recog_index=0     #useless
        self.is_done=[]     #useless



#################
# body form arg #
#################
class body_det:     #all useless 
    def __init__(self) ->None:
        self.msg=str()          #useless
        self.recog_msg="None"   #useless
        self.recog_index=0      #useless
        self.is_done=[]         #useless
#################
# 抓取物品的参数 #
#################
class grab:
   def __init__(self) -> None:
        self.only_once=0            #unknow
        self.count=0                #now grab index
        self.object_msg=String()    #useless
        self.position="targets"     #useless
#    count_to_name={0:"Alice",1:"Bob",2:"Carol"}
        self.get_msg="0"            #flag for each success grab 

#################
# 放置物品的参数 #
#################
class put:
   def __init__(self) -> None: 
        self.count=0                #useless
        self.done_msg="0"           #flag for each success put
        self.down_msg=String()      #useless
        self.down_msg.data="1"  
        self.Choose=3               #all finish grab

########################
# 预设点 厨房、客厅的信息 #
########################
class PreDefinedWaypoints :
    def __init__(self,gate_name,msg:str):
        self.is_defined=True
        self.persons_waypoint=""
        self.targets_waypoint="targets_"+msg
        self.exit_waypoint=""
        self.grab_out_0="grab_out_"+msg+"_0"
        self.grab_out_1="grab_out_"+msg+"_1"
        if (gate_name== "B"):
            self.persons_waypoint = "persons_living"
            self.exit_waypoint = "A"
        elif(gate_name=="A"):
            self.persons_waypoint = "persons_dining"
            self.exit_waypoint = "B"

######################
# 发布信息的大类 Puber #
######################
class pub:
    def __init__(self):
        self.choose=rospy.Publisher("/general_service_choose_face_or_pose",String,queue_size=10)#pose.py
        self.init_pose=rospy.Publisher("initialpose",Pose,queue_size=10)#amcl_node.cpp
        self.human_det=rospy.Publisher("general_service_human_detection_switch",String,queue_size=100)#test_human_detection.py
        self.words_det=rospy.Publisher("/general_service_xfsaywords",String,queue_size=100)#robot #unknow suber
        self.face_det=rospy.Publisher("general_service_now_goals",String,queue_size=10)#pose2.py
        self.grab_need=rospy.Publisher("general_service_need",String,queue_size=1)#grab.py
        self.putting=rospy.Publisher("genenal_service_put_down",String,queue_size=10)#grab.py
        self.goto_Thing=rospy.Publisher("/general_service_go_to_things",String,queue_size=10)#grab.py
        self.grab_room=rospy.Publisher("grab_room",String,queue_size=10)#grab.py
        self.guidence=rospy.Publisher("all_name",String,queue_size=10)#guidence
        self.over_speak=rospy.Publisher("xf_tts",String,queue_size=10)#unknow suber
        self.refresh_speak=rospy.Publisher("refresh",String,queue_size=10)#unknow suber
        # the following is added by lzh
        self.start_recognize=rospy.Publisher("start_recognize",String,queue_size=10)  # not writing suber yet
        self.facial_det=rospy.Publisher("facial_det",String,queue_size=10)  # not writing suber yet
        self.pose_det=rospy.Publisher("pose_det",String,queue_size=10)  # not writing suber yet
        self.start_recognize_robbish=rospy.Publisher("start_recognize_robbish",String,queue_size=10)  # not writing suber yet
        self.collect_robbish=rospy.Publisher("collect_robbish",String,queue_size=10)  # not writing suber yet
        self.drop_robbish=rospy.Publisher("drop_robbish",String,queue_size=10)  # not writing suber yet
        self.get_robbish_pos=rospy.Publisher("get_robbish_pos",String,queue_size=10)  # not writing suber yet
        self.move_robot_arm=rospy.Publisher("move_robot_arm",String,queue_size=10)  # not writing suber yet
        # self.urgency=rospy.Publisher("")
###################################
#### lzh delete gate name because there is only one wait position
#### wait position in data file is named wait_pos
###################################
    def Init_Pose(self):
        #amcl_node.cpp
        msg=Pose()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id="map"
        #??? unchanged yet
        msg.pose.covariance[0]= 0.25
        msg.pose.covariance[7]= 0.25
        msg.pose.covariance[35]= 0.06853892326654787
        #???
        data=pd.read_xml('/home/lzh/test/src/main_function/maps/waypoints.xml')
        Name=data['Name']
        index= np.where(Name=='wait_pos')
        index=int(index[0])
        Pos_x=data["Pos_x"][index]
        Pos_y=data["Pos_y"][index]
        Ori_z=data["Ori_z"][index]
        Ori_w=data["Ori_w"][index]
        msg.pose.pose.position.x=Pos_x
        msg.pose.pose.position.y=Pos_y
        msg.pose.pose.orientation.z=Ori_z
        msg.pose.pose.orientation.w=Ori_w
        '''
        if (gate_name== "B"):
            rospy.loginfo("选择了B门")
            wait_position = "B_wait"
        elif (gate_name=="A"):
            rospy.loginfo("选择了A门")
            wait_position="A_wait"
        else :
            wait_position=gate_name
        rospy.loginfo("选择了Explore作为开始")'''
        # refresh_data=String()
        # refresh_data.data='onno'
        # self.refresh_speak.publish(refresh_data)
        time.sleep(1)
        self.init_pose.publish(msg)
        # return wait_position
    

    def Words_det(self,msg:str="",index:int=-1):
        #word.py
        msg_new=String()
        msg_new.data=msg
        states=["one","two","three","three"]
        if index !=-1:
            msg_new.data=msg_new.data+states[index]
        refresh_data=String()
        refresh_data.data="one"
        self.refresh_speak.publish(refresh_data)
        time.sleep(1)    
        self.words_det.publish(msg_new)


    def Renew_goal(self,msg:str=""):
        #pose2.py
        msg_new=String()
        msg_new.data=msg
        self.face_det.publish(msg_new)

    def Grab(self,msg:str=""):
        #grab.py
        msg_new=String()
        msg_new.data=msg
        
        
        # 这里是新加的测试，
        rospy.sleep(1)
        
        self.grab_need.publish(msg_new)
        rospy.loginfo('Grab_need_msg_publish succeed')
    
    def Choose(self,msg:str=""):
        #pose.py
        msg_new=String()
        msg_new.data=msg
        self.choose.publish(msg_new)
        rate=rospy.Rate(0.5)
        rate.sleep()

    def Put(self,msg:str=""):
        #grab.py
        msg_new=String()
        msg_new.data=msg
        self.putting.publish(msg_new)

    #########################
    #added by lzh
    def start_recognizing(self):
        self.start_recognize.publish("OK")

    def facial_deting(self):
        self.facial_det.publish("OK")

    def pose_deting(self):
        self.pose_det.publish("OK")
    
    def start_recognize_robbishing(self):
        self.start_recognize_robbish.publish("OK")

    def collect_robbishing(self):
        self.collect_robbish.publish("OK")

    def drop_robbishing(self):
        self.drop_robbish.publish("OK")

    def get_robbish_posing(self):
        self.get_robbish_pos.publish("OK")

    def open_robot_arm(self):
        self.move_robot_arm.publish('1')

    def close_robot_arming(self):
        self.move_robot_arm.publish('0')

    def over_speaking(self,msg:String):
        du = rospy.Duration(4)
        rospy.sleep(du)
        rospy.loginfo("---------------lzh is your dad--------------------")
        self.over_speak.publish(msg)

#########################
# 用于Gotogoal的客户端操作#
#########################
class client:
    def __init__(self) -> None:
        self.go_to_flag=False       #unknow
        self.togetname=rospy.ServiceProxy("/waterplus/get_waypoint_name",GetWaypointByName)#robot   #unknow
        self.ac=actionlib.SimpleActionClient("move_base",MoveBaseAction)    #receive bot action status,server is ros itself

def Gotogoal(goal:MoveBaseGoal):
   rospy.loginfo("等待服务器")
   Client.ac.wait_for_server()
   rospy.loginfo('Gotogoal消息发出')
   Client.ac.send_goal(goal)
   Client.ac.wait_for_result()
   if Client.ac.get_state() == actionlib.SimpleGoalState.DONE:
       Client.go_to_flag=True
   return True


######################
# 接收信息的大类 Suber #
######################
class sub:
    def __init__(self) -> None:
     self.object_names=[]
     self.word_flag=False
     self.i=0
     self.goal=MoveBaseGoal()
     self.now_goal=MoveBaseGoal()
     self.init_goals=rospy.Subscriber("/general_service_loc_target",MoveBaseGoal,self.InitPersonFrontCallBack,queue_size=10)#pose1.py
     self.object_name=rospy.Subscriber("general_service_object_name_return",String,self.ObjectNameCallBack,queue_size=10)#words.py
     self.wait_time=rospy.Subscriber("/wpb_home/entrance_detect",String,self.EntranceCallBack,queue_size=10)#entrance_detect.cpp
     #self.renew_goals=rospy.Subscriber("/person/waypoint",Goals_name,self.Renew_goals,queue_size=100)#pose2.py
     self.explore_msg=rospy.Subscriber("general_service_recognition",String,self.RecognitionCallBack,queue_size=10 )#pose1.py
     self.down_msg=rospy.Subscriber("genenal_service_put_down_result",String,self.PutDownResultCallBack,queue_size=10)#grab.py
     self.get_msg=rospy.Subscriber("genenal_service_get_it",String,self.GetItCallBack,queue_size=10)#grab.py
     self.now_position=rospy.Subscriber("/general_service_now_position",MoveBaseGoal,self.Now_position_callback,queue_size=10)
     self.start_recognize_reply=rospy.Subscriber("start_recognize_reply",String,self.start_recognize_reply_callback,queue_size=10)
     self.facial_det_reply=rospy.Subscriber("facial_det_reply",String,self.facial_det_reply_callback,queue_size=10)
     self.pose_det_reply=rospy.Subscriber("pose_det_reply",String,self.pose_det_reply_callback,queue_size=10)
     self.start_recognize_robbish_reply=rospy.Subscriber("start_recognize_robbish_reply",String,self.start_recognize_robbish_reply_callback,queue_size=10)
     self.collect_robbish_reply=rospy.Subscriber("collect_robbish_reply",String,self.collect_robbish_reply_callback,queue_size=10)
     self.get_robbish_pos_reply=rospy.Subscriber("get_robbish_pos_reply",MoveBaseGoal,self.get_robbish_pos_reply_callback,queue_size=10)
     self.move_robot_arm_reply=rospy.Subscriber("move_robot_arm_reply",String,self.move_robot_arm_reply_callback,queue_size=10)

    def Now_position_callback(self,msg:MoveBaseGoal):
        self.now_goal=msg
   
    def ObjectNameCallBack(self,msg:String):
        global PDW,test_i,recode_temp,delete_flag,Ahead
        # rospy.loginfo("收到房间名称%s::ObjectNameCallBack: ",msg.data)
        if msg.data!="pass":
            if msg.data in ["bedroom","kitchen","living","dining"]:
                rospy.loginfo("收到物品所在房间: %s",msg.data)
            elif msg.data!="false" and test_i==1:
                rospy.loginfo("收到物品的名称为%s",msg.data)
                Puber.guidence.publish(msg)
                
                self.word_flag=True
                if recode_temp==0:
                    self.object_names.append("pass")
                    Ahead.goals_dit["pass"+str(Ahead.index)]=Ahead.goals_list[Ahead.index]
                    Ahead.index+=1
                else :
                    self.object_names.append(msg.data)
                    recode_temp=0
                rospy.loginfo("Ahead.current_index=%d,len(object_name)=%d",Ahead.current_goals_index,len(self.object_names))
   
    def EntranceCallBack(self,msg:String):
        global params
        # self.i+=1
        # rospy.loginfo("wait_time %d 消息收到",params.wait_time)
        if params.wait_time>100:
            return
        elif params.wait_time<0:
            rospy.logwarn("wait_time数值明显错误")
        if msg.data=="door open":
            params.wait_time+=1
        else:
            params.wait_time=0

    def InitPersonFrontCallBack(self,goal:MoveBaseGoal):
        global Ahead
        self.i+=1
        self.goal=goal
        Ahead.goals_list.append(goal)
        rospy.loginfo("收到了第%d个goal<---pose1.py",self.i)


    def PutDownResultCallBack(self,msg:String):
        global Put
        if Put.done_msg=="0":
            Put.done_msg=msg.data
            rospy.loginfo("收到的Put.done_msg==1,fsm转为stop或者grab")
        else:
            rospy.loginfo("接受Put.done.msg话题错误,发过来的时候Put.done_msg不是0")

    def RecognitionCallBack(self,msg:String):
         global Ahead,recode_temp
         if msg.data!="None":
            rospy.loginfo("第%d个人注册成功",Ahead.current_goals_index+1) 
            Ahead.goals_dit[msg.data]=Ahead.goals_list[Ahead.index]
            Ahead.index+=1
            Face_det.recog_msg=msg.data
            recode_temp=1
         else:
            Face_det.recog_msg=msg.data
         rospy.loginfo("main:收到的第%d个人的rocognition=%s",Ahead.current_goals_index+1,Face_det.recog_msg)

    def GetItCallBack(self,msg:String):
        global Grab
        if Grab.get_msg=="0":
            rospy.loginfo("收到 get_it 为 1, 状态转为 put")
            Grab.get_msg=msg.data
        else:
            rospy.loginfo("接受get_it话题错误,发过来的时候get_msg不是0")
        pass

    def start_recognize_reply_callback(self,msg:String):
        global params
        params.finish=int(msg[0])
        params.people_exist=int(msg[-1])

    def facial_det_reply_callback(self,msg:String):
        global Face_det
        Face_det.recog_msg=msg.data
        rospy.loginfo(f"facial_det_reply_callback receive {Face_det.recog_msg}")

    def pose_det_reply_callback(self,msg:String):
        global Body_det
        Body_det.recog_msg=msg

    def start_recognize_robbish_reply_callback(self,msg:String):
        global params
        params.finish=int(msg[0])
        params.people_exist=int(msg[-1])

    def collect_robbish_reply_callback(self,msg:String):
        global Robbish_det
        Robbish_det.recog_msg=msg
    
    def get_robbish_pos_reply_callback(self,msg:MoveBaseGoal):
        global Robbish_det
        Robbish_det.pos=msg
        
    def move_robot_arm_reply_callback(self,msg:String):
        global params
        if msg=='1':
            params.finish=1
        else:
            rospy.ERROR("cannot move robot arm")

'''
    def Renew_goals(self,msg:Goals_name):
        global Ahead,current_name
        if msg.goal.target_pose.header.frame_id=="None" and Put.Choose==4:
            rospy.loginfo("exchange msg.name %s with current_name %s",msg.name,current_name)
            t=Ahead.goals_dit[msg.name]
            Ahead.goals_dit[msg.name]=Ahead.goals_dit[current_name]
            Ahead.goals_dit[current_name]=t
            Ahead.goal_renew_flag[msg.name]=1
        elif msg.goal!=None:
            rospy.loginfo("Renew the goal:%s",msg.name)
            Ahead.goal_renew_flag[msg.name]=1
            Ahead.goals_dit[msg.name]=msg.goal
'''
    


def Dijkstra(goals_list:List[MoveBaseGoal],now_goal:MoveBaseGoal):
    temp_list=goals_list
    now_count=0
    l=[now_goal]
    for i  in range(len(goals_list)):
        min_distance=99
        now_idx=-1
        for idx,g in  enumerate (goals_list):
            temp_goal=l[now_count]
            temp_goal_x=temp_goal.target_pose.pose.position.x
            temp_goal_y=temp_goal.target_pose.pose.position.y
            g_x=g.target_pose.pose.position.x
            g_y=g.target_pose.pose.position.y
            distance=math.sqrt((temp_goal_x-g_x)**2+(g_y-temp_goal_y)**2)
            if distance<min_distance:
                min_distance=distance
                now_idx=idx
        l.append(goals_list[now_idx])
        now_count+=1
        goals_list.remove(goals_list[now_idx])
    l.remove(now_goal)
    if l!=temp_list:rospy.loginfo("客人位置用DJ交换了")
    return l
            

#####################################
# 操控机器移动的函数，简单理解为前往某个点 #
#####################################
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


########
# Main #
########
def get_odom_message(odom:Odometry):
    global machine_odom_now_y,machine_odom_now_x,mathince_theta
    machine_odom_now_x= odom.pose.pose.position.x
    machine_odom_now_y=odom.pose.pose.position.y 
    odom_ox=odom.pose.pose.orientation.x
    odom_oy=odom.pose.pose.orientation.y
    odom_oz=odom.pose.pose.orientation.z
    odom_ow=odom.pose.pose.orientation.w 
    _,_,mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow])
    
'''
if __name__=="__main__":
    # global delete_flag
    test=False
    test_i=0
############################################  
# 初始化节点,设置频率名称，fsm初始状态----------#
# param中得到gate_name,根据gatename选姿态信息 #
# 并发送姿态信息初始化位置 --------------------#
############################################
    rospy.init_node("main_service")
    Status=Statu()
    params=Params()
    Ahead=ahead()
    Face_det=face_det()
    Human_det=human_det()
    Grab=grab()
    Put=put()
    Puber=pub()
    Suber=sub()
    Client=client()
    Body_det=body_det()

    rate=rospy.Rate(40.0)
    rospy.Rate(1).sleep()    
    rospy.loginfo("节点main_service启动")
    params.gate_name=rospy.get_param("gate_name")
    params.targets_waypoint=rospy.get_param("targets_waypoint")
    params.wait_position=Puber.Init_Pose(params.gate_name)
    PDW=PreDefinedWaypoints(params.gate_name,params.targets_waypoint)
    rospy.loginfo("初始位置设置完毕")
    fsm=Status.Enter
    # test_i=1 # Put test
    # fsm=Status.Grab#Grab test
    # Suber.object_names=["bottle"]
    # fsm=Status.CallBack# Put test
    # Grab.get_msg="1"# Put test
    # Face_det.recog_msg="end"# Put Grab test
    # fsm=Status.Explore#Explore 测试用
    # rospy.sleep(9)#Exlore Put test
    while not rospy.is_shutdown():
       ####################################################
       # Enter持续检测门20次，进入等待地点，进入下一状态Explore #
       ####################################################
        if fsm==Status.Enter:
            if params.wait_time>20:
                rospy.loginfo("门已开")
                rospy.loginfo("person_waypoint={1} \n targets_waypoint={0}".format(PDW.targets_waypoint,PDW.persons_waypoint))
                if test==False:
                    success=Gotopoint(params.wait_position)
                else: 
                    success=True
                if success==True:
                    fsm=Status.WaitRoom
        ############################
        #shibie#
        ############################
        elif fsm==Status.Form:
            rospy.loginfo("Start recoginze body form")
            rospy.loginfo("aaaaaaaaa")
                
        ############################
        # Waitroom等待2s进入人所在房间 #
        ############################
                
        elif fsm==Status.WaitRoom:
            rospy.loginfo("当前状态为Waitroom,等待2s")
            rospy.sleep(params.duration)
            if test==True:
                success==True
            else:
                success=Gotopoint(PDW.persons_waypoint)
            if success==True:
                fsm=Status.Explore
                rospy.loginfo("start Explore")

        ######################################         
        # Explore 准备识别人，到面前进入 Collect #
        ######################################
        elif fsm==Status.Explore:
            if test_i!=1:
                # rospy.sleep(1)
                rospy.loginfo("Choose 1 Already send")
                Puber.Choose("1")
                rospy.sleep(5)
                Puber.Choose("0")
                test_i=1
                Ahead.goals_list=Dijkstra(Ahead.goals_list,Suber.now_goal)
                Human_det.max_n=len(Ahead.goals_list)
                rospy.loginfo("要人脸检测的数量为man_n=%d",Human_det.max_n)
            goal_explore=Ahead.goals_list[Ahead.current_goals_index]
            Gotogoal(goal_explore)
            fsm=Status.Collect
        #############################################
        # Collect 采集信息一旦某人注册成功，进入Callbeck #
        #############################################
        elif fsm==Status.Collect:
                begin_time=time.time()
                rospy.loginfo("start Collect")
                rospy.loginfo("目前状态为Collect采集信息,目前执行到第%d位客人",Ahead.current_goals_index+1)
                if Ahead.current_goals_index==0 or Ahead.current_goals_index==1 or Ahead.current_goals_index==2 or Ahead.current_goals_index==3:
                    rospy.loginfo("正在前往第%d个目标",Ahead.current_goals_index+1)
                    Gotogoal(Ahead.goals_list[Ahead.current_goals_index])
                    Puber.Choose("2")
                    Puber.Words_det(index=Ahead.current_goals_index)
                    #shang2 hang unknow suber
                    rospy.loginfo("语音 和Choose 2消息已经发出")
                    rospy.sleep(2)
                    if Face_det.recog_msg=="None":
                        rospy.loginfo("%d个人注册没成功,进入回调状态",Ahead.current_goals_index)
                        now_time=rospy.Time.now()
                        fsm=Status.CallBack
                else:
                    rospy.loginfo("当前current_person_index是%d",Ahead.current_goals_index)
                    continue
                fsm=Status.CallBack
                continue
        ##################################
        # Grab 发布信息，前往抓取count号物品 #
        ##################################
        elif fsm==Status.Grab:
            rospy.loginfo("当前状态为Grab")
            data=String()
            data.data=params.targets_waypoint
            if Grab.count==len(Suber.object_names):
                fsm=Status.Stop
                continue
            while Suber.object_names[Grab.count]=="pass":
                Grab.count+=1
                if Grab.count==len(Suber.object_names):
                    fsm=Status.Stop
                    break
            if fsm==Status.Stop:
                pass
            else:
                Puber.grab_room.publish(data)
                Gotopoint(PDW.targets_waypoint)#门口
                rospy.Rate(2).sleep()
                Puber.Grab(msg=Suber.object_names[Grab.count])
                rospy.loginfo("the people want is %s",Suber.object_names[Grab.count])
                #TODO
                #这里要更改bottle
                # Puber.Grab("bottle") # Grab test
                # 发布物品信息
                rospy.loginfo("发送完成第%d个客人的物品:%s",Grab.count,Suber.object_names[Grab.count])
                fsm=Status.CallBack
        ########
        # Put  #
        ########
        elif fsm==Status.Put:
            rospy.loginfo("当前状态为Put")
            params.msg_cache="please take your"+Suber.object_names[Grab.count]
            rospy.loginfo("Put 的 语言消息发出了")
            Puber.Words_det(params.msg_cache)
            rospy.loginfo("%s Put 消息已经发财",current_name)
            Puber.Put("1")
            fsm=Status.CallBack
        ############
        # Stop 离场 #
        ############
        elif fsm==Status.Stop:
            rospy.loginfo("当前 main_service 状态为 stop")
            Gotopoint(position="B")
            rospy.loginfo("机器人出场完成")
            pass
        ############
        # CallBack #
        ############
        elif fsm==Status.CallBack:
            #################
          # 人体注册成功，停止人脸识别，否则继续#
          # this if is for Collect state ##
          #################################
          if Face_det.recog_msg=="None" and test_i==1:
             now_time=time.time()
             if now_time-begin_time>30:
                Suber.object_names.append("pass")
                Ahead.goals_dit[str(Ahead.index)+"pass"]=Ahead.goals_list[Ahead.index]
                Suber.word_flag=False
                Client.go_to_flag=False
                Ahead.current_goals_index+=1
                over_data=String()
                over_data.data="over"
                Puber.over_speak.publish(over_data)
                if Ahead.current_goals_index>=Human_det.max_n:
                    rospy.loginfo("采集完毕进入Grab状态")
                    Face_det.recog_msg="End"
                    Suber.word_flag==False
                    fsm=Status.Grab
                    # fsm=Status.CallBack #Put test
                    # Grab.get_msg="1"# Put test
                elif Ahead.current_goals_index<0:
                    rospy.ERROR("current_person_index 的值出现异常，建议立即进行检查")
                else:
                    rospy.loginfo("进行下一个人的信息采集")
                    Face_det.recog_msg="None"
                    fsm=Status.Explore
             else :
              Puber.Choose("2")
              rospy.loginfo("再次尝试注册第%d人脸",Ahead.current_goals_index+1)
              rospy.sleep(2)
          ##########################################################################
          # 人脸识别成功后，如果走到x个人面前&&语音信息对的上&&人数合理，就抓去，否则继续前往人 #
          # this will turn state to Grab or Explore ################################
          # this is the for Collect state  ##########################################
          ##########################################################################
          if len(Suber.object_names)+delete_flag==Ahead.current_goals_index+1 and Suber.word_flag==True:
        #   if Suber.word_flag==True:
             Suber.word_flag=False
             Client.go_to_flag=False
             rospy.loginfo("收到语音识别结果，第 %d 个客人需要 %s"
                           ,Ahead.current_goals_index+1,Suber.object_names[Ahead.current_goals_index]) 
             Ahead.current_goals_index+=1
             if Ahead.current_goals_index>=Human_det.max_n:
                rospy.loginfo("采集完毕进入Grab状态")
                Face_det.recog_msg="End"
                Suber.word_flag==False
                fsm=Status.Grab
                # fsm=Status.CallBack #Put test
                # Grab.get_msg="1"# Put test
             elif Ahead.current_goals_index<0:
                rospy.ERROR("current_person_index 的值出现异常，建议立即进行检查")
             else:
                rospy.loginfo("进行下一个人的信息采集")
                Face_det.recog_msg="None"
                fsm=Status.Explore
           ############################################     
           # 放下东西后，如果放完了，准备出场，否则继续去抓取##
           # this will turn state to Stop or Grab######
           # this is for Put state ####################
           ############################################
          if Put.done_msg=="1":
                Put.done_msg="0"
                if Grab.count+1==Human_det.max_n:
                    rospy.loginfo("全部抓取完毕，准备出场")
                    fsm=Status.Stop
                else:
                    rospy.loginfo("送完了第%d个物品",Grab.count+1)
                    Grab.count+=1
                    fsm=Status.Grab
                    # fsm=Status.CallBack #Put test
                    # Grab.get_msg="1"# Put test
                continue
        #    ########################################+
        #    # 如果拿到物品，Gotopoint去客人位置放置物品 #
        #    # this is for Grab state turn to Put####
        #    ########################################
          if Grab.get_msg=="1":
                Grab.get_msg="0"
                # PDW.grab_out_0="grab_out_kitchen_0"#test
                # PDW.grab_out_1="grab_out_kitchen_1"#test
                Gotopoint(PDW.grab_out_0)
                Gotopoint(PDW.grab_out_1)
                current_name=list(Ahead.goals_dit.keys())[Grab.count]
                rospy.loginfo("抓取完成，正在去客人所在位置")
                # rospy.sleep(3)
                Put.Choose=4
                if Grab.only_once==0:
                    Grab.only_once+=1
                    for name in Ahead.goals_dit.keys() :
                        Ahead.goal_renew_flag[name]=0        #why reset renew_flag?????
                rospy.loginfo("当前目标客户是%s",current_name)
                while Ahead.goal_renew_flag[current_name]!=1:
                    rospy.loginfo("Target haven't refreshed ,Now go to explore the customer")
                    goal=Ahead.goals_dit[current_name]
                    print(Ahead.goals_dit)
                    Gotogoal(goal) 
                    Puber.Choose("4")
                    Puber.Renew_goal("1")
                    rospy.sleep(2)
                    Puber.Renew_goal("0")
                rospy.loginfo("Now go to current customer after relocated")
                Gotogoal(Ahead.goals_dit[current_name])
                fsm=Status.Put
                continue
    rospy.spin()
    rate.sleep()
    rospy.loginfo("节点已经退出")
      '''



#############
## LZH main #
#############
if __name__ =="__main__":
    #??
    test=False
    test_i=0
    #??
    rospy.init_node("main_function")
    Status=Statu()
    params=Params()
    Ahead=ahead()
    Face_det=face_det()
    Human_det=human_det()
    Grab=grab()
    Put=put()
    Puber=pub()
    Puber.over_speaking("started")
    Suber=sub()
    Client=client()
    Body_det=body_det()
    Robbish_det=robbish_det()
    #waypoints=pd.read_xml('/home/lzh/test/src/main_function/maps/waypoints.xml') ###in __main__
    #waypoints_name=waypoints['Name']
    #index= np.where(waypoins_name=='wait_pos')
    #error here
    waypoints_index=0        ###error here, unknow the first waypoint index
    Collect_index=0
    Collect_robbish_index=0
    Robbish_waypoints_index=0
    rate=rospy.Rate(40.0)
    rospy.Rate(1).sleep()    
    rospy.loginfo("节点main_service启动")
    '''
    params.gate_name=rospy.get_param("gate_name")
    params.targets_waypoint=rospy.get_param("targets_waypoint")
    '''
    #params.wait_position=Puber.Init_Pose()
    #PDW=PreDefinedWaypoints(params.gate_name,params.targets_waypoint)
    rospy.loginfo("初始位置设置完毕")
    fsm=Status.Collect

    # test_i=1 # Put test
    # fsm=Status.Grab#Grab test
    # Suber.object_names=["bottle"]
    # fsm=Status.CallBack# Put test
    # Grab.get_msg="1"# Put test
    # Face_det.recog_msg="end"# Put Grab test
    # fsm=Status.Explore#Explore 测试用
    # rospy.sleep(9)#Exlore Put test
    while not rospy.is_shutdown():
        if fsm==Status.Enter:
            if params.wait_time>20: #unknow
                rospy.loginfo("ENTER!")
                # rospy.loginfo("person_waypoint={1} \n targets_waypoint={0}".format(PDW.targets_waypoint,PDW.persons_waypoint))
                if test==False:
                    success=Gotopoint(params.wait_position)
                else: 
                    success=True
                if success==True:
                    fsm=Status.Explore

        elif fsm==Status.Explore:
            rospy.loginfo("EXPORE!")
            msg=Pose()
            msg.header.stamp=rospy.Time.now()
            msg.header.frame_id="map"
            #??? unchanged yet
            # msg.pose.covariance[0]= 0.25
            # msg.pose.covariance[7]= 0.25
            # msg.pose.covariance[35]= 0.06853892326654787
            #???
            #data=pd.read_xml('/home/linxi/ServiceRobot-General/src/general_service_2022/maps/waypoints.xml')
            # Name=data['Name']
            index=waypoints_index
            success=Gotopoint(waypoints_name(index))
            rospy.loginfo(f"going {waypoints_name(index)}")
            if success==True:
                fsm=Status.Find
                waypoints_index+=1
            else:
                rospy.ERROR(f"Cannot goto {waypoints_name(index)}")

        elif fsm==Status.Find:
            #rospy.loginfo("目前状态为Collect采集信息,目前执行到第%d位客人",Collect_index+1)
            # rotate
            have_people=0 # useless
            goal_rotate=MoveBaseGoal()
            for i in range(1,20):
                goal_rotate.target_pose.header.frame_id = "odom"
                goal_rotate.target_pose.header.stamp =rospy.Time.now()
                goal_rotate.target_pose.pose.position.x = 0
                goal_rotate.target_pose.pose.position.y = 0
                goal_rotate.target_pose.pose.position.z = 0.0
                # bu hui suan
                goal_rotate.target_pose.pose.orientation.x =  0.995
                goal_rotate.target_pose.pose.orientation.y =  0.000
                goal_rotate.target_pose.pose.orientation.z = 0.000
                goal_rotate.target_pose.pose.orientation.w =  0.105
                Client.ac.wait_for_server()
                Client.ac.send_goal(goal_rotate)
                Client.ac.wait_for_result()
                if Client.ac.get_state() == actionlib.SimpleGoalState.DONE:
                    Puber.start_recognizing()
                    rospy.loginfo("start_recognizing...")
                    while params.finish==0:  #waiting for recognize
                        pass
                    params.finish=0
                    if params.people_exist==1:#exist people , exter Collect
                        fsm=Status.Collect
                        params.people_exist=0
                        rospy.loginfo(f"Find people {Collect_index+1}")
                        Collect_index+=1
                        have_people=1
                        break
                    elif params.people_exist==0:
                        continue
                else:
                    rospy.loginfo("error occer! cannot rotate!")
            if have_people==0:
                rospy.loginfo("No people here!")
                fsm==Status.Explore
        elif fsm==Status.Collect:
            du=rospy.Duration(3)
            du1=rospy.Duration(1)
            rospy.loginfo(" Enter Collect!")
            Puber.over_speaking("Enter Collect!")
            # face
            rospy.sleep(du1)
            Puber.facial_deting()
            rospy.sleep(du1)
            face_data=String()
            face_data.data=f"this people is {Face_det.recog_msg}"
            Puber.over_speaking(face_data)
            rospy.sleep(du)
            # pose ,error here, unknow the puber for starting pose recognize
            Puber.pose_deting()
            rospy.sleep(du1)
            pose_data=String()
            pose_data.data=f"he is {Body_det.recog_msg}"
            Puber.over_speaking(pose_data)
            rospy.sleep(du)
            next_data=String()
            next_data.data="you can do your next pose"
            Puber.over_speaking(next_data)
            rospy.sleep(du)
            Puber.pose_deting()
            rospy.sleep(du1)
            pose_data.data=f"he is {Body_det.recog_msg}"
            Puber.over_speaking(pose_data)
            rospy.sleep(du)
            params.people_sum+=1
            if params.people_sum==3 or waypoints_index==4:
                rospy.loginfo("Finished recognizing people, enter Robbish_Explore")
                pose_data.data="enter Robbish Explore"
                Puber.over_speaking(pose_data)
                rospy.sleep(du)
                fsm=Status.Robbish_Explore
            else:
                pose_data.data="enter Explore"
                Puber.over_speaking(pose_data)
                rospy.sleep(du)
                fsm=Status.Explore
        #robbish, similiar to face and pose recog
        elif fsm==Status.Robbish_Explore:
            rospy.loginfo("Robbish_Explore!")
            # msg=Pose()
            # msg.header.stamp=rospy.Time.now()
            # msg.header.frame_id="map"
            #??? unchanged yet
            # msg.pose.covariance[0]= 0.25
            # msg.pose.covariance[7]= 0.25
            # msg.pose.covariance[35]= 0.06853892326654787
            #???
            #data=pd.read_xml('/home/linxi/ServiceRobot-General/src/general_service_2022/maps/waypoints.xml')
            # Name=data['Name']
            index=Robbish_waypoints_index
            success=Gotopoint(waypoints_name(index))
            rospy.loginfo(f"going {waypoints_name(index)}")
            if success==True:
                fsm=Status.Find_Robbish
                Robbish_waypoints_index+=1
            else:
                rospy.ERROR(f"Cannot goto {waypoints_name(Robbish_waypoints_index)}")
        elif fsm==Status.Find_Robbish:
            have_robbish=0 # useless
            goal_rotate=MoveBaseGoal()
            for i in range(1,20):
                goal_rotate.target_pose.header.frame_id = "odom"
                goal_rotate.target_pose.header.stamp =rospy.Time.now()
                goal_rotate.target_pose.pose.position.x = 0
                goal_rotate.target_pose.pose.position.y = 0
                goal_rotate.target_pose.pose.position.z = 0.0
                # bu hui suan
                goal_rotate.target_pose.pose.orientation.x =  0.995
                goal_rotate.target_pose.pose.orientation.y =  0.000
                goal_rotate.target_pose.pose.orientation.z = 0.000
                goal_rotate.target_pose.pose.orientation.w =  0.105
                Client.ac.wait_for_server()
                Client.ac.send_goal(goal_rotate)
                Client.ac.wait_for_result()
                if Client.ac.get_state() == actionlib.SimpleGoalState.DONE:
                    Puber.start_recognize_robbishing()
                    rospy.loginfo("start_recognizing_robbish...")
                    while params.finish==0:  #waiting for recognize
                        pass
                    params.finish=0
                    if params.people_exist==1:#exist robbish , exter Collect
                        fsm=Status.Collect_Robbish
                        params.people_exist=0
                        rospy.loginfo(f"Find robbish {Collect_index+1}")
                        Collect_robbish_index+=1
                        have_robbish=1
                        break
                    elif params.people_exist==0:
                        continue
                else:
                    rospy.loginfo("error occer! cannot rotate!")
            if have_robbish==0:
                rospy.ERROR("Have not find robbish in this room, rocognize again")
        
        elif fsm==Status.Collect_Robbish:
            Puber.collect_robbishing()
            robbish_data=String()
            robbish_data.data=f"this robbish is {Robbish_det.recog_msg}"
            Puber.over_speak(robbish_data)
            Puber.get_robbish_posing()
            Client.ac.wait_for_server()
            Client.ac.send_goal(Robbish_det.pos)
            Client.ac.wait_for_result()
            if Client.ac.get_state() == actionlib.SimpleGoalState.DONE:
                rospy.loginfo("Please help me grabbing the robbish")
                pose_data=String()
                pose_data.data=f"give me the robbish please"
                Puber.over_speak(pose_data)
                Puber.open_robot_arm()
                rospy.sleep(20)
                Puber.close_robot_arming()
            else:
                rospy.ERROR("cannot rotate!")
            rospy.sleep(5)
            rospy.loginfo("start sending")
            fsm=Status.Send




        elif fsm==Status.Send:
            index=Robbish_waypoints_index
            success=Gotopoint(waypoints_name(index+4))
            rospy.loginfo(f"going {waypoints_name(index+4)}")
            Puber.open_robot_arm()
            while params.finish ==0: pass
            params.finish=0
            if Robbish_waypoints_index==3:
                rospy.loginfo("Finished grab all robbish,now quit")
                fsm=Status.Quit
            else :
                fsm=Status.Robbish_Explore


        elif fsm==Status.Quit:
            rospy.loginfo("ALL FINISH!")
            break
    rospy.loginfo("node has quit")
        







    
            
