# from ctypes.wintypes import tagMSG
# from ctypes.wintypes import tagMSG
from importlib.resources import path
import queue
# from tokenize import String
from std_msgs.msg import String
from turtle import width
from xml.dom.expatbuilder import theDOMImplementation
import tf2_ros
from inspect import stack
import re
from select import select
import matplotlib.pyplot as plt
import torch
import cv2
from torchvision import transforms
import numpy as np
import sys
from nav_msgs.msg import Odometry
sys.path.insert(0,'/home/linxi/ServiceRobot-General/src/pose_estimate')
sys.path.insert(1,'/home/linxi/ServiceRobot-General/src/pose_estimate/scripts')
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts,plot_one_box
from face_detect import FaceRecognition
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist
import time
import os
import threading
from tf2_geometry_msgs import PointStamped,PoseStamped
from queue import Queue
# 加载模型
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import message_filters
import mediapipe as mp
import rospy
from math import atan2,pi,sin,cos
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from judge_arrive.msg import judge_arrive
from std_msgs.msg import Int32
import actionlib
import math
from general_service_2022.msg import Goals_name
import tf.transformations as transformations

goal_name=Goals_name()
w_img = 960  # 图像宽度
h_img = 540  # 图像高度
p_x = 479 
p_y = 269
f_x = 540.68603515625
f_y = 540.68603515625
image = Image()
depth = Image()
bridge = CvBridge()
eps = 0.01
mediapipe_detect_count = 0
people_havebeen_recorder=0
start_work=False
goals=[]
end_goal=MoveBaseGoal()
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
weigths = torch.load('/home/linxi/ServiceRobot-General/src/pose_estimate/scripts/yolov7-w6-pose.pt')
model = weigths['model']
model = model.half().to(device)
_ = model.eval()
flag=0
mutex1=threading.Lock()
mutex2=threading.Lock()
mutex_quene=threading.Lock()
now_flag=0
person_index_count=0
person_index=0
mediapipe_detect_count=0
all_work_count=0
empty_list=[]
empty_list_index=0
machine_odom_now_y=0
machine_odom_now_x=0
mathince_theta=0
all_image_count=0
mediape_mutex=threading.Lock()
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=1,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)
# mp_pose=None
choose_face_or_pose_flag=0
face_recognitio = FaceRecognition()
loc_quene=Queue(10)
people_can_get_count=0
face_db_store_name=[]
def task3(mutex,name):
    global flag,person_index_count,face_recognitio,now_flag,people_havebeen_recorder
    # print(now_flag)
    mutex.acquire()

    begin=time.time()
    # names_list =os.listdir("/home/linxi/ServiceRobot-General/src/pose_estimate/data_face")
    # names_list.reverse()
    # rospy.loginfo("name    5555")
    img=cv2.imdecode(np.fromfile(name,dtype=np.uint8),-1)
    # rospy.loginfo("%s",name)
    store_name=name.split('/')[-1]
    store_name=store_name.split('.')[0]
    # result = face_recognitio.register(img, user_name=f'person{person_index_count}')
    result=face_recognitio.recognition(img)
    try:
        result=result[0]
    except BaseException as e:
        result="unknown"
    rospy.loginfo("this is result %s",result)

    people_havebeen_recorder=1
    mutex.release()
    return result
def add_name_to_list():
    global face_db_store_name
    face_db_store_name.clear()
    face_db_store_name=os.listdir("/home/linxi/ServiceRobot-General/src/pose_estimate/face_db")
    for index,data in enumerate( face_db_store_name):
        face_db_store_name[index]=face_db_store_name[index].split(".")[0]
    print(222222222222222)
    print(face_db_store_name)
def stop():
    vel_cmd=Twist()
    vel_cmd.linear.x=0
    vel_cmd.linear.y=0
    vel_cmd.linear.z=0
    vel_cmd.angular.x=0
    vel_cmd.angular.z=0
    vel_cmd.angular.z=0

def image_callback(image_rgb,image_depth):
    global flag,person_index,people_can_get_count,loc_quene,all_work_count,choose_face_or_pose_flag,loc_target_pub,face_feature_pub,empty_list,empty_list_index
    global goal_name,all_image_count
    if choose_face_or_pose_flag==0:
        return
    if choose_face_or_pose_flag==3 and start_work==False:
        return
    if choose_face_or_pose_flag==4 and start_work==False:
        return
    if start_work==True and choose_face_or_pose_flag!=3 and choose_face_or_pose_flag!=4:
        return
    # rospy.loginfo("now it is in image_cb")
    
    if all_work_count==0:
        if choose_face_or_pose_flag==1:
            time.sleep(1)
        else :
            time.sleep(0.2)
    # rospy.loginfo("have entered image_callback")
    if all_work_count<=0:
        # rospy.loginfo("this is caused by all_work_count")
        image = bridge.imgmsg_to_cv2(
                image_rgb, desired_encoding='passthrough')
        depth = bridge.imgmsg_to_cv2(
                image_depth, desired_encoding='passthrough')
        last_time=0
        url=0
        now_time=time.time()
        # cap = cv2.VideoCapture(url)
        t=time.time()
        count=0
        begin_time=time.time()
        # while (cap.isOpened()):
        mutex1.acquire()
        # ret, image = cap.read()
        #image = cv2.imread('xiaolu.jpg')
        # mediapipe_detect(image)
        image = letterbox(image, 960, stride=64, auto=True)[0]
        image_ = image.copy()
        image = transforms.ToTensor()(image)
        image = torch.tensor(np.array([image.numpy()]))
        image = image.to(device)
        image = image.half()

    # 姿势识别
        with torch.no_grad():
            output, _ = model(image)
            output = non_max_suppression_kpt(output, 0.25, 0.65, nc=model.yaml['nc'], nkpt=model.yaml['nkpt'], kpt_label=True)
            output = output_to_keypoint(output)
        nimg = image[0].permute(1, 2, 0) * 255
        nimg = nimg.cpu().numpy().astype(np.uint8)
        nimg = cv2.cvtColor(nimg, cv2.COLOR_RGB2BGR)
        # print(output.shape)
        cv2.imwrite("/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/medi"+str(all_image_count)+".jpg",nimg)
        all_image_count+=1
        area=[]
        for i in range(output.shape[0]) :
            print(output[i,2:6])
            area.append((output[i,5]*output[i,4],i))
        rospy.loginfo("this is output size %d",output.shape[0])
        print(area)
        for i in range(output.shape[0]):
            for j in range(i+1,output.shape[0],1):
                if (area[i]<area[j]):
                    area[i],area[j]=area[j],area[i]
                    # output[i],output[j]=output[j],output[i]
        length_area=min(len(area),3)
        idx_list=[]
        for i in range(length_area):
            idx_list.append(area[i][1])
        for i in range(len(idx_list)):
            index_i=idx_list[i]
            for j in range(i+1,len(idx_list),1):
                index_j=idx_list[j]
                if abs(output[index_i][2]-480)>abs(output[index_j][2]-480):
                    idx_list[i],idx_list[j]=idx_list[j],idx_list[i]
        
        for i in idx_list:
            print(output[i][2])
        print(area)
        # print(output[0,:])
        # area.sort(area)
        # print(output)
        # print("-------------------")
        rospy.loginfo("222222222222")
        idx_count=0
        for idx in idx_list:
            # idx=area[idx][1]
            rospy.loginfo("now all the image size is %d",output.shape[0])
            print(idx,output[idx,2:6])
            # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
            print(output[idx,2:6])
            if idx_count>0 and choose_face_or_pose_flag==2:
                continue
            if idx_count>0 and choose_face_or_pose_flag==4:
                continue
            idx_count+=1
            x_center=int(output[idx][2])
            y_center=int(output[idx][3])
            half_w=int(output[idx][4]/2)
            half_h=int(output[idx][5]/2)
            # if x_center+2*half_w>=960:
            #     half_w=int((960-x_center)/2)
            # elif x_center-2*half_w<0:
            #     half_h=int(x_center/2)
            # if y_center+2*half_h>=540:
            #     half_h=int((540-y_center)/2)
            # elif y_center-2*half_h<0:
            #     half_h=int(y_center/2)
            cv2.rectangle(nimg,(int(output[idx][2]-output[idx][4]/2),int(output[idx][3]-output[idx][5]/2)),(int(output[idx][4]/2+output[idx][2]),int(output[idx][5]/2+output[idx][3])),color=(0,0,0),thickness=2)
            # Draw_circle(output[idx],len(output[idx]),nimg)
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]
            # depth=depth[y_cent    all_work_count+=1er-half_h:y_center+half_h,x_center-half_w:x_center+half_w]
            # mutex_temp=threading.Lock()
            # cv2.imwrite(f"{}")
            # cv2.imwrite(f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/{person_index}.jpg",change_image)
            if start_work==False:
                task_new=threading.Thread(target=mediapipe_detect,args=(change_image,half_w*2,half_h*2,x_center,y_center,depth,idx,))
                task_new.start()
            else :
                task_new=threading.Thread(target=mediapipe_detect_2,args=(change_image,half_w*2,half_h*2,x_center,y_center,depth,idx,))
                task_new.start()
            person_index+=1
            # mm=threading.Lock()
            # mediapipe_task(change_image,mm)
        count+=1
        fps=1/(time.time()-last_time)
        last_time=time.time()    
    # 打开摄像头
        # cv2.putText(nimg,"fps"+str(fps),(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,0),thickness=1)
        # print(count)    
        # cv2.imshow("ning",nimg)
        # cv2.waitKey(60)
        all_work_count+=1
        mutex1.release()
        # cv2.destroyAllWindows()
    else:
        # rospy.loginfo("this is loc_quene size %d",loc_quene.qsize())
        if (not loc_quene.empty()):
            # rospy.loginfo(l)
            if start_work==False:
                if choose_face_or_pose_flag==1:
                    loc_target:MoveBaseGoal
                    loc_target,image_name=loc_quene.get()
                    
                    goal=judge_arrive()
                    goal.x = loc_target.target_pose.pose.position.x
                    goal.y=loc_target.target_pose.pose.position.y 
                    judge_can_or_not_go_pub.publish(goal)
                    m=255
                    while True:
                        try:
                            m=empty_list[empty_list_index]
                            empty_list_index+=1
                            break
                        except BaseException as  e:
                            pass
                    rospy.loginfo("this is %d index judge is %d",empty_list_index,m)
                    if m<253:
                        loc_target_pub.publish(loc_target)
                    rospy.loginfo("goal  have published")
                else :
                    face_feature,name_recordeed=loc_quene.get()
                    rospy.loginfo("this is namerecorded %s",name_recordeed)
                    rospy.loginfo("face_feature  have published")
                    
                    face_feature_pub.publish(name_recordeed)
                    choose_face_or_pose_flag=0
                    all_work_count=0
            else :
                loc_target,face_name,image_name=loc_quene.get()    
                rospy.loginfo("published message")
                if len(face_db_store_name)==0:               
                    return
                else:
                    rospy.loginfo("have accepted and enter the pose2 node")
                    # if person_num>=3:
                    # #     return               
                    if face_name=='unknown':
                        return
                    now_face_name=face_name
                    goal=judge_arrive()
                    goal.x = loc_target.target_pose.pose.position.x
                    goal.y=loc_target.target_pose.pose.position.y 
                    judge_can_or_not_go_pub.publish(goal)
                    m=255
                    if choose_face_or_pose_flag==3:
                        while True:
                            try:
                                m=empty_list[empty_list_index]
                                empty_list_index+=1
                                break
                            except BaseException as  e:
                                pass
                        if m>=253:
                            loc_target=MoveBaseGoal()
                            loc_target.target_pose.header.frame_id="None"
                    rospy.loginfo("this is %d index judge is %d",empty_list_index,m)
                    rospy.loginfo("this is now face_name %s",now_face_name)
                    # now_face_name=now_face_name.replace("person","")
                    # rospy.loginfo("this is target x1 y1 %f %f",loc_target.)
                    # if now_face_name in face_db_store_name and m<253:
                    try:
                        face_db_store_name.remove(now_face_name)
                    except BaseException as e:
                        pass
                    goal_name.name=face_name
                    goal_name.goal=loc_target
                    rospy.loginfo("refresh msg has sent")
                    now_goal_pub.publish(goal_name)
                all_work_count+=1
        # else :
        #     all_work_count=0
def get_map_pose(x, y):
    ps = PointStamped()
    ps.header.frame_id = "base_footprint"
    ps.point.x = x
    ps.point.y = y
    ps.header.stamp = rospy.Time.now()
    ps_new = tfBuffer.transform(ps, "map", rospy.Duration(1))
    return ps_new.point.x, ps_new.point.y
def real_pose(u_img, v_img, d):
    global p_x, p_y, f_x, f_y
    Z = d/math.sqrt(1+(u_img-p_x)**2/f_x**2+(v_img-p_y)**2/f_y**2)
    X = (u_img-p_x)*Z/f_x
    Y = (v_img-p_y)*Z/f_y
    y=abs(Y/1000)
    zzz= d/1000*math.cos(0.4)-y*math.sin(0.4)+0.04
    yyy=1.58-d/1000*math.sin(0.4)-y*math.cos(0.4)
    return X/1000+0.015, yyy, zzz
def convert_machine_axis_to_world(results,width,height,x_mid,y_mid,depth,name):
    global machine_odom_now_y,machine_odom_now_x,mathince_theta
    
    right_hip_x = int(results.pose_landmarks.landmark[24].x * width + x_mid - 0.5 * width)
    right_hip_y = int(results.pose_landmarks.landmark[24].y * height + 
        y_mid - 0.5 * height)
    left_hip_x = int(results.pose_landmarks.landmark[23].x * width + x_mid - 0.5 * width)
    left_hip_y = int(results.pose_landmarks.landmark[23].y * height + 
        y_mid - 0.5 * height)
    left_shoulder_x = int(results.pose_landmarks.landmark[11].x * width + x_mid - 0.5 * width)
    left_shoulder_y = int(results.pose_landmarks.landmark[11].y * height + 
        y_mid - 0.5 * height)
    right_shoulder_x = int(results.pose_landmarks.landmark[12].x * width + x_mid - 0.5 * width)
    right_shoulder_y = int(results.pose_landmarks.landmark[12].y * height + 
        y_mid - 0.5 * height)
    if right_hip_x > 960:
        right_hip_x = 959
    if right_hip_y > 540:
        right_hip_y = 539
    if left_hip_x > 960:
        left_hip_x = 959
    if left_hip_y > 540:
        left_hip_y = 539
    depth_right_hip = depth[right_hip_y][right_hip_x]
    depth_left_hip = depth[left_hip_y][left_hip_x]
    depth_left_shoulder = depth[left_shoulder_y][left_shoulder_x]
    depth_right_shoulder = depth[right_shoulder_y][right_shoulder_x]
    depths = [depth_left_hip, depth_left_shoulder, depth_right_hip, depth_right_shoulder]
    length = len(depths)
    for i in depths:
        if i < eps:
            length -= 1
    average_depth = sum(depths) / length
    # average_depth = depth[right_hip_y][right_hip_x]
    real_person = real_pose(x_mid, y_mid, average_depth)
    rospy.loginfo("this is robot x,z %f,%f",real_person[0],real_person[2])
    if real_person[2] < eps:
        return
    rospy.loginfo('x: %f, 深度: %f', real_person[0], average_depth)
    alpha = get_alpha(results)
    z_second = real_person[2] - 0.7* cos(alpha)
    x_second = real_person[0] + 0.7 * sin(alpha)
    if (alpha >= 0.45 * pi and alpha <= 0.55 * pi):
        z_second = real_person[2] - 0.75* cos(alpha)
        x_second = real_person[0] + 0.75 * sin(alpha)
        x_second += 0.5
        z_second+=0.45
    if (alpha >= 1.45 * pi and alpha <= 1.55 * pi):
        z_second = real_person[2] - 0.75* cos(alpha)
        x_second = real_person[0] + 0.75 * sin(alpha)
        x_second -= 0.5
        z_second+=0.45
    if alpha >= 0.8 * pi and alpha <= 1.2 * pi:
        z_second = real_person[2] - 1.7* cos(alpha)
        x_second = real_person[0] + 1.7 * sin(alpha)
    tho_second = math.sqrt(z_second ** 2 + x_second ** 2)
    theta_second = atan2(x_second, z_second)

    x1, y1 = get_map_pose(
        machine_odom_now_x+tho_second*cos(theta_second-mathince_theta), machine_odom_now_y-tho_second*sin(theta_second-mathince_theta))
    
    # y1=-y1
    rospy.loginfo("this is x1  %f and this is y1 %f and name is %s",x1,y1,name)
    robot_states = get_map_pose_theta()
    rpy = ori_to_rpy(
        0.0, 0.0, robot_states[2], robot_states[3])
    theta1 = alpha + rpy[2]
    rospy.loginfo('角度: %f', alpha)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x1
    goal.target_pose.pose.position.y = y1
    xyzw = rpy2quaternion(0.0, 0.0, theta1)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = xyzw[2]
    goal.target_pose.pose.orientation.w = xyzw[3]
    return goal
def ori_to_rpy(x, y, z, w):
    from tf import transformations
    (r, p, y) = transformations.euler_from_quaternion([x, y, z, w])
    return [r, p, y]
def rpy2quaternion(roll, pitch, yaw):
    x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    return [x, y, z, w]

def get_map_pose_theta():
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
    ps_new = tfBuffer.transform(ps, "map", rospy.Duration(1))
    return [ps_new.pose.position.x, ps_new.pose.position.y, ps_new.pose.orientation.z, ps_new.pose.orientation.w]


def task2(mutex,name):
    global flag,person_index_count,face_recognitio,now_flag,people_havebeen_recorder
    # print(now_flag)
    mutex.acquire()

    begin=time.time()

    img=cv2.imdecode(np.fromfile(name,dtype=np.uint8),-1)
    store_name=name.split('/')[-1]
    store_name=store_name.split('.')[0]
    result=face_recognitio.register(img,user_name=store_name)

    rospy.loginfo("result is %s",result)
    people_havebeen_recorder=1
    if result!="success":
        mutex.release()
        return None
    mutex.release()
    
    return "success"
def work(mutex:threading.Lock,image):
    mutex.acquire()
    results = mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    mutex.release()
    return results
def mediapipe_task(image,mutex,name,width,height,mid_x,mid_y,depth):
    
    global mediapipe_detect_count,mediape_mutex
    mutex.acquire()
    # cv2.imwrite("")
    # cv2.imwrite(f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/55565600{person_index}.jpg",image)
    
    # print(image)
    # mutex_temp=threading.Lock()
    # results=work(mutex_temp,image)
    mediape_mutex.acquire()
    results = mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    mediape_mutex.release()
    # cv2.imshow("key",image)

    # cv2.waitKey(10)
    final_rslt=None
    mediapipe_detect_count += 1
    if results.pose_world_landmarks!=None:
        print("+++++++++++++++++++++++++++++++++")
        print(results.pose_world_landmarks.landmark[11])
        alpha=get_alpha(results)
        alpha=alpha/pi*180
        print("this is alpha ",alpha ,"and name is ",name)
        rospy.loginfo("this is result, %s",name)
        final_rslt=convert_machine_axis_to_world(results,width,height,mid_x,mid_y,depth,name)
    else :
            # cv2.imwrite(f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/medi00{person_index}.jpg",image)
            pass
    mutex.release()
    return final_rslt   
def get_my_alpha(results):
    z_left = results.pose_world_landmarks.landmark[11].z
    z_right = results.pose_world_landmarks.landmark[12].z
    x_left = results.pose_world_landmarks.landmark[11].x
    x_right = results.pose_world_landmarks.landmark[12].x
    z_distance=(z_right-z_left)
    x_distance=(x_left-x_right)
    alpha=-math.atan(z_distance/x_distance)
    if (x_left<x_right and z_right>z_left):
        alpha-=pi
    elif (x_left<x_right and z_left>z_right):
        alpha+=pi
    print("my alpha get is ",alpha/pi*180," ",z_left," ",z_right," ",x_left," ",x_right )
    return alpha
def get_alpha(results):
    print(results)
    z_left = results.pose_world_landmarks.landmark[11].z
    z_right = results.pose_world_landmarks.landmark[12].z
    x_left = results.pose_world_landmarks.landmark[11].x
    x_right = results.pose_world_landmarks.landmark[12].x
    if abs(x_left - x_right) < 0.1:
        if z_left > z_right:
            return 0.5 * pi
        elif z_right > z_left:
            return 1.5 * pi
        else:
            rospy.loginfo('alpha 不合理')
            return 0
    elif abs(z_left - z_right) < 0.1:
        if x_left > x_right:
            return 0
        elif x_left < x_right:
            return pi
        else:
            rospy.loginfo('alpha 不合理')
            return 0
    elif x_left > 0 and z_left > 0:
        alpha = -(atan2(x_left, z_left) +
                  atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    elif x_left < 0 and z_left > 0:
        alpha = pi - (atan2(x_left, z_left) +
                      atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    elif x_left < 0 and z_left < 0:
        alpha = pi - (atan2(x_left, z_left) +
                      atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    elif x_left > 0 and z_left < 0:
        alpha = 2 * pi - (atan2(x_left, z_left) +
                          atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    alpha = -(atan2(results.pose_world_landmarks.landmark[11].x, results.pose_world_landmarks.landmark[11].z) +
              atan2(results.pose_world_landmarks.landmark[12].x, results.pose_world_landmarks.landmark[12].z)) * 0.5
    if alpha > 2 * pi:
        alpha -= 2 * pi
    return alpha
class Mediapipe_detect(threading.Thread):
    def __init__(self, target,image,mutex,path1,width,height,mid_x,mid_y,depth) -> None:
        self.target=target
        self.result=None
        self.image=image
        self.target=target
        self.mutex=mutex
        self.path=path1
        self.width=width
        self.height=height
        self.mid_x=mid_x
        self.mid_y=mid_y
        self.depth=depth
        super().__init__(target=target,args=(self.image,self.mutex,self.path,self.width,self.height,self.mid_x,self.mid_y,self.depth))
    def run(self) -> None:
        # super().run()
        self.result=self.target(self.image,self.mutex,self.path,self.width,self.height,self.mid_x,self.mid_y,self.depth)
    def return_results(self):
        return self.result
class Face_detect(threading.Thread):
    def __init__(self,target,mutex,name):
        super().__init__(target=target)
        self.target=target
        self.mutex=mutex
        self.result=None
        self.name=name
    def run(self) -> None:
        self.result=self.target(self.mutex,self.name)
    def return_result(self):
        return self.result   
def put_loc_into_quene(data_of_loc):
    global loc_quene,mutex_quene,people_can_get_count
    mutex_quene.acquire()
    rospy.loginfo("people_all_count is %d",people_can_get_count)
    loc_quene.put(data_of_loc)
    mutex_quene.release()
def mediapipe_detect(image:np.ndarray,width,height,mid_x,mid_y,depth,name):
    global mp_pose, mediapipe_detect_count,person_index,people_can_get_count,choose_face_or_pose_flag
    # mutex_all.acquire()
    # print(len(image.shape))
    last_person=person_index
    if (image.size==0):
        # mutex_all.release()
        return
    try:
        cv2.imwrite(f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/{name}{last_person}.jpg",image)
    except BaseException as e:
        pass
    image=cv2.imread(f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/{name}{last_person}.jpg")
    mutex_new1=threading.Lock()
    mutex_new2=threading.Lock()
    if choose_face_or_pose_flag==1:
        task_me=Mediapipe_detect(target=mediapipe_task,image=image,mutex=mutex_new1,path1=f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/{name}{last_person}.jpg",width=width,height=height,mid_x=mid_x,mid_y=mid_y,depth=depth)
        task_me.start()
        task_me.join()
        pose_feature=task_me.return_results()
        tuple_all=(pose_feature,f"{name}{last_person}")
        if pose_feature!=None:
            put_loc_thread=threading.Thread(target=put_loc_into_quene,args=(tuple_all,))
        # person_index+=1
            put_loc_thread.start()
    elif choose_face_or_pose_flag==2:
        store_path=f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face/{name}{last_person}.jpg"
        task_face=Face_detect(target=task2,mutex=mutex_new2,name=store_path)
        task_face.start()
        task_face.join()
        face_feature=task_face.return_result()
        name_recored=f"{name}{last_person}"
        if face_feature==None:
            name_recored="None"
        tuple_all=(face_feature,name_recored)
        put_loc_thread=threading.Thread(target=put_loc_into_quene,args=(tuple_all,))
    # person_index+=1
        put_loc_thread.start()
    # if pose_feature!=None:
def judge_arrive_result_cb(msg:Int32):
    global empty_list
    empty_list.append(msg.data)
def start_and_end_dopose2(msg:String):
    global start_work,choose_face_or_pose_flag,all_work_count
    rospy.loginfo("main_message have accepted")
    if msg.data=="1":
        start_work=True
        add_name_to_list()
    else:
        start_work=False
        choose_face_or_pose_flag=0
        all_work_count=0

def mediapipe_detect_2(image:np.ndarray,width,height,mid_x,mid_y,depth,name):
    global mp_pose, mediapipe_detect_count,person_index,people_can_get_count,choose_face_or_pose_flag,all_work_count

    # mutex_all.acquire()
    # print(len(image.shape))
    last_person=person_index
    if (image.size==0):
        # mutex_all.release()
        return
    try:
        cv2.imwrite(f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face2/{name}{last_person}.jpg",image)
    except BaseException as e:
        pass
    mutex_new1=threading.Lock()
    mutex_new2=threading.Lock()
    store_path=f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face2/{name}{last_person}.jpg"
    task_face=Face_detect(target=task3,mutex=mutex2,name=store_path)
    if choose_face_or_pose_flag==3:
        task_me=Mediapipe_detect(target=mediapipe_task,image=image,mutex=mutex1,path1=f"/home/linxi/ServiceRobot-General/src/pose_estimate/data_face2/{name}{last_person}.jpg",width=width,height=height,mid_x=mid_x,mid_y=mid_y,depth=depth)
        task_me.start()
        task_face.start()
        task_me.join()
        task_face.join()
        pose_feature=task_me.return_results()
        face_feature=task_face.return_result()
    else :
        task_face.start()
        task_face.join()
        rospy.loginfo("nooooooooooooooooo!!!!!!!!!!!!!!!111")
        pose_feature=MoveBaseGoal()
        pose_feature.target_pose.header.frame_id="None"
        face_feature=task_face.return_result()
    # return results
    # if pose_feature!=None:
    if pose_feature!=None:
        rospy.loginfo("!!!!!!!!!!!!!!!!!!11")
        tuple_all=(pose_feature,face_feature,f"{name}{last_person}.img")
        global machine_odom_now_y,machine_odom_now_x,mathince_theta
        # tuple_all=1
        put_loc_thread=threading.Thread(target=put_loc_into_quene,args=(tuple_all,))
        # person_index+=1
        put_loc_thread.start()
    # loc_quene.put(tuple_all)

        # people_can_get_count+=1
    # mutex_all.release()
    
def get_odom_message(odom:Odometry):
    global machine_odom_now_y,machine_odom_now_x,mathince_theta
    # if last_time_postion_z==now_time_postion_z:
    #     target_position_z=last_time_postion_z
    #     machine_odom_last_x=machine_odom_now_x
    #     machine_odom_last_y=machine_odom_now_y
    machine_odom_now_x= odom.pose.pose.position.x
    machine_odom_now_y=odom.pose.pose.position.y 

    odom_ox=odom.pose.pose.orientation.x
    odom_oy=odom.pose.pose.orientation.y
    odom_oz=odom.pose.pose.orientation.z
    odom_ow=odom.pose.pose.orientation.w
        # rospy.loginfo("this is odom_msg%f,%f,%f,%f",self.odom_oz,self.odom_ow,self.odom_oy,self.odom_ox)
        
    _,_,mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow])
    
def choose_face_or_pose_cb(msg:String):
    global choose_face_or_pose_flag,all_work_count,start_work
    all_work_count=0
    choose_face_or_pose_flag=int(msg.data)
    if choose_face_or_pose_flag in [0,1,2]:
        start_work=False
    if choose_face_or_pose_flag==0:
        all_work_count=0
    rospy.loginfo("i have accepted message")
if __name__=="__main__":
    # global mp_pose
    # global mutex1
    # time.sleep(4)
    rospy.init_node("pose")
    # time.sleep(2)
    start_and_end_pose2=rospy.Subscriber("general_service_now_goals",String,start_and_end_dopose2)
    file_in_face_db=os.listdir('/home/linxi/ServiceRobot-General/src/pose_estimate/face_db')
    
    if len(file_in_face_db)!=0:
        for i in file_in_face_db:
            path_file=os.path.join('/home/linxi/ServiceRobot-General/src/pose_estimate/face_db',i)
            os.remove(path_file)
    # 用以删除文件
    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    choose_face_or_pose_sub=rospy.Subscriber("/general_service_choose_face_or_pose",String,choose_face_or_pose_cb,queue_size=10)
    loc_target_pub=rospy.Publisher("/general_service_loc_target",MoveBaseGoal,queue_size=10)
    face_feature_pub=rospy.Publisher("/general_service_recognition",String,queue_size=10)
    judge_can_or_not_go_pub=rospy.Publisher("judge_arrive_target",judge_arrive,queue_size=10)
    judge_can_or_not_go_sub=rospy.Subscriber ("judge_arrive_result",Int32,judge_arrive_result_cb,queue_size=10)
    now_goal_pub = rospy.Publisher("/person/waypoint", Goals_name, queue_size=100)
    ts.registerCallback(image_callback)
    odom_grab_position_subscriber=rospy.Subscriber("odom",Odometry,get_odom_message,queue_size=10)
    tfBuffer = tf2_ros.Buffer    
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # task2()

    rospy.spin()
    
 
 
 