import os,cv2,sys,time,torch,rospy,tf2_ros,threading,insightface,message_filters
import numpy as np
import mediapipe as mp
import actionlib
import random
import matplotlib.pyplot as plt
from tqdm import tqdm
import numpy as np
from ultralytics import YOLO

from sklearn import preprocessing
from PIL import Image, ImageFont, ImageDraw
from math import atan2,pi,sqrt,sin,cos
from queue import Queue
from nav_msgs.msg import Odometry
from sklearn import preprocessing
from cv_bridge import CvBridge
from torchvision import transforms
from tf import transformations
from tf2_geometry_msgs import PointStamped,PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Twist
sys.path.insert(0,'/home/zzy/test/src/vis')

class GlobalVar:
    eps = 1e-2
    P_x = 479 
    P_y = 269 
    f_x = 540.68603515625
    f_y = 540.68603515625
    followflag = 0
    machine_odom_now_x = 0
    machine_odom_now_y = 0
    machine_theta = 0
    frame = 0
    last_person = 0
    reaction_flag = 0
    start_work = False
    rospy.init_node("realsense")
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    bridge = CvBridge()
    # goal_name = Goals_name()

    pub_queue = Queue(10)
    cb_mutex = threading.Lock()
    face_mutex = threading.Lock()
    queue_mutex = threading.Lock()
    mediapipe_mutex = threading.Lock()
    def ori_to_rpy(x, y, z, w):
        (r, p, y) = transformations.euler_from_quaternion([x, y, z, w])
        return [r, p, y]

    def rpy2quaternion(roll, pitch, yaw):
        x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
        y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
        z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
        w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
        return [x, y, z, w]

    # 摄像头节点坐标系到map坐标系的转变角，返回四元数
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
        ps_new = GlobalVar.tfBuffer.transform(ps, "map", rospy.Duration(1))
        return [ps_new.pose.position.x, ps_new.pose.position.y, ps_new.pose.orientation.z, ps_new.pose.orientation.w]

    # 将摄像头坐标系的坐标转化为机器人坐标系
    def get_map_pose(x, y):
        rospy.loginfo(f"machine_odom_now_pos: {x},{y}")
        ps = PointStamped()
        ps.header.frame_id = "base_footprint"
        ps.point.x = x
        ps.point.y = y
        ps.header.stamp = rospy.Time.now()
        ps_new = GlobalVar.tfBuffer.transform(ps, "map", rospy.Duration(1))
        return ps_new.point.x, ps_new.point.y
    
class Yolov8:
    def __init__(self,model_path="/home/zzy/vision/src/vis/scripts/best_rs.pt"):
        self.model = YOLO(model_path)
###detect函数，对图像进行检测，并返回裁剪后的图像
###输入：图片
###输出：裁剪后的图片，框的宽度和高度，框的中心点，标签
    def detect(self,image):
        results = list()
        nimg = image
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        output = self.model(image)
        name = list()
        for o in output:
            boxes = o.boxes
            names = o.names
            for box in boxes:
                # 获取类别索引
                class_id = int(box.cls[0])
                # 获取类别名称
                name.append(names[class_id])
            # 如果没有检测到任何对象
            if len(boxes) == 0:
                continue
        for i in range (len(name)):
            xywh = boxes.cpu().xywh.detach().numpy()
            x_center = int(xywh[i][0])
            y_center = int(xywh[i][1])
            half_w = int(xywh[i][2]/2)
            half_h = int(xywh[i][3]/2)
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]#裁减之后的框
            image_name = f"/home/zzy/{name[i]}.jpg"
            cv2.imwrite(image_name,change_image)
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i]))
        return results
  
    #确保人在视野的中心
    def rotate(self, image, width, height, mid_x, mid_y, name):
        GlobalVar.mediapipe_mutex.acquire()
        cmdvel = Twist()
        x_average = mid_x
        print(f"the x_average is {x_average}")
        cmdvel.linear.x = 0
        cmdvel.linear.y = 0
        cmdvel.linear.z = 0
        cmdvel.angular.x = 0
        cmdvel.angular.y = 0
        if x_average < width / 2 :
            cmdvel.angular.z = 1
        else:
            cmdvel.angular.z = -1
        vel_pub.publish(cmdvel)
        rospy.sleep(0.2)#每次转0.5弧度试试
        cmdvel.angular.z=0
        vel_pub.publish(cmdvel)
        rospy.sleep(0.5)
        GlobalVar.mediapipe_mutex.release()
        return None
    
def image_callback(image_rgb):
    global image, last_detection_time
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')
    GlobalVar.cb_mutex.acquire()
    if GlobalVar.frame == 0 :
        result = yolov8.detect(image)
        for res in result:
            if res[5] == 'bin':
                GlobalVar.followflag = 1
        if GlobalVar.followflag == 1 and GlobalVar.reaction_flag == 1:
            #只有同时收到抓取的信息和识别到垃圾桶,才开始转
            if (time.time() - last_detection_time < 0.5):
                print(114)
            else:
                last_detection_time = time.time()
                rospy.loginfo(f"Now the task is follow")
                if (res[5]=="bin"):
                    if (not(abs(res[3] - 640) < 50)):
                        cv2.imwrite("/home/zzy/rotate.jpg", image)
                        yolov8.rotate(image, 1280,720,res[3],res[4],res[5])
                    else:
                        GlobalVar.followflag = 0
        if (GlobalVar.followflag == 0):
            print("------------i am setting the followflag to 0----------")    
            res_pub.publish("spin ok")        
            GlobalVar.frame = 1
            GlobalVar.reaction_flag = 0
    GlobalVar.cb_mutex.release()

    rospy.loginfo(f"检测到{result[5]}")
    GlobalVar.reaction_flag = 0
    #cv2.imwrite(store_path,image)
    GlobalVar.last_person += 1

def spin_sub(msg:String):
    if msg.data == "spin":
        GlobalVar.reaction_flag = 1

if __name__ == "__main__":
    
    yolov8 = Yolov8()

    rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image)
    res_sub = rospy.Subscriber("/home_sevice/robot_spin",String,queue_size=10)

    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    res_pub = rospy.Publisher("/home_sevice/robot_spin_reply",String,queue_size=10)

    rospy.spin()
