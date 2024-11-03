#!/usr/bin/python3
import os,cv2,sys,time,torch,rospy,tf2_ros,threading
import numpy as np
from ultralytics import YOLO

from PIL import Image
from math import atan2,pi,sqrt,sin,cos
from queue import Queue
from cv_bridge import CvBridge
from tf import transformations
from tf2_geometry_msgs import PointStamped,PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
sys.path.insert(0,'/home/lzh/test/src/vis')
spin_count = 0

class GlobalVar:
    eps = 1e-2
    P_x = 479 
    P_y = 269 
    f_x = 540.68603515625
    f_y = 540.68603515625
    followflag = -1
    machine_odom_now_x = 0
    machine_odom_now_y = 0
    machine_theta = 0
    frame = 1
    last_person = 0
    reaction_flag = -1
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
    def __init__(self,model_path1="/home/lzh/test/src/main_function/models/models_all/best_more_all_rubbish.pt",
                 model_path2="/home/lzh/test/src/main_function/models/models_all/best_bin.pt"):
        self.model1 = YOLO(model_path1)
        self.model2 = YOLO(model_path2)

###detect函数，对图像进行检测，并返回裁剪后的图像
###输入：图片
###输出：裁剪后的图片，框的宽度和高度，框的中心点，标签
    def detect(self,image,num):
        results = list()
        nimg = image
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        if num==1:
            output = self.model1(image)
        if num==2:
            output = self.model2(image)
        name = list()
        
        for r in output:
            names = r.names
            boxes = r.boxes
            for box in boxes:
                # 获取类别索引
                class_id = int(box.cls[0])
                # 获取类别名称
                class_name = names[class_id]
                # 获取置信度
                confidence = float(box.conf[0])
                name.append((class_name, confidence))
                # print(f"检测到: {class_name}, 置信度: {confidence:.2f}")
            # 如果没有检测到任何对象
            if len(r.boxes) == 0:
                continue
        for i in range (len(name)):
            xywh = boxes.cpu().xywh.detach().numpy()
            x_center = int(xywh[i][0])
            y_center = int(xywh[i][1])
            half_w = int(xywh[i][2]/2)
            half_h = int(xywh[i][3]/2)
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]#裁减之后的框
            # image_name = f"/home/lzh/{name[i]}.jpg"
            # cv2.imwrite(image_name,change_image)
            # 宽 高 
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))
        max_index = -1
        confidence_max = -1
        for i in range (len(name)):
            if name[i][1] > confidence_max:
                    confidence_max = name[i][1]
                    max_index = i
        max_result = list()
        max_result = [(0,0,0,0,0,"nothing")]
        if not max_index == -1:
            max_result = list()
            max_result.append(results[max_index])
        # print(max_result)
        return max_result
  
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
        #TODO
        #
        rospy.sleep(0.3)#每次转0.5弧度试试
        cmdvel.angular.z=0
        vel_pub.publish(cmdvel)
        rospy.sleep(0.5)
        GlobalVar.mediapipe_mutex.release()
        return None
    
last_detection_time = 0    
def image_callback(image_rgb):
    #print("---------- i am in the callback --------------")
    global image, last_detection_time, spin_count
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')
    r, b, g = cv2.split(image)
    image = cv2.merge([g,b,r])
    #cv::Mat
    GlobalVar.cb_mutex.acquire()
    if GlobalVar.frame == 0 :
        if GlobalVar.reaction_flag == 5:
            rospy.loginfo(f"Now the task is 5")
            result = yolov8.detect(image, 1)
            # cv2.imshow("hhh",image)
            # cv2.waitKey(0)
            rospy.loginfo(f"任务5检测到{result[0][5]}")
            collect_robbish_pub.publish(result[0][5])
            GlobalVar.reaction_flag = -1
            # cv2.imwrite(store_path,image)
            GlobalVar.frame = 1

        elif GlobalVar.reaction_flag == 1:
            if (time.time() - last_detection_time > 2):
                last_detection_time = time.time()
                print(last_detection_time)
                cv2.imwrite("/home/lzh/rotate.jpg", image)
                result = yolov8.detect(image, 2)
                # cv2.imshow("jhh",image)
                for res in result:
                    if res[5] == 'bin':
                        GlobalVar.followflag = 1
                if GlobalVar.followflag == 1 :
                    #只有同时收到抓取的信息和识别到垃圾桶,才开始转
                    rospy.loginfo(f"Now the task is follow")
                    if (res[5]=="bin"):
                        if (not(abs(res[3] - 640) < 86) and spin_count < 8):
                            cv2.imwrite("/home/lzh/rotate.jpg", image)
                            spin_count += 1
                            yolov8.rotate(image, 1280,720,res[3],res[4],res[5])
                        else:
                            GlobalVar.followflag = 0
                            GlobalVar.frame = 1
                            spin_count = 0
                            res_pub.publish("spin ok")
    GlobalVar.cb_mutex.release()

def spin_sub(msg:String):
    global spin_count
    if msg.data == "spin":
        GlobalVar.reaction_flag = 1
        spin_count = 0
        GlobalVar.frame = 0 # after tiao shi delete this
        rospy.loginfo("---------received spin signal--------")


def collect_robbish_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=5
        rospy.loginfo(f"reaction flag:{GlobalVar.reaction_flag}")
        rospy.loginfo(f"frame:{GlobalVar.frame}")

if __name__ == "__main__":
    yolov8 = Yolov8()
    rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image,image_callback)


    res_sub = rospy.Subscriber("robot_spin",String,spin_sub)

    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    res_pub = rospy.Publisher("robot_spin_reply",String,queue_size=10)
    collect_robbish = rospy.Subscriber("collect_robbish",String,collect_robbish_callback)

    collect_robbish_pub = rospy.Publisher("collect_robbish_reply", String, queue_size=10)
    rospy.spin()
