#!/usr/bin/python3
#上面的“#！”是python的执行路径，不要删掉否则会报错
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

#添加执行路径
sys.path.insert(0,'/home/lzh/test/src/vis')

""" 
iridium_rs.py
专门用于处理需要通过realsense相机实现的功能,如:拍摄地上的垃圾
如果不知道前面写的是什么，请直接拉到最下面看主函数
 """
spin_count = 0
# 全局变量类，用于定义需要全局化的变量（这样便不用每次都global）
class GlobalVar:
    eps = 1e-2# 一个小值，防止出现无穷大
    followflag = -1# 是否需要跟随？-1为初始值，1开始旋转，旋转后返回0
    frame = 1# 实际上并不是帧数的意思，只是一个是否需要进入图像回调的标记
    reaction_flag = -1# 需要处理任务的编号，5为识别垃圾，1为将垃圾桶放在视野中心
    rospy.init_node("realsense")# 初始化节点，名字随便
    bridge = CvBridge()# 实例化，将image信息转化为cv2格式，即矩阵形式

    cb_mutex = threading.Lock()# 线程锁，作用见下
    mediapipe_mutex = threading.Lock()

# yolo类,识别垃圾和垃圾桶
class Yolov8:
    """ 
     init:初始化函数,定义初始化模型路径
     model1：垃圾模型
     model2：垃圾桶模型
       """
    def __init__(self,model_path1="/home/lzh/test/src/main_function/models/models_all/best_more_all_rubbish.pt",
                 model_path2="/home/lzh/test/src/main_function/models/models_all/best_bin.pt"):
        self.model1 = YOLO(model_path1)# 自己去看YOLO的用法
        self.model2 = YOLO(model_path2)

    """ 
    detect函数，对图像进行检测，并返回裁剪后的图像
    输入：图片
    输出：裁剪后的图片，框的宽度和高度，框的中心点，标签 
    """
    def detect(self,image,num):
        results = list()
        nimg = image
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        if num==1:
            output = self.model1(image)
        if num==2:
            output = self.model2(image)
        name = list()
        # 上面的都很简单
        for r in output:
            """ 
            output预测结果,里面有很多我也不清楚的东西,只知道是这个用法
            r是一个结构体,包含了names,boxes等元素
            names是一个字典,存储了识别结果和索引
            boxes是一个结构体,里面存储了类别索引,置信度等等信息,具体可以自己探索
               """
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
            # 如果没有检测到任何对象
            if len(r.boxes) == 0:
                # 如果=0就是没检测到任何东西
                continue
        for i in range (len(name)):
            xywh = boxes.cpu().xywh.detach().numpy()
            # 上面的用法是把boxes中的xywh结构转化为numpy数组,返回的是二维数组,第一个维度是识别到物体的索引,第二个维度见下
            # x:物品识别的框中心的x坐标,单位:像素点,原点在左上角,y同理,w是宽度,即横向,h是高度,即纵向
            x_center = int(xywh[i][0])
            y_center = int(xywh[i][1])
            half_w = int(xywh[i][2]/2)
            half_h = int(xywh[i][3]/2)
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]#裁减之后的框
            # 把下面两行取消注释,可以看到输出的照片,即裁剪之后的照片
            # image_name = f"/home/lzh/{name[i]}.jpg"
            # cv2.imwrite(image_name,change_image)

            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))
            # results是一个元组
        # 考虑到模型可能出错，这里只取置信度最大的那一个           
        max_index = -1
        confidence_max = -1
        for i in range (len(name)):
            if name[i][1] > confidence_max:
                    confidence_max = name[i][1]
                    max_index = i
        max_result = list()
        max_result = [(0,0,0,0,0,"nothing")]# 防止输出一个空列表从而报错
        if not max_index == -1:
            max_result = list()#如果不是空列表，重新置空
            max_result.append(results[max_index])
        # print(max_result)
        return max_result
  
    """ 
    rotate函数:当垃圾桶不在视野正中心的时候，旋转机器人让垃圾桶在视野中心
    输入：图像，图像的宽度，高度（在realsense中是1280和960），垃圾桶中心的xy坐标，物品名字，实际上只有宽和x中心坐标是有用的，懒得改了
    输出：无
      """
    def rotate(self, image, width, height, mid_x, mid_y, name):
        GlobalVar.mediapipe_mutex.acquire()#获得线程锁，防止在处理一帧时下一帧的信息已经到来
        cmdvel = Twist()# 发布速度信息的实例化
        x_average = mid_x# 没有用的操作
        print(f"the x_average is {x_average}")
        cmdvel.linear.x = 0
        cmdvel.linear.y = 0
        cmdvel.linear.z = 0
        cmdvel.angular.x = 0
        cmdvel.angular.y = 0
        if x_average < width / 2 :# 垃圾桶在左边就往左边转，在右边就往右边转
            cmdvel.angular.z = 1
        else:
            cmdvel.angular.z = -1
        vel_pub.publish(cmdvel)
        rospy.sleep(0.3)#每次转0.3弧度试试
        # 注意，这个参数受地面影响，可能会因为打滑等因素不能及时启动，导致实际上转的时间小于这个值，在现场务必调试
        cmdvel.angular.z=0
        vel_pub.publish(cmdvel)# 回到静止
        rospy.sleep(0.5)
        GlobalVar.mediapipe_mutex.release()# 释放线程锁
        return None

""" 
图像回调函数，处理相机传回来的照片，这个是最重要的一部分
输入：图像的rgb矩阵
输出：无
实际上，图像回调是自始至终都在进行，但我们只在需要的时候给主函数发信息
 """
last_detection_time = 0    # 最后一次检测的时间
def image_callback(image_rgb):

    global image, last_detection_time, spin_count
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')# 获取图片，image是一个含有颜色信息的矩阵
    r, b, g = cv2.split(image)
    image = cv2.merge([g,b,r])# rs的rgb通道顺序和cv2不一样，需要重新组合，为什么这样组合？试出来的
    #cv::Mat
    GlobalVar.cb_mutex.acquire() #获得线程锁，防止在处理一帧时下一帧的信息已经到来
    if GlobalVar.frame == 0 :# 开始处理图像
        if GlobalVar.reaction_flag == 5:# 检测垃圾
            # rospy.loginfo(f"Now the task is 5")
            result = yolov8.detect(image, 1) 
            # cv2.imshow("hhh",image)
            # cv2.waitKey(0)
            # rospy.loginfo(f"任务5检测到{result[0][5]}")
            collect_robbish_pub.publish(result[0][5])# 因为只会有一个垃圾（置信度最大的那个），所以直接[0][5]，发布垃圾的名字给主函数
            GlobalVar.reaction_flag = -1# 防止不停发消息给主函数
            # cv2.imwrite(store_path,image)
            GlobalVar.frame = 1 #将frame设置为1，防止重复处理

        elif GlobalVar.reaction_flag == 1:# 找垃圾桶
            if (time.time() - last_detection_time > 2):
                last_detection_time = time.time()
                # print(last_detection_time)
                # cv2.imwrite("/home/lzh/rotate.jpg", image)
                result = yolov8.detect(image, 2)
                # cv2.imshow("jhh",image)
                for res in result:
                    if res[5] == 'bin':
                        GlobalVar.followflag = 1
                if GlobalVar.followflag == 1 :
                    #只有同时收到抓取的信息和识别到垃圾桶,才开始转
                    rospy.loginfo(f"Now the task is follow")
                    if (res[5]=="bin"):
                        """ 
                         86是阈值，单位是像素点。为什么是86？实际上是试出来的合适值。小于这个阈值，则认为对准垃圾桶了
                         然而，为了防止转的角度过大，导致一直左右横跳，我们设置最大旋转次数 
                           """
                        if (not(abs(res[3] - 640) < 86) and spin_count < 8):
                            cv2.imwrite("/home/lzh/rotate.jpg", image)
                            spin_count += 1
                            yolov8.rotate(image, 1280,720,res[3],res[4],res[5])
                        else:
                            # 这时已经转到视野中央，停止旋转，停止处理回调图像
                            GlobalVar.followflag = 0
                            GlobalVar.frame = 1
                            spin_count = 0
                            res_pub.publish("spin ok")# 发布信息
    GlobalVar.cb_mutex.release() #释放互斥锁

# 通信的回调函数，接收到话题就做出相应的动作
def spin_sub(msg:String):
    global spin_count
    if msg.data == "spin":
        GlobalVar.reaction_flag = 1
        GlobalVar.frame = 0
        # rospy.loginfo("---------received spin signal--------")


def collect_robbish_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=5
        # rospy.loginfo(f"reaction flag:{GlobalVar.reaction_flag}")
        # rospy.loginfo(f"frame:{GlobalVar.frame}")

if __name__ == "__main__":
    yolov8 = Yolov8()
    # yolo类的实例化
    rgb_sub = rospy.Subscriber("/camera/color/image_raw",Image,image_callback)
    # 接收rs相机传回的图像rgb参数，双引号内是话题，类型是image,直接作为参数传递给image callback

    res_sub = rospy.Subscriber("robot_spin",String,spin_sub)
    # 接收抓取那边传回的是否需要旋转的话题，关于旋转是什么，请转到Yolov8.rotate函数

    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    # 发布速度信息，控制机器人的移动
    res_pub = rospy.Publisher("robot_spin_reply",String,queue_size=10)
    # 告诉抓取我已经转好了
    collect_robbish = rospy.Subscriber("collect_robbish",String,collect_robbish_callback)
    # 接收主函数识别垃圾的话题，开始识别垃圾
    collect_robbish_pub = rospy.Publisher("collect_robbish_reply", String, queue_size=10)
    # 告诉主函数我认完了
    rospy.spin()
