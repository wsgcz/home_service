import matplotlib.pyplot as plt
import torch
import cv2
from torchvision import transforms
import numpy as np
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
# 加载模型
import tf2_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import os
import message_filters
import multiprocessing
import math
w_img = 960  # 图像宽度
h_img = 540  # 图像高度
p_x = 479 
p_y = 269
f_x = 540.68603515625
f_y = 540.68603515625
Image_show_count=0
image = Image()
depth = Image()
bridge = CvBridge()
eps = 0.01
mediapipe_detect_count = 0
begin_time=time.time()
goals=[]
end_goal=MoveBaseGoal()
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
weigths = torch.load('yolov7-w6-pose.pt')
model = weigths['model']
model = model.half().to(device)
_ = model.eval()
flag=0
mutex1=threading.Lock()
mutex2=threading.Lock()
mutex3=threading.Lock()
mutex_quene=threading.Lock()
image_all_count=0
now_flag=0
Image_recorded=0
person_index_count=0
person_index=0
begin_recogine=None
def Draw_circle(posetion_arr:list,length:int,img):
    for i in range(7,length,3):
        pos_x=int(posetion_arr[i])
        pos_y=int(posetion_arr[i+1])
        print(pos_x)
        # pos_y=posetion_arr[i]
        # cv2.circle(img,(int(pos_x),int(pos_y)),2,thickness=2)
        cv2.putText(img,"index"+str(i),(pos_x+10,pos_y+10),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),thickness=1)
# 读取摄像头画面
face_recognitio = FaceRecognition()
def stop():
    vel_cmd=Twist()
    vel_cmd.linear.x=0
    vel_cmd.linear.y=0
    vel_cmd.linear.z=0
    vel_cmd.angular.x=0
    vel_cmd.angular.z=0
    vel_cmd.angular.z=0
    vel_pub.publish(vel_cmd) 
    time.sleep(1)
def real_pose(u_img, v_img, d):
    global p_x, p_y, f_x, f_y
    Z = d/sqrt(1+(u_img-p_x)**2/f_x**2+(v_img-p_y)**2/f_y**2)
    X = (u_img-p_x)*Z/f_x
    Y = (v_img-p_y)*Z/f_y
    y=abs(Y/1000)
    zzz= d/1000*math.cos(0.52)-y*math.sin(0.52)+0.04
    yyy=1.58-d/1000*math.sin(0.53)-y*math.cos(0.53)
    return X/1000+0.015, yyy, zzz

def task1(image:Image,depth:Image):
    global flag,person_index,Image_recorded,image_all_count
    last_time=0
    url=0
    now_time=time.time()
    # cap = cv2.VideoCapture(url)
    t=time.time()
    count=0
    begin_time=time.time()
    # while (cap.isOpened()):
    # mutex1.acquire()
    # ret, image = cap.read()
    #image = cv2.imread('xiaolu.jpg')
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

    area=[]
    for idx in range(output.shape[0]) :
        area.append(output[idx,5]*output[idx,4])
    # area.sort(are)
    for i in range(len(area)):
        for j in range(i,len(area),1):
            if (area[i]<area[j]):
                output[i],output[j]=output[j],output[i]
                area[i],area[j]=area[j],area[i]
            
    image_all_count+=1    
    for idx in range(output.shape[0]):
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        # print(output[idx,2:6])
        x_center=int(output[idx,2])
        y_center=int(output[idx,3])
        half_w=int(output[idx,4]/2)
        half_h=int(output[idx,5]/2)
        cv2.rectangle(nimg,(int(output[idx,2]-output[idx,4]/2),int(output[idx,3]-output[idx,5]/2)),(int(output[idx,4]/2+output[idx,2]),int(output[idx,5]/2+output[idx,3])),color=(0,0,0),thickness=2)
        # Draw_circle(output[idx],len(output[idx]),nimg)
        if (image_all_count%10==0 ) :
            flag+=1
            print("zzzzzzzzzzzz")
            # chaneg_image=cv2.crop (nimg,(int(output[idx,2]-output[idx,4]/2),int(output[idx,3]-output[idx,5]/2)),(int(output[idx,4]/2+output[idx,2]),int(output[idx,5]/2+output[idx,3])))
            # change_image=nimg[int(output[idx,2]-output[idx,4]/2):int(output[idx,2]+output[idx,4]/2),int(output[idx,3]+output[idx,5]/2):int(output[idx,3]+output[idx,5]/2)]
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]
            now_t=time.time()
            if len(change_image)!=0 :
                if (now_t-begin_time<20):
                    print("zzzzzzzzzzzzzzz")
                    try:
                        cv2.imwrite(f"data_face/00{person_index}.jpg",change_image)
                    except BaseException as e:
                        pass
                    new_task=threading.Thread(target=task2)
                    new_task.start()
                    now_time=time.time()
                else :
                    task3=threading.Thread(target=recognize_image,args=(nimg,))
                    task3.start()
                    
                person_index+=1
                print(time.time()-begin_time)
                count+=1
        if (now_time-t>1):
            # print(area)
            t=now_time
    fps=1/(time.time()-last_time)
    last_time=time.time()    
# 打开摄像头

    # print(count)    
    # cv2.namedWindow("show",cv2.WINDOW_NORMAL)
    # cv2.imshow("show",nimg)
    cv2.waitKey(1)
    # mutex1.release()
    return nimg
# cv2.destroyAllWindows()
def recognize_image(change_image:Image):
    mutex3.acquire()          
    rslt=face_recognitio.recognition(change_image)
    if len(rslt)!=0:
        cv2.imwrite(f"data_face/00{person_index}.jpg",change_image) 
        print(f"person_index{person_index}:"+rslt[0])
    mutex3.release()
def task2():
    mutex2.acquire()
    global flag,person_index_count,face_recognitio,now_flag,Image_recorded,begin_recogine
    print("i have recognized someone ")
    begin=time.time()
    names_list =os.listdir("data_face")
    names_list.reverse()
    
    # if Image_recorded==0:
    for index,data in enumerate(names_list):
        if index<person_index_count : 
            continue
        print("nooooooooooooooooooooooooooo")
        img = cv2.imdecode(np.fromfile(os.path.join("data_face",data), dtype=np.uint8), -1)
        result = face_recognitio.register(img, user_name=f'person{person_index_count}')
        person_index_count+=1
        print(result)
        # results = face_recognitio.recognition(img)
        now_flag=flag
    Image_recorded=1
    final_l=os.listdir("face_db")
    if (len(final_l)==3):
        print("the last time is",time.time()-begin_recogine)
        exit(0)
    mutex2.release()
def put_data_into_quene(data):
    global mutex_quene,loc_qu
    mutex_quene.acquire()
    
def image_callback(image_rgb: Image, image_depth: Image):
    global image, depth, begin_recogine,eps, goals, bridge,Image_show_count,Image_recorded
    print(Image_show_count)
    Image_show_count+=1
    if Image_show_count<100:
        return
    if Image_show_count==100:
        begin_recogine=time.time()
    image=bridge.imgmsg_to_cv2(image_rgb, desired_encoding='passthrough')
    depth=bridge.imgmsg_to_cv2(image_depth, desired_encoding='passthrough')
    nimg=task1(image,depth)
    # show_img(nimg)
    # cv2.putText(nimg,"fps"+str(fps),(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,0),thickness=1)
    # cv2.imshow("ning",image)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # print(22222)
    # show_task=multiprocessing.Process(target=show_img,args=(image,))
    # show_task=threading.Thread(target=show_img,args=(image,))
    # show_task.start()
    # show_task.join()
    
    # cv2.imshow("show",nimg)
    # cv2.waitKey(10)
    # cv2.destroyAllWindows()
def sum_func(img:Image,depth:Image):
    now_task1=threading.Thread(target=task1,args=(img,depth,))
    now_task2=threading.Thread(target=task2)
    now_task1.start()
    # now_task2.start()
if __name__=="__main__":
    rospy.init_node("yolo-pose_intemediate")
    # time.sleep(4)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    # time.sleep(4)
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub],10)
    ts.registerCallback(image_callback)
    rospy.spin()

    # sum_func()    
    # task2()

    
    
 
 
 