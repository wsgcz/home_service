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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import message_filters
import mediapipe as mp
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
now_flag=0
person_index_count=0
person_index=0
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=1,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)

def mediapipe_detect(image):
    global mp_pose, mediapipe_detect_count
    results = mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if results.pose_landmarks is None:
        cv2.imwrite('/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/data/people_false' +
                    str(mediapipe_detect_count) + '.jpg', image)
        mediapipe_detect_count += 1
        return None
    print(results.pose_world_landmarks.landmark[11].z)
    return results

def Draw_circle(posetion_arr:list,length:int,img):
    for i in range(7,length,3):
        pos_x=int(posetion_arr[i])
        pos_y=int(posetion_arr[i+1])
        # print(pos_x)  
        if(i==25):
            print("this is x ", posetion_arr[i],"y ",posetion_arr[i+1]," z " , posetion_arr[i+2])
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
def task1():
    global flag,person_index
    last_time=0
    url=0
    now_time=time.time()
    cap = cv2.VideoCapture(url)
    t=time.time()
    count=0
    begin_time=time.time()
    while (cap.isOpened()):
        mutex1.acquire()
        ret, image = cap.read()
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

        area=[]
        for idx in range(output.shape[0]) :
            area.append(output[idx,5]*output[idx,4])
        # area.sort(are)
        for i in range(len(area)):
            for j in range(i,len(area),1):
                if (area[i]<area[j]):
                    output[i],output[j]=output[j],output[i]
                    area[i],area[j]=area[j],area[i]
                
        
        for idx in range(output.shape[0]):
            plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
            # print(output[idx,2:6])
            x_center=int(output[idx,2])
            y_center=int(output[idx,3])
            half_w=int(output[idx,4]/2)
            half_h=int(output[idx,5]/2)
            cv2.rectangle(nimg,(int(output[idx,2]-output[idx,4]/2),int(output[idx,3]-output[idx,5]/2)),(int(output[idx,4]/2+output[idx,2]),int(output[idx,5]/2+output[idx,3])),color=(0,0,0),thickness=2)
            Draw_circle(output[idx],len(output[idx]),nimg)
            if (count%100==0):
                flag+=1
                print("zzzzzzzzzzzz")
                # chaneg_image=cv2.crop (nimg,(int(output[idx,2]-output[idx,4]/2),int(output[idx,3]-output[idx,5]/2)),(int(output[idx,4]/2+output[idx,2]),int(output[idx,5]/2+output[idx,3])))
                # change_image=nimg[int(output[idx,2]-output[idx,4]/2):int(output[idx,2]+output[idx,4]/2),int(output[idx,3]+output[idx,5]/2):int(output[idx,3]+output[idx,5]/2)]
                # change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]
                # if (time.time()-begin_time<10):
                #     cv2.imwrite(f"data_face/00{person_index}.jpg",change_image)
                #     new_task=threading.Thread(target=task2)
                #     new_task.start()
                #     now_time=time.time()
                # else :
                #     rslt=face_recognitio.recognition(change_image)
                #     if len(rslt)!=0:
                #         cv2.imwrite(f"data_face/00{person_index}.jpg",change_image) 
                #         print(f"person_index{person_index}:"+rslt[0])
                person_index+=1
                print(time.time()-begin_time)
            if (now_time-t>1):
                # print(area)
                t=now_time
        fps=1/(time.time()-last_time)
        last_time=time.time()    
    # 打开摄像头
        cv2.putText(nimg,"fps"+str(fps),(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,0),thickness=1)
        cv2.imshow("ning",nimg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        count+=1
        # print(count)    
        mutex1.release()
    cap.release()
    cv2.destroyAllWindows()
def task2():
    global flag,person_index_count,face_recognitio,now_flag,people_havebeen_recorder
    print(now_flag)
    mutex2.acquire()

    begin=time.time()
    names_list =os.listdir("data_face")
    names_list.reverse()
    for index,data in enumerate(names_list):
        if index<person_index_count : 
            continue
        img = cv2.imdecode(np.fromfile(os.path.join("data_face",data), dtype=np.uint8), -1)
        result = face_recognitio.register(img, user_name=f'person{person_index_count}')
        person_index_count+=1
        print(result)
        # results = face_recognitio.recognition(img)
        now_flag=flag
    people_havebeen_recorder=1
    mutex2.release()
def sum_func():
    now_task1=threading.Thread(target=task1)
    now_task2=threading.Thread(target=task2)
    now_task1.start()
    # now_task2.start()
if __name__=="__main__":
    time.sleep(4)
    sum_func()
    # task2()

    
    
 
 
 