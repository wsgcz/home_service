import matplotlib.pyplot as plt
import torch
import cv2
from torchvision import transforms
import numpy as np
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts,plot_one_box
from face_detect import FaceRecognition
import time
# 加载模型
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
weigths = torch.load('yolov7-w6-pose.pt')
model = weigths['model']
model = model.half().to(device)
_ = model.eval()

def Draw_circle(posetion_arr:list,length:int,img):
    for i in range(7,length,3):
        pos_x=int(posetion_arr[i])
        pos_y=int(posetion_arr[i+1])
        print(pos_x)
        # pos_y=posetion_arr[i]
        # cv2.circle(img,(int(pos_x),int(pos_y)),2,thickness=2)
        cv2.putText(img,"index"+str(i),(pos_x+10,pos_y+10),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255),thickness=1)
# 读取摄像头画面
last_time=time.time()
if __name__=="__main__":
    url=0
    now_time=time.time()
    cap = cv2.VideoCapture(url)
    t=time.time()
    count=0
    while (cap.isOpened()):
        ret, image = cap.read()
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
                
        
        for idx in range(output.shape[0]):
            plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
            # print(output[idx,2:6])
            cv2.rectangle(nimg,(int(output[idx,2]-output[idx,4]/2),int(output[idx,3]-output[idx,5]/2)),(int(output[idx,4]/2+output[idx,2]),int(output[idx,5]/2+output[idx,3])),color=(0,0,0),thickness=2)
            # Draw_circle(output[idx],len(output[idx]),nimg)
            now_time=time.time()
            if (now_time-t>1):
                # print(area)
                t=now_time
        fps=1/(now_time-last_time)
        last_time=now_time    
    # 打开摄像头
        cv2.putText(nimg,"fps"+str(fps),(50,50),cv2.FONT_HERSHEY_COMPLEX,3,(0,0,0),thickness=3)
        cv2.imshow("ning",nimg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        count+=1
        if (count==20):
            cv2.imwrite("data_face/001.jpg",nimg)

    cap.release()
    cv2.destroyAllWindows()
 
 