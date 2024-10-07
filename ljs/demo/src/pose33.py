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
# from general_service_2022.msg import Goals_name

sys.path.insert(0,'/home/cch/general_service/src/pose_estimate')
sys.path.insert(1,'/home/cch/general_service/src/pose_estimate/scripts')
# from utils.datasets import letterbox
# from utils.general import non_max_suppression_kpt
# from utils.plots import output_to_keypoint, plot_skeleton_kpts,plot_one_box


class GlobalVar:
    eps = 1e-2
    P_x = 479 
    P_y = 269 
    f_x = 540.68603515625
    f_y = 540.68603515625
    machine_odom_now_x = 0
    machine_odom_now_y = 0
    machine_theta = 0
    frame = 0
    last_person = 0
    reaction_flag = 0
    start_work = False
    '''
    reaction_flag:
    0:初始状态
    1:识别人体骨架，寻找朝向位置坐标，探索房间
    2:注册人脸
    3:匹配+识别
    4:匹配人脸
    1，2对应start_work == False, 3,4对应start_work == True
    '''
    rospy.init_node("pose3")
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

    # 将待传输数据放入队列中
    def put_data_into_quene(data):
        GlobalVar.queue_mutex.acquire()
        GlobalVar.pub_queue.put(data)
        rospy.loginfo("put into queue")
        GlobalVar.queue_mutex.release()

    def get_odom_message(odom:Odometry):
        GlobalVar.machine_odom_now_x= odom.pose.pose.position.x
        GlobalVar.machine_odom_now_y=odom.pose.pose.position.y 

        odom_ox=odom.pose.pose.orientation.x
        odom_oy=odom.pose.pose.orientation.y
        odom_oz=odom.pose.pose.orientation.z
        odom_ow=odom.pose.pose.orientation.w
        _,_,GlobalVar.mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow])

class FaceRecognition:
    # def __init__(self, gpu_id=0, face_db='/home/cch/general_service/src/pose_estimate/face_db', threshold=1.24, det_thresh=0.50, det_size=(640, 640)):
    #     """
    #     人脸识别工具类
    #     :param gpu_id: 正数为GPU的ID，负数为使用CPU
    #     :param face_db: 人脸库文件夹
    #     :param threshold: 人脸识别阈值
    #     :param det_thresh: 检测阈值
    #     :param det_size: 检测模型图片大小
    #     """
    #     self.gpu_id = gpu_id
    #     self.face_db = face_db
    #     self.threshold = threshold
    #     self.det_thresh = det_thresh
    #     self.det_size = det_size
    #     GlobalVar.last_person = len(os.listdir(face_db))
    #     # 加载人脸识别模型，当allowed_modules=['detection', 'recognition']时，只单纯检测和识别
    #     self.model = insightface.app.FaceAnalysis(
    #                                               allowed_modules=['detection', 'recognition'],
    #                                               providers=['CUDAExecutionProvider'])
    #     self.model.prepare(ctx_id=self.gpu_id, det_thresh=self.det_thresh, det_size=self.det_size)
    #     # 人脸库的人脸特征
    #     self.faces_embedding = list()
    #     # 加载人脸库中的人脸
    #     self.load_faces()

    # # 加载人脸库中的人脸
    # def load_faces(self):
    #     if not os.path.exists(self.face_db):
    #         os.makedirs(self.face_db)
    #     for root, dirs, files in os.walk(self.face_db):
    #         for file in files:
    #             input_image = cv2.imdecode(np.fromfile(os.path.join(root, file), dtype=np.uint8), 1)
    #             user_name = file.split(".")[0]
    #             face = self.model.get(input_image)[0]
    #             embedding = np.array(face.embedding).reshape((1, -1))
    #             embedding = preprocessing.normalize(embedding)
    #             self.faces_embedding.append({
    #                 "user_name": user_name,
    #                 "feature": embedding
    #             })

    # # 人脸识别
    # # 输入：图片
    # # 输出：facedb中匹配到的人脸的特征（字典）的列表
    # def recognition(self, image):
    #     faces = self.model.get(image)
    #     results = list()
    #     for face in faces:
    #         # 开始人脸识别
    #         embedding = np.array(face.embedding).reshape((1, -1))
    #         embedding = preprocessing.normalize(embedding)
    #         user_name = "unknown"
    #         for com_face in self.faces_embedding:
    #             r = self.feature_compare(embedding, com_face["feature"], self.threshold)
    #             if r:
    #                 user_name = com_face["user_name"]
    #                 print('Found existing matched face!',user_name)
    #                 break
    #         results.append(user_name)
    #     return results

    # @staticmethod
    # def feature_compare(feature1, feature2, threshold):
    #     diff = np.subtract(feature1, feature2)
    #     dist = np.sum(np.square(diff), 1)
    #     if dist < threshold:
    #         return True
    #     else:
    #         return False
    
    # # 注册人脸
    # # 输入：图片+名字
    # # 输出：成功与否
    # def register(self, image, user_name):
    #     faces = self.model.get(image)
    #     print(len(faces))
    #     if (len(faces)>1):
    #         return "有多个人"
    #     if len(faces)<=0:
    #         return '图片检测不到人脸'

    #     # 判断人脸是否存在
    #     embedding = np.array(faces[0].embedding).reshape((1, -1))
    #     embedding = preprocessing.normalize(embedding)
    #     is_exits = False
    #     for com_face in self.faces_embedding:
    #         r = self.feature_compare(embedding, com_face["feature"], self.threshold)
    #         if r:
    #             is_exits = True
    #     if is_exits:
    #         return '该用户已存在'
    #     # 符合注册条件保存图片，同时把特征添加到人脸特征库中
    #     cv2.imencode('.png', image)[1].tofile(os.path.join(self.face_db, '%s.png' % GlobalVar.last_person))
    #     self.faces_embedding.append({
    #         "user_name": user_name,
    #         "feature": embedding
    #     })
    #     return "success"

    # # 检测人脸
    # # 输入：图片
    # # 输出：含人脸特征（字典）的列表
    # def detect(self, image):
    #     faces = self.model.get(image)
    #     results = list()
    #     for face in faces:
    #         attribte = dict()
    #         # 获取人脸属性
    #         attribte["bbox"] = np.array(face.bbox).astype(np.int32).tolist()
    #         attribte["kps"] = np.array(face.kps).astype(np.int32).tolist()
    #         gender = '男'
    #         if face.gender == 0:
    #             gender = '女'
    #         attribte["gender"] = gender
    #         # 开始人脸识别
    #         embedding = np.array(face.embedding).reshape((1, -1))
    #         embedding = preprocessing.normalize(embedding)
    #         attribte["embedding"] = embedding
    #         results.append(attribte)
    #     return results
    
    # # 从facedb中检索人脸信息
    # # 输入： 文件名
    # # 输出： 特征字典
    # def retrieve(self, addr):
    #     GlobalVar.face_mutex.acquire()
    #     face = cv2.imdecode(np.fromfile(addr,dtype=np.uint8),-1)
    #     store_name = addr.split('/')[-1].split('.')[0] 
    #     result = self.recognition(face)
    #     try:
    #         result=result[0]
    #     except BaseException as e:
    #         result="unknown"
    #     GlobalVar.face_mutex.release()
    #     return result

    # def face_rec_threading(self, name): 
    #     addr = f"/home/cch/general_service/src/pose_estimate/data_face/{name}.jpg"
    #     face_feature = self.retrieve(addr)
    #     rospy.loginfo("after face recognition, this is result %s",face_feature)
    #     pose_feature=MoveBaseGoal()
    #     pose_feature.target_pose.header.frame_id="None"
    #     if face_feature != 'unknown':
    #         pub_msg = (pose_feature, face_feature, f"{name}{GlobalVar.last_person}.img")
    #         pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
    #         pub_thread.start()    
    #     else:
    #         rospy.loginfo("No matched face in facedb!")        

    
    # class Face_register(threading.Thread):
    #     def __init__(self,target,img,name):
    #         super().__init__(target=target)
    #         self.target = target
    #         self.img = img
    #         self.result=None
    #         self.name=name
    #     def run(self) -> None:
    #         GlobalVar.face_mutex.acquire()
    #         self.result = self.target(self.img,self.name)
    #         GlobalVar.face_mutex.release()
    #     def return_result(self):
    #         return self.result if self.result == 'success' else None   
    def __init__(self, gpu_id=0, face_db='', threshold=1.24, det_thresh=0.50, det_size=(640, 640)):
        """
        人脸识别工具类
        :param gpu_id: 正数为GPU的ID，负数为使用CPU
        :param face_db: 人脸库文件夹
        :param threshold: 人脸识别阈值
        :param det_thresh: 检测阈值
        :param det_size: 检测模型图片大小
        """
        self.gpu_id = gpu_id
        self.face_db = face_db
        self.threshold = threshold
        self.det_thresh = det_thresh
        self.det_size = det_size
        self.model = insightface.app.FaceAnalysis(allowed_modules=['detection', 'recognition'], providers=['CUDAExecutionProvider'])
        self.model.prepare(ctx_id=self.gpu_id, det_thresh=self.det_thresh, det_size=self.det_size)
        # 人脸信息，列表，每个元素为字典，每个字典包含两个键值对
        self.faces_embedding = list()
        # 加载人脸库中人脸
        self.load_faces()

    # 加载人脸库中的人脸至self.faces_embedding
    # 数据库中的人脸应当保证命名为name.jpg
    def load_faces(self):
        if not os.path.exists(self.face_db):
            os.makedirs(self.face_db)
        for root, dirs, files in os.walk(self.face_db):
            for file in files:
                input_image = cv2.imread(os.path.join(root, file))
                user_name = file.split(".")[0]
                faces = self.model.get(input_image)
                # 仅支持单个人脸
                if faces:
                    face = faces[0]
                    embedding = np.array(face.embedding).reshape((1, -1))
                    embedding = preprocessing.normalize(embedding)
                    self.faces_embedding.append({
                        "user_name": user_name,
                        "feature": embedding
                    })
                    print(f"已加载人脸: {user_name}")
                else:
                    print(f"在 {file} 中未检测到人脸")

    # 人脸识别
    # 输入：图片
    # 输出：facedb中匹配到的人脸的名字的列表
    def recognition(self, image):
        faces = self.model.get(image)
        results = list()
        for face in faces:
            embedding = np.array(face.embedding).reshape((1, -1))
            embedding = preprocessing.normalize(embedding)
            user_name = "unknown"
            for com_face in self.faces_embedding:
                r = self.feature_compare(embedding, com_face["feature"], self.threshold)
                if r:
                    user_name = com_face["user_name"]
                    break
            results.append(user_name)
        return results

    # 特征比较
    @staticmethod
    def feature_compare(feature1, feature2, threshold):
        diff = np.subtract(feature1, feature2)
        dist = np.sum(np.square(diff), 1)
        return dist < threshold

    # 注册人脸
    # 输入：图片+名字
    # 输出：成功与否
    def register(self, image, user_name):
        faces = self.model.get(image)
        if len(faces) > 1:
            return f"有{len(faces)}张人脸，注册失败"
        if len(faces) <= 0:
            return '图片检测不到人脸，注册失败'

        embedding = np.array(faces[0].embedding).reshape((1, -1))
        embedding = preprocessing.normalize(embedding)
        is_exits = False
        for com_face in self.faces_embedding:
            r = self.feature_compare(embedding, com_face["feature"], self.threshold)
            if r:
                is_exits = True
        if is_exits:
            return '该用户已存在'
        
        cv2.imwrite(os.path.join(self.face_db, f'{user_name}.jpg'), image)
        self.faces_embedding.append({
            "user_name": user_name,
            "feature": embedding
        })
        return "成功添加人脸"

class Yolov8:
    # def __init__(self,model_path='/home/cch/general_service/src/pose_estimate/scripts/yolov7-w6-pose.pt'):
    #     self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    #     weights = torch.load(model_path)
    #     self.model = weights['model']
    #     self.model = self.model.half().to(self.device)
    #     self.model.eval()

    # # 对图像进行检测，并返回各个裁剪过后的人体图像，高宽等
    # # 输入：rgb图片
    # # 输出：包含n个（裁剪后图像BGR，宽，高，中心x坐标，y坐标，索引）的列表，n为识别到人的个数
    def __init__(self):
        pass

    # def detect(self,image):
    #     image = letterbox(image, 960, stride=64, auto=True)[0]
    #     image = transforms.ToTensor()(image)
    #     image = torch.tensor(np.array([image.numpy()]))
    #     image = image.to(self.device)
    #     image = image.half()
    #     with torch.no_grad():
    #         output, _ = self.model(image)
    #         output = non_max_suppression_kpt(output, 0.25, 0.65, nc=self.model.yaml['nc'], nkpt=self.model.yaml['nkpt'], kpt_label=True)
    #         output = output_to_keypoint(output)
    #     # 原图像nimg    
    #     nimg = image[0].permute(1, 2, 0) * 255
    #     nimg = nimg.cpu().numpy().astype(np.uint8)
    #     nimg = cv2.cvtColor(nimg, cv2.COLOR_RGB2BGR)
    #     area=[]
    #     for i in range(output.shape[0]) :
    #         area.append((output[i,5]*output[i,4],i))
    #     for i in range(output.shape[0]):
    #         for j in range(i+1,output.shape[0],1):
    #             if (area[i]<area[j]):
    #                 area[i],area[j]=area[j],area[i]
    #                 # output[i],output[j]=output[j],output[i]
    #     length_area=min(len(area),4)
    #     idx_list=[]
    #     for i in range(length_area):
    #         idx_list.append(area[i][1])
    #     idx_count=0
    #     results = list()
    #     for idx in idx_list:
    #         # idx=area[idx][1]
    #         # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
    #         idx_count+=1
    #         x_center=int(output[idx][2])
    #         y_center=int(output[idx][3])
    #         half_w=int(output[idx][4]/2)
    #         half_h=int(output[idx][5]/2)
    #         cv2.rectangle(nimg,(int(output[idx][2]-output[idx][4]/2),int(output[idx][3]-output[idx][5]/2)),(int(output[idx][4]/2+output[idx][2]),int(output[idx][5]/2+output[idx][3])),color=(0,0,0),thickness=2)
    #         change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]      
    #         results.append((change_image,half_w*2,half_h*2,x_center,y_center,idx))
    #     return results

    def start_single_predict(self, yolo_model, img_path):
        model = YOLO(yolo_model)
        results = model(img_path)
        confidence_max = -1
        class_name_max = ""
        # 获取类别名称字典
        for r in results:
            # 获取类别名称字典
            names = r.names
            # 遍历每个检测到的对象
            for box in r.boxes:
                # 获取类别索引
                class_id = int(box.cls[0])
                # 获取类别名称
                class_name = names[class_id]
                # 获取置信度
                confidence = float(box.conf[0])
                if confidence > confidence_max:
                    confidence_max = confidence
                    class_name_max = class_name
            # 如果没有检测到任何对象
            if len(r.boxes) == 0:
                continue
        return confidence_max, class_name_max
    
    # def start_several_predict(self, yolo_model, img_folder_path):
    #     model = YOLO(yolo_model)  
    #     # 指定图片文件夹路径
    #     image_folder = img_folder_path
    #     # 获取文件夹中所有图片文件
    #     image_files = [f for f in os.listdir(image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]
    #     # 遍历所有图片文件
    #     for image_file in image_files:
    #         image_path = os.path.join(image_folder, image_file)
    #         print(f"正在处理图片: {image_file}")
    #         # 开始预测
    #         results = model(image_path)
    #         for r in results:
    #             # 获取类别名称字典
    #             names = r.names
    #             print(names)
    #             # 可视化
    #             r.show()
    #             # 遍历每个检测到的对象
    #             for box in r.boxes:
    #                 # 获取类别索引
    #                 class_id = int(box.cls[0])
    #                 # 获取类别名称
    #                 class_name = names[class_id]
    #                 # 获取置信度
    #                 confidence = float(box.conf[0])
    #                 # 打印结果
    #                 print(f"检测到: {class_name}, 置信度: {confidence:.2f}")
    #             # 如果没有检测到任何对象
    #             if len(r.boxes) == 0:
    #                 print("未检测到任何对象")
    #         print(f"完成处理图片: {image_file}")
    #         print("----------------------------")
    #         time.sleep(3)  # 每次完成后暂停3秒
    #     print("所有图片处理完成")

    # def main_yolo():
    #     # 修改参数
    #     # 训练模型的数据为'A_my_data.yaml'，轮数为100，图片大小为640，设备为本地的GPU显卡，关闭多线程的加载，图像加载的批次大小为4，开启图片缓存
    #     # 如果gpu占用少，可以把batch由4调到16
    #     data='src/RubbishRecognition/scripts/A_my_data.yaml'
    #     epochs=10
    #     imgsz=640
    #     batch=16
    #     # start_train = StartTrain(data=data, epochs=epochs, imgsz=imgsz, batch=batch)
    #     # start_val = StartVal(data=data, imgsz=imgsz, batch=batch)
    #     while(True):
    #         print("请输入数字选择功能：")
    #         print("1. 训练模型")
    #         print("2. 验证模型")
    #         print("3. 单张图片预测")
    #         print("4. 多张图片预测")
    #         print("5. 退出")
    #         choice = input("请输入数字：")
    #         # if choice == "1":
    #         #     start_train.start_train()
    #         # elif choice == "2":
    #         #     yolo_model = input("请输入模型路径：")
    #         #     start_val.start_val(yolo_model)
    #         if choice == "3":
    #             yolo_model = input("请输入模型路径：")
    #             img_path = input("请输入图片路径：")
    #             start_predict.start_single_predict(yolo_model, img_path)
    #         elif choice == "4":
    #             yolo_model = input("请输入模型路径：")
    #             img_folder_path = input("请输入图片文件夹路径：")
    #             start_predict.start_several_predict(yolo_model, img_folder_path)
    #         elif choice == "5":
    #             break
    #         else:
    #             print("无效选择，请重新输入")

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(static_image_mode=True)

class Mediapipe:
    def __init__(self):
        self.mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=1,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)
    
    class Mediapipe_thread(threading.Thread):
        def __init__(self,target,image,width,height,mid_x,mid_y,path,depth):
                self.target = target
                self.result=None
                self.image=image
                self.path=path
                self.width=width
                self.height=height
                self.mid_x=mid_x
                self.mid_y=mid_y
                self.depth=depth
                super().__init__(target=target,args=(self.image,self.width,self.height,self.mid_x,self.mid_y,self.path,self.depth))

        def run(self):
            self.result=self.target(self.image,self.width,self.height,self.mid_x,self.mid_y,self.path,self.depth)

        def return_results(self):
            return self.result
        

    def detect(self, image, width, height, mid_x, mid_y, name, depth):
        GlobalVar.mediapipe_mutex.acquire()
        results = self.mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        if not results.pose_world_landmarks:
            return None
        alpha = self.get_alpha(results)
        alpha = alpha/pi*180
        print("this is alpha ",alpha ,"and name is ",name)
        goal = self.convert_machine_axis_to_world(results,width,height,mid_x,mid_y,name,depth)
        GlobalVar.mediapipe_mutex.release()
        return goal
    
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(static_image_mode=True)

    # v2相对于v1顺时针小于0、逆时针大于0
    def get_angle(self, v1, v2):
        angle = np.dot(v1, v2) / (np.sqrt(np.sum(v1 * v1)) * np.sqrt(np.sum(v2 * v2)))
        angle = np.arccos(angle) / 3.14159265 * 180

        cross = v2[0] * v1[1] - v2[1] * v1[0]
        if cross < 0:
            angle = - angle
        return angle

    def get_distance(self, v1, v2):
        return np.sqrt(np.sum((v1 - v2) ** 2))

    def get_pos(self, keypoints):
        """
        0 - 鼻子
        1 - 左眼(内)
        2 - 左眼
        3 - 左眼(外)
        4 - 右眼(内)
        5 - 右眼
        6 - 右眼(外)
        7 - 左耳
        8 - 右耳
        9 - 嘴(左)
        10 - 嘴(右)
        11 - 左肩
        12 - 右肩
        13 - 左肘
        14 - 右肘
        15 - 左手腕
        16 - 右手腕
        17 - 左小指
        18 - 右小指
        19 - 左食指
        20 - 右食指
        21 - 左大拇指
        22 - 右大拇指
        23 - 左臀
        24 - 右臀
        25 - 左膝
        26 - 右膝
        27 - 左踝
        28 - 右踝
        29 - 左脚后跟
        30 - 右脚后跟
        31 - 左脚尖
        32 - 右脚尖
        """
        str_pose = "无"
        keypoints = np.array(keypoints)

        ### 角度

        # 躯干与竖直轴
        # 左
        v1 = keypoints[11] - keypoints[23]
        v2 = np.array([0, -1]); # 竖直向上的向量
        from_23_11_to_y = Mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[24]
        v2 = np.array([0, -1]); # 竖直向上的向量
        from_24_12_to_y = Mediapipe.get_angle(v1, v2)

        # 小腿之间
        v1 = keypoints[27] - keypoints[25]
        v2 = keypoints[28] - keypoints[26]
        from_25_27_to_26_28 = Mediapipe.get_angle(v1, v2)

        # 大腿之间
        v1 = keypoints[25] - keypoints[23]
        v2 = keypoints[26] - keypoints[24]
        from_23_25_to_24_26 = Mediapipe.get_angle(v1,v2)

        # 躯干与大腿
        # 左
        v1 = keypoints[11] - keypoints[23]
        v2 = keypoints[25] - keypoints[23]
        from_23_to_11_25 = Mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[24]
        v2 = keypoints[26] - keypoints[24]
        from_24_to_12_26 = Mediapipe.get_angle(v1, v2)    

        # 大腿和小腿
        # 左
        v1 = keypoints[23] - keypoints[25]
        v2 = keypoints[27] - keypoints[25]
        from_25_to_23_27 = Mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[24] - keypoints[26]
        v2 = keypoints[28] - keypoints[26]
        from_26_to_24_28 = Mediapipe.get_angle(v1, v2)

        # 躯干与小臂
        # 左
        v1 = keypoints[23] - keypoints[11]
        v2 = keypoints[15] - keypoints[13]
        from_11_23_to_13_15 = Mediapipe.get_angle(v1,v2)
        # 右
        v1 = keypoints[24] - keypoints[12]
        v2 = keypoints[16] - keypoints[14]
        from_12_24_to_14_16 = Mediapipe.get_angle(v1,v2)

        # 手肘
        # 左
        v1 = keypoints[11] - keypoints[13]
        v2 = keypoints[15] - keypoints[13]
        from_13_to_11_15 = Mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[14]
        v2 = keypoints[16] - keypoints[14]
        from_14_to_12_16 = Mediapipe.get_angle(v1, v2)
        
        ### 距离
        
        # 肩膀之间
        distance_11_12 = Mediapipe.get_distance(keypoints[11],keypoints[12])

        # 食指和嘴
        # 左
        distance_19_9 = Mediapipe.get_distance(keypoints[19],keypoints[9])
        # 右
        distance_20_10 = Mediapipe.get_distance(keypoints[20],keypoints[10])

        # 食指和耳
        # 左
        distance_19_7 = Mediapipe.get_distance(keypoints[19],keypoints[7])
        # 右
        distance_20_8 = Mediapipe.get_distance(keypoints[20],keypoints[8])

        ### 姿态识别

        # 蹲起
        if 20<abs(from_23_11_to_y)<60 and 20<abs(from_24_12_to_y)<60:
            str_pose = "蹲起_1"
            if 30<abs(from_25_to_23_27)<150 and 30<abs(from_26_to_24_28)<150:
                str_pose = "蹲起"
                return str_pose
            return str_pose

        # 站立、走、躺、俯卧撑、吸烟、打电话
        if abs(from_23_11_to_y)<45 and abs(from_24_12_to_y)<45:
            str_pose = "站立"
            if distance_19_9<0.75*distance_11_12 or distance_20_10<0.75*distance_11_12 or distance_19_7<0.75*distance_11_12 or distance_20_8<0.75*distance_11_12:
                distances = [distance_19_9,distance_20_10,distance_19_7,distance_20_8]
                min_index, min_distance = min(enumerate(distances), key=lambda x: x[1])
                if min_index==0 or min_index==1:
                    str_pose = "吸烟"
                    return str_pose
                else:
                    str_pose = "打电话"
                    return str_pose
            if abs(abs(from_23_to_11_25)-90)<45 or abs(abs(from_24_to_12_26)-90)<45:
                str_pose = "坐"
                return str_pose
            if abs(from_25_27_to_26_28)>20 or abs(from_23_25_to_24_26)>20:
                str_pose = "行走"
                return str_pose
            if abs(from_11_23_to_13_15)>100:
                str_pose = "举左手"
                if abs(from_12_24_to_14_16)>100:
                    str_pose = "举双手"
                return str_pose
            if abs(from_12_24_to_14_16)>100:
                str_pose = "举右手"
                if abs(from_11_23_to_13_15)>100:
                    str_pose = "举双手"
                return str_pose
            return str_pose
        else :
            str_pose = "躺"
            if 45<abs(from_11_23_to_13_15)<135 and 45<abs(from_12_24_to_14_16)<135:
                str_pose = "俯卧撑"
                return str_pose
            return str_pose
        
        return str_pose

    def process_frame(self, img):
        start_time = time.time()
        h, w = img.shape[0], img.shape[1]               # 高和宽
        # 调整字体
        tl = round(0.005 * (img.shape[0] + img.shape[1]) / 2) + 1
        tf = max(tl-1, 1)
        # BRG-->RGB
        img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # 将RGB图像输入模型，获取 关键点 预测结果
        results = Mediapipe.pose.process(img_RGB)
        keypoints = ['' for i in range(33)]
        if results.pose_landmarks:
            Mediapipe.mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            for i in range(33):
                cx = int(results.pose_landmarks.landmark[i].x * w)
                cy = int(results.pose_landmarks.landmark[i].y * h)
                keypoints[i] = (cx, cy)                                 # 得到最终的33个关键点
        else:
            print("NO PERSON")
            struction = "NO PERSON"
            img = cv2.putText(img, struction, (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (255, 255, 0),6)
            return img, struction
        end_time = time.time()
        process_time = end_time - start_time            # 图片关键点预测时间
        fps = 1 / process_time                          # 帧率
        colors = [[random.randint(0,255) for _ in range(3)] for _ in range(33)]
        radius = [random.randint(8,15) for _ in range(33)]
        for i in range(33):
            cx, cy = keypoints[i]
            #if i in range(33):
            img = cv2.circle(img, (cx, cy), radius[i], colors[i], -1)
        str_pose = self.get_pos(keypoints)            #获取姿态
        # cv2.putText(img, "POSE-{}".format(str_pose), (12, 100), cv2.FONT_HERSHEY_TRIPLEX,
        #             tl / 3, (255, 0, 0), thickness=tf)
        cv2.putText(img, "FPS-{}".format(str(int(fps))), (12, 100), cv2.FONT_HERSHEY_SIMPLEX,tl/3, (255, 255, 0),thickness=tf)
        return img, str_pose
    
    def main_mediapipe(self, image):
        # img_folder_path = "/home/shanhe/mediapipe_pose_predict/all/all_static"

        # # 指定图片文件夹路径
        # image_folder = img_folder_path

        # # 获取文件夹中所有图片文件
        # image_files = [f for f in os.listdir(image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]

        # 遍历所有图片文件
        # for image_file in image_files:
            
        #     image_path = os.path.join(image_folder, image_file)
        #     print(f"正在处理图片: {image_file}")

        #     # 开始预测
        #     img = cv2.imread(image_path)
        #     image, pose_str = self.process_frame(img)

        #     if pose_str == "NO PERSON":
        #         print("未检测到人")
        #         output_filename = f"{image_file.split('.')[0]}_no_person.jpg"
        #         output_path = os.path.join(image_folder, output_filename)
        #         cv2.imwrite(output_path, image)
        #         print(f"完成处理图片: {image_file}")
        #         print("----------------------------")
        #         continue

        #     # 生成输出文件名
        #     output_filename = f"{image_file.split('.')[0]}_{pose_str}.jpg"
        #     output_path = os.path.join(image_folder, output_filename)

        #     # 保存处理后的图像
        #     cv2.imwrite(output_path, image)

        #     print(f"检测到动作：{pose_str}")
        #     print(f"完成处理图片: {image_file}")
        #     print("----------------------------")
        #     # time.sleep(3)  # 每次完成后暂停3秒
        # print("所有图片处理完成")
        img, pose_str = self.process_frame(image)
        return pose_str

    
    
    
    # 返回人的朝向角
    # 输入：mediapipe返回的results
    # 输出：浮点数，表示弧度
    # @staticmethod
    # def get_alpha(results):
    #     z_left = results.pose_world_landmarks.landmark[11].z
    #     z_right = results.pose_world_landmarks.landmark[12].z
    #     x_left = results.pose_world_landmarks.landmark[11].x
    #     x_right = results.pose_world_landmarks.landmark[12].x
    #     z_distance=(z_right-z_left)
    #     x_distance=(x_right-x_left)
    #     alpha = pi + atan2(z_distance, x_distance)
    #     return alpha
    
    # # 返回物体在机器人坐标系下的真实距离
    # # 输入： 物体在图片中的像素位置u_img(x),v_img(y),d:深度，单位mm
    # # 输出： 用米做单位的坐标，机器人坐标系
    # @staticmethod
    # def real_pose(u_img, v_img, d):
    #     other_angle=(v_img-270)*43/540
    #     this_angle=0.46*180/pi
    #     now_angle=abs(this_angle+other_angle)*pi/180
    #     now_angle1=abs(other_angle)*pi/180
    #     Z = d/sqrt(1+(u_img-GlobalVar.P_x)**2/GlobalVar.f_x**2+(v_img-GlobalVar.P_y)**2/GlobalVar.f_y**2)
    #     X = (u_img-GlobalVar.P_x)*Z/GlobalVar.f_x
    #     Y = (v_img-GlobalVar.P_y)*Z/GlobalVar.f_y
    #     y = (Y/1000)
    #     zzz = d/1000*cos(now_angle)+0.24-y*sin(now_angle1)
    #     # yyy=1.62-d/1000*math.sin(0.4)+y*math.cos(0.4)
    #     return X/1000+0.015, y, zzz
    
    # @staticmethod
    # def convert_machine_axis_to_world(results,width,height,x_mid,y_mid,name,depth):
    #     if results.pose_world_landmarks == None:
    #         return 
    #     # 计算节点在真实图片中的像素位置，不过landmark的x的单位肯定是width吗，有待商榷
    #     right_hip_x = int(results.pose_landmarks.landmark[24].x * width + x_mid - 0.5 * width)
    #     right_hip_y = int(results.pose_landmarks.landmark[24].y * height + y_mid - 0.5 * height)
    #     left_hip_x = int(results.pose_landmarks.landmark[23].x * width + x_mid - 0.5 * width)
    #     left_hip_y = int(results.pose_landmarks.landmark[23].y * height + y_mid - 0.5 * height)
    #     left_shoulder_x = int(results.pose_landmarks.landmark[11].x * width + x_mid - 0.5 * width)
    #     left_shoulder_y = int(results.pose_landmarks.landmark[11].y * height + y_mid - 0.5 * height)
    #     right_shoulder_x = int(results.pose_landmarks.landmark[12].x * width + x_mid - 0.5 * width)
    #     right_shoulder_y = int(results.pose_landmarks.landmark[12].y * height + y_mid - 0.5 * height)
    #     if right_hip_x >= 960: right_hip_x = 959
    #     if right_hip_y >= 540: right_hip_y = 539
    #     if left_hip_x >= 960: left_hip_x = 959
    #     if left_hip_y >= 540: left_hip_y = 539
    #     depth_right_hip = depth[right_hip_y][right_hip_x]
    #     depth_left_hip = depth[left_hip_y][left_hip_x]
    #     depth_left_shoulder = depth[left_shoulder_y][left_shoulder_x]
    #     depth_right_shoulder = depth[right_shoulder_y][right_shoulder_x]
    #     depths = [depth_left_hip, depth_left_shoulder, depth_right_hip, depth_right_shoulder]
    #     length = len(depths)
    #     for d in depths:
    #         if d < GlobalVar.eps:
    #             length -= 1
    #     average_depth = sum(depths) / length
    #     real_person = Mediapipe.real_pose(x_mid, y_mid, average_depth)
    #     rospy.loginfo(f"this is the relative pos of person: {real_person}")
    #     if real_person[2] < GlobalVar.eps:
    #         return
    #     # 接下来是坐标转换
    #     alpha = Mediapipe.get_alpha(results)
    #     robot_states = GlobalVar.get_map_pose_theta()
    #     rospy.loginfo(f"this is robot position:{robot_states}")
    #     rpy = GlobalVar.ori_to_rpy(0.0, 0.0, robot_states[2], robot_states[3])
    #     # 人前1.5m位置
    #     z_second = real_person[2] - 1.5 * cos(alpha)
    #     x_second = real_person[0] + 1.5 * sin(alpha)
    #     # if alpha == 0.5 * pi or alpha == 1.5 * pi:
    #     #     z_second -= 0.2
    #     if alpha >= 0.8 * pi and alpha <= 1.2 * pi:
    #         z_second += 0.1

    #     # y坐标，即高度没用,地图x方向向右，y方向向前
    #     # 最后x应该向前，y向左
    #     x1, y1 = GlobalVar.get_map_pose(x_second, z_second)
    #     rospy.loginfo(f"ultimate pos in the coordinate:{x1},{y1}")
    #     yaw = alpha - 2*pi
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.pose.position.x = y1
    #     goal.target_pose.pose.position.y = -x1
    #     xyzw = GlobalVar.rpy2quaternion(0.0, 0.0, yaw)
    #     goal.target_pose.pose.orientation.x = 0.0
    #     goal.target_pose.pose.orientation.y = 0.0
    #     goal.target_pose.pose.orientation.z = xyzw[2]
    #     goal.target_pose.pose.orientation.w = xyzw[3]
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     return goal
        
image,depth = None, None
# def image_callback(image_rgb,image_depth):
#     global image, depth
#     #rospy.loginfo(f"Start to detect!,flag = {GlobalVar.reaction_flag}")
#     image = GlobalVar.bridge.imgmsg_to_cv2(
#             image_rgb, desired_encoding='passthrough')
#     depth = GlobalVar.bridge.imgmsg_to_cv2(
#             image_depth, desired_encoding='passthrough')
#     if not GlobalVar.reaction_flag or GlobalVar.reaction_flag in (1,2) and GlobalVar.start_work \
#         or GlobalVar.reaction_flag == 4 and not GlobalVar.start_work:
#         return
#     if GlobalVar.frame == 0: # 第一帧图片
#         GlobalVar.cb_mutex.acquire()
#         human_detect_result = yolo.detect(image)
#         for person in human_detect_result:
#             store_path=f"/home/cch/general_service/src/pose_estimate/data_face/{GlobalVar.last_person}.jpg"
#             cv2.imwrite(store_path,image)
#             face_feature = face.retrieve(store_path)
#             if GlobalVar.reaction_flag == 1:
#                 rospy.loginfo(f"Now the task is 1")
#                 thread = mediapipe.Mediapipe_thread(mediapipe.detect,*person[:5],store_path,depth)
#                 thread.start()
#                 thread.join()
#                 pose_feature = thread.return_results()
#                 rospy.loginfo(pose_feature)
#                 if pose_feature:
#                     pub_msg = (pose_feature, face_feature, f"{GlobalVar.last_person}{GlobalVar.last_person}.img")
#                     pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
#                     pub_thread.start()
#             elif GlobalVar.reaction_flag == 2:  
#                 rospy.loginfo(f"Now the task is 2")
#                 thread = face.Face_register(target=face.register,img=image,name=GlobalVar.last_person)
#                 thread.start()
#                 thread.join()
#                 result = thread.return_result()
#                 name_record=f"{person[-1]}{GlobalVar.last_person}"
#                 if face_feature==None:
#                     name_record="None"
#                 pub_msg = (result, name_record)
#                 rospy.loginfo(f"face register result: {result}, name_record: {name_record}")
#                 pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
#                 pub_thread.start()
#             elif GlobalVar.reaction_flag == 3:# 坏了，3和1一模一样
#                 thread = mediapipe.Mediapipe_thread(mediapipe.detect,*person[:5],store_path,depth)
#                 thread.start()
#                 face_feature = face.retrieve(store_path)
#                 thread.join()
#                 pose_feature = thread.return_results()
#                 if pose_feature:
#                     pub_msg = (pose_feature, face_feature, f"{GlobalVar.last_person}.img")
#                     pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
#                     pub_thread.start()
#             else:
#                 rospy.loginfo(f"Now the task is 4")
#                 thread = threading.Thread(target=face.face_rec_threading, args=(GlobalVar.last_person,))
#                 thread.start()
#             GlobalVar.last_person += 1
#             # 稍微修改，为了调试
#             GlobalVar.frame += 1
#         GlobalVar.cb_mutex.release()
#     # 如果不是第一帧图片就没必要再检查图片了
#     else: 
#         if GlobalVar.pub_queue.empty():
#             return
#         if GlobalVar.reaction_flag == 2:
#             face_feature,name_recordeed = GlobalVar.pub_queue.get()
#             face_feature_pub.publish(name_recordeed)
#             GlobalVar.reaction_flag = 0
#             GlobalVar.frame = 0
#         else:
#             loc_target, face_feature, img_name = GlobalVar.pub_queue.get()
#             # if face_feature == 'unknown':
#             #     return
#             GlobalVar.goal_name.name = face_feature
#             GlobalVar.goal_name.goal = loc_target
#             rospy.loginfo("publish the result:")
#             rospy.loginfo(GlobalVar.goal_name)
#             #now_goal_pub.publish(GlobalVar.goal_name)
#             ac.send_goal(loc_target)
#             ac.wait_for_result()
#             rospy.loginfo("have arrived the goal")

yolo_model = ""

def image_callback(image_rgb,image_depth):
    global image, depth
    #rospy.loginfo(f"Start to detect!,flag = {GlobalVar.reaction_flag}")
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')
    depth = GlobalVar.bridge.imgmsg_to_cv2(
            image_depth, desired_encoding='passthrough')
    # if not GlobalVar.reaction_flag or GlobalVar.reaction_flag in (1,2) and GlobalVar.start_work \
    #     or GlobalVar.reaction_flag == 4 and not GlobalVar.start_work:
    #     return
    if GlobalVar.frame == 0: # 第一帧图片
        GlobalVar.cb_mutex.acquire()
        if GlobalVar.reaction_flag == 1:
            rospy.loginfo(f"Now the task is 1")
            results = face.recognition(image)
            num = 0
            for result in results:
                if result != "Unknown":
                    num += 1
            if num > 0:
                rospy.set_param("Params.people_exist",1)
            rospy.set_param("Params.finish",1)
        elif GlobalVar.reaction_flag == 2:
            rospy.loginfo(f"Now the task is 2")
            results = face.recognition(image)
            for result in results:
                if result != "Unknown":
                    rospy.set_param("Face_det.recog_msg",result)
                    break
        elif GlobalVar.reaction_flag == 3:
            rospy.loginfo(f"Now the task is 3")
            pose_str = mediapipe.main_mediapipe(image)
            rospy.set_param("Body_det.recog_msg",pose_str)
        elif GlobalVar.reaction_flag == 4:
            rospy.loginfo(f"Now the task is 4")
            confidence, result = yolov8.start_single_predict(yolo_model, image)
            if confidence > 0.5:
                rospy.set_param("arams.people_exist",1)
            rospy.set_param("Params.finish",1)
        elif GlobalVar.reaction_flag == 5:
            rospy.loginfo(f"Now the task is 5")
            confidence, result = yolov8.start_single_predict(yolo_model, image)
            if confidence > 0.5:
                rospy.set_param("Robbish_det.recog_msg",result)

        # human_detect_result = yolo.detect(image)
        # for person in human_detect_result:
        #     store_path=f"/home/cch/general_service/src/pose_estimate/data_face/{GlobalVar.last_person}.jpg"
        #     cv2.imwrite(store_path,image)
        #     face_feature = face.retrieve(store_path)
        #     if GlobalVar.reaction_flag == 1:
        #         rospy.loginfo(f"Now the task is 1")
        #         thread = mediapipe.Mediapipe_thread(mediapipe.detect,*person[:5],store_path,depth)
        #         thread.start()
        #         thread.join()
        #         pose_feature = thread.return_results()
        #         rospy.loginfo(pose_feature)
        #         if pose_feature:
        #             pub_msg = (pose_feature, face_feature, f"{GlobalVar.last_person}{GlobalVar.last_person}.img")
        #             pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
        #             pub_thread.start()
        #     elif GlobalVar.reaction_flag == 2:  
        #         rospy.loginfo(f"Now the task is 2")
        #         thread = face.Face_register(target=face.register,img=image,name=GlobalVar.last_person)
        #         thread.start()
        #         thread.join()
        #         result = thread.return_result()
        #         name_record=f"{person[-1]}{GlobalVar.last_person}"
        #         if face_feature==None:
        #             name_record="None"
        #         pub_msg = (result, name_record)
        #         rospy.loginfo(f"face register result: {result}, name_record: {name_record}")
        #         pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
        #         pub_thread.start()
        #     elif GlobalVar.reaction_flag == 3:# 坏了，3和1一模一样
        #         thread = mediapipe.Mediapipe_thread(mediapipe.detect,*person[:5],store_path,depth)
        #         thread.start()
        #         face_feature = face.retrieve(store_path)
        #         thread.join()
        #         pose_feature = thread.return_results()
        #         if pose_feature:
        #             pub_msg = (pose_feature, face_feature, f"{GlobalVar.last_person}.img")
        #             pub_thread = threading.Thread(target=GlobalVar.put_data_into_quene, args=(pub_msg,))
        #             pub_thread.start()
        #     else:
        #         rospy.loginfo(f"Now the task is 4")
        #         thread = threading.Thread(target=face.face_rec_threading, args=(GlobalVar.last_person,))
        #         thread.start()
            # GlobalVar.last_person += 1
            # 稍微修改，为了调试
        GlobalVar.frame += 1
        GlobalVar.cb_mutex.release()
    # 如果不是第一帧图片就没必要再检查图片了
    # else: 
    #     if GlobalVar.pub_queue.empty():
    #         return
    #     if GlobalVar.reaction_flag == 2:
    #         face_feature,name_recordeed = GlobalVar.pub_queue.get()
    #         face_feature_pub.publish(name_recordeed)
    #         GlobalVar.reaction_flag = 0
    #         GlobalVar.frame = 0
    #     else:
    #         loc_target, face_feature, img_name = GlobalVar.pub_queue.get()
    #         # if face_feature == 'unknown':
    #         #     return
    #         GlobalVar.goal_name.name = face_feature
    #         GlobalVar.goal_name.goal = loc_target
    #         rospy.loginfo("publish the result:")
    #         rospy.loginfo(GlobalVar.goal_name)
    #         #now_goal_pub.publish(GlobalVar.goal_name)
    #         ac.send_goal(loc_target)
    #         ac.wait_for_result()
    #         rospy.loginfo("have arrived the goal")

def show_image():
    info = 0
    while not rospy.is_shutdown():
        if depth is not None:
            if not info :
                print("GOOD")
                info = 1
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth[::2,::2], alpha=0.03), cv2.COLORMAP_JET)
            show = np.hstack((image[::2,::2,:], depth_colormap))
            cv2.imshow("image",show)
            cv2.waitKey(10)

# def change_start_work(msg:String):
#     if msg.data == '1':
#         GlobalVar.start_work = True
#     else:
#         GlobalVar.start_work = False

# def change_reaction_flag(msg:String):
#     GlobalVar.frame = 0
#     GlobalVar.reaction_flag=int(msg.data)
#     if GlobalVar.reaction_flag==1:
#         x1, y1 = GlobalVar.get_map_pose(GlobalVar.machine_odom_now_x, GlobalVar.machine_odom_now_y)
#         robot_states = GlobalVar.get_map_pose_theta()
#         rpy = GlobalVar.ori_to_rpy(
#             0.0, 0.0, robot_states[2], robot_states[3])
#         theta1 =  rpy[2]
#         # rospy.loginfo('角度: %f', alpha)
 
#         goal = MoveBaseGoal()
#         goal.target_pose.header.frame_id = 'map'
#         goal.target_pose.pose.position.x = 2
#         goal.target_pose.pose.position.y = 1
#         xyzw = GlobalVar.rpy2quaternion(0.0, 0.0, theta1)
#         goal.target_pose.pose.orientation.x = 0.0
#         goal.target_pose.pose.orientation.y = 0.0
#         goal.target_pose.pose.orientation.z = xyzw[2]
#         goal.target_pose.pose.orientation.w = xyzw[3]
#         # goal_name = Goals_name()
#         # goal_name.goal = goal
#         # ac.send_goal(goal)
#         # rospy.loginfo("start to navigate")
#         # ac.wait_for_result()
#         # rospy.loginfo("get to the waypoint")
#         #now_goal_pub.publish(goal)
#         #rospy.loginfo("change flag into 1, send goal...")
#         #now_goal_pub.publish(goal)
#     if  GlobalVar.reaction_flag in [0,1,2]:
#         GlobalVar.start_work=False
#     else:
#         GlobalVar.start_work=True

def start_recognize_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=1

def facial_det_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=2

def pose_det_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=3

def start_recognize_robbish_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=4

def collect_robbish_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=5
        

if __name__ == '__main__':

    face = FaceRecognition()
    yolov8 = Yolov8()
    mediapipe = Mediapipe()
    # yolov8.main_yolo()
    # mediapipe.main_mediapipe()


    # face_feature_pub=rospy.Publisher("/general_service_recognition",String,queue_size=10) 
    # #now_goal_pub = rospy.Publisher("/person/waypoint", Goals_name, queue_size=100)
    # # now_goal_pub = rospy.Publisher("/person/waypoint", MoveBaseGoal, queue_size=100)
    # ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # ac.wait_for_server()
    start_recognize_sub = rospy.Subscriber("start_recognize",String,start_recognize_callback)
    facial_det_sub = rospy.Subscriber("facial_deb",String,facial_det_callback)
    pose_det = rospy.Subscriber("pose_det",String,pose_det_callback)
    start_recognize_robbish = rospy.Subscriber("start_recognize_robbish",String,start_recognize_robbish_callback)
    collect_robbish = rospy.Subscriber("collect_robbish",String,collect_robbish_callback)


    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)


    # change_state=rospy.Subscriber("general_service_now_goals",String,change_start_work)
    # change_reaction_sub=rospy.Subscriber("/general_service_choose_face_or_pose",String,change_reaction_flag,queue_size=10)
    # s = String()
    # s.data = '4'
    # change_reaction_flag(s)
    # odom_grab_position_subscriber=rospy.Subscriber("odom",Odometry,GlobalVar.get_odom_message,queue_size=10)


    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)

    # show_image()
    # robot_states = GlobalVar.get_map_pose_theta()
    # rpy = GlobalVar.ori_to_rpy(
    #     0.0, 0.0, robot_states[2], robot_states[3])
    # theta1 =  rpy[2]
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = 'map'
    # goal.target_pose.pose.position.x = 1
    # goal.target_pose.pose.position.y = 1
    # xyzw = GlobalVar.rpy2quaternion(0.0, 0.0, theta1)
    # goal.target_pose.pose.orientation.x = 0.0
    # goal.target_pose.pose.orientation.y = 0.0
    # goal.target_pose.pose.orientation.z = xyzw[2]
    # goal.target_pose.pose.orientation.w = xyzw[3]
    # goal_name = Goals_name()
    # goal_name.goal = goal
    # now_goal_pub.publish(goal)
    rospy.spin()


