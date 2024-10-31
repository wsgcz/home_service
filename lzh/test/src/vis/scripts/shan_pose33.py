#!/usr/bin/python3
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
sys.path.insert(0,'/home/lzh/test/src/vis')
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
    reaction_flag = -1
    start_work = False
    '''
    reaction_flag:
    -1:初始状态
    0:识别人体骨架，寻找朝向位置坐标，探索房间
    1:识别是否有人
    2:识别人脸
    3:姿态识别
    4:识别是否有垃圾
    5:垃圾识别 
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
    rotate_mutex = threading.Lock()
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
    # def put_data_into_quene(data):
    #     GlobalVar.queue_mutex.acquire()
    #     GlobalVar.pub_queue.put(data)
    #     rospy.loginfo("put into queue")
    #     GlobalVar.queue_mutex.release()

    # def get_odom_message(odom:Odometry):
    #     GlobalVar.machine_odom_now_x= odom.pose.pose.position.x
    #     GlobalVar.machine_odom_now_y=odom.pose.pose.position.y 

    #     odom_ox=odom.pose.pose.orientation.x
    #     odom_oy=odom.pose.pose.orientation.y
    #     odom_oz=odom.pose.pose.orientation.z
    #     odom_ow=odom.pose.pose.orientation.w
    #     _,_,GlobalVar.mathince_theta=transformations.euler_from_quaternion([odom_ox,odom_oy,odom_oz,odom_ow])

class FaceRecognition:
    def __init__(self, gpu_id=0, face_db='/home/lzh/test/src/vis/face_db', threshold=1.24, det_thresh=0.50, det_size=(640, 640)):
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

    def retrieve(self, addr):
        GlobalVar.face_mutex.acquire()
        face = cv2.imdecode(np.fromfile(addr,dtype=np.uint8),-1)
        store_name = addr.split('/')[-1].split('.')[0] 
        result = self.recognition(face)
        try:
            result=result[0]
        except BaseException as e:
            result="unknown"
        GlobalVar.face_mutex.release()
        return result
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
        
        # cv2.imwrite(os.path.join(self.face_db, f'{user_name}.jpg'), image)
        self.faces_embedding.append({
            "user_name": user_name,
            "feature": embedding
        })
        return "成功添加人脸"

class Yolov8:
    def __init__(self,model_path1="/home/lzh/test/src/main_function/models/yolov10n.pt", # 预训练
                 model_path2="/home/lzh/test/src/main_function/models/best_3people_1.pt", # 三人
                 model_path3="/home/lzh/test/src/main_function/models/yolov10n.pt"):
        self.model_yuxunlian = YOLO(model_path1)
        self.model_3people = YOLO(model_path2)
        self.model_yuxunlian2 = YOLO(model_path3)
###detect函数，对图像进行检测，并返回裁剪后的图像
###输入：图片
###输出：裁剪后的图片，框的宽度和高度，框的中心点，标签
    def detect(self,image, choice):
        results = list()
        nimg = image
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        if(choice == 1):
            output = self.model_yuxunlian(image)
        elif(choice == 2):
            output = self.model_3people(image)
        elif(choice == 3):
            output = self.model_yuxunlian2(image)
        name = list()
        confidence_max = -1
        # class_name_max = "nothing"
        # name_max = ""
        # class_id_max = 0
        # 获取类别名称字典
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
            cv2.imwrite(f"/home/lzh/peopel_{i}.jpg", change_image)
            # image_name = f"/home/lzh/{name[i]}.jpg"
            # cv2.imwrite(image_name,change_image)
            # 宽 高 
            w = half_w*2
            h = half_h*2
            if h < 2*w :
                continue
            #TODO
            # 人的移动(预训练模型的最大面积)和人的识别的长宽比不同   1.5 
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))
        # max_index = 0
        # for i in range (len(name)):
        #     if name[i][1] > confidence_max:
        #             confidence_max = name[i][1]
        #             max_index = i
        # max_result = list()
        # max_result.append(results[max_index])

        # 创建面积和索引的配对
        areas = [(result[1] * result[2], i) for i, result in enumerate(results)]
        # areas现在是: [(8000, 0), (2000, 1), (10800, 2)]
        # 按面积降序排序并获取排序后的索引
        sorted_indices = [i for _, i in sorted(areas, reverse=True)]
        # sorted_indices现在是: [2, 0, 1]
        # 重新排序results和name
        results = [results[i] for i in sorted_indices]

        print(f"-----------------识别的结果是{len(results)}-------------")
        return results
    
    def detect_yolo3(self,image, choice):

        # print(image)
        results = list()
        # image = "/home/shanhe/test123/tests/02B4A7F99A6243C61ABC3D34E150A188.jpg" # 正面
        # image = "/home/shanhe/test123/tests/DD0E93CEE01490071D266F79EFC85FBE.jpg" # 反面
        # image = "/home/shanhe/test123/tests/IMG_20241029_193312.jpg" # 侧面
        # image = cv2.imread(image)
        nimg = image
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        if(choice == 1):
            output = self.model_yuxunlian(image)
        elif(choice == 2):
            output = self.model_3people(image)
        elif(choice == 3):
            output = self.model_yuxunlian2(image)
        name = list()
        
        # class_name_max = "nothing"
        # name_max = ""
        # class_id_max = 0
        # 获取类别名称字典
        for r in output:
            # r.show()
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
                print(f"-------------i see person {class_name}, confidence {confidence}-----------------")
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
            # w = half_w*2
            # h = half_h*2
            # if h < 3*w :
            #     continue
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))
        
        # 创建面积和索引的配对
        areas = [(result[1] * result[2], i) for i, result in enumerate(results)]
        # areas现在是: [(8000, 0), (2000, 1), (10800, 2)]
        # 按面积降序排序并获取排序后的索引
        sorted_indices = [i for _, i in sorted(areas, reverse=True)]
        # sorted_indices现在是: [2, 0, 1]
        # 重新排序results和name
        results = [results[i] for i in sorted_indices]
        name = [name[i] for i in sorted_indices]

        max_index = -1
        # confidence_max = -1
        threshold = 0.7
        max_result = list()
        # print(f"----------------name 长度：{len(name)}-----------------------")
        for i in range (len(name)):
            w = results[i][1]
            h = results[i][2]
            print(f"----------------名字: {results[i][5]},w={w},h={h}-----------------------")
            if h < 2*w:
                continue
            if name[i][1] > threshold:
                print(f"--------------i see person {name[i][0]}, confidence {name[i][1]}-----------------")
                max_index = i
                break
        if not (max_index == -1):
            max_result.append(results[max_index])
        return max_result
    
    def detect_following(self,image, choice):
        results = list()
        nimg = image
        # plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
        if(choice == 1):
            output = self.model_yuxunlian(image)
        elif(choice == 2):
            output = self.model_3people(image)
        elif(choice == 3):
            output = self.model_yuxunlian2(image)
        name = list()
        
        # class_name_max = "nothing"
        # name_max = ""
        # class_id_max = 0
        # 获取类别名称字典
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
            cv2.imwrite(f"/home/lzh/peopel_{i}.jpg", change_image)
            # image_name = f"/home/lzh/{name[i]}.jpg"
            # cv2.imwrite(image_name,change_image)
            # 宽 高 
            w = half_w*2
            h = half_h*2
            if h < 1.5*w :
                continue
            #TODO
            # 人的移动(预训练模型的最大面积)和人的识别的长宽比不同   1.5 
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))
        max_index = -1
        s_max = -1
        for i in range (len(name)):
            if results[i][1]*results[i][2] > s_max:
                    s_max = results[i][1]*results[i][2]
                    max_index = i
        max_result = list()
        if not (max_index == -1):
            max_result.append(results[max_index])
        print(f"-----------------识别的结果是{len(max_result)}-------------")
        return max_result
    
class Mediapipe:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(static_image_mode=True,model_complexity=1,
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
        str_pose = "NO PERSON"
        keypoints = np.array(keypoints)

        ### 角度

        # 躯干与竖直轴
        # 左
        v1 = keypoints[11] - keypoints[23]
        v2 = np.array([0, -1]); # 竖直向上的向量
        from_23_11_to_y = mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[24]
        v2 = np.array([0, -1]); # 竖直向上的向量
        from_24_12_to_y = mediapipe.get_angle(v1, v2)

        # 小腿之间
        v1 = keypoints[27] - keypoints[25]
        v2 = keypoints[28] - keypoints[26]
        from_25_27_to_26_28 = mediapipe.get_angle(v1, v2)

        # 大腿之间
        v1 = keypoints[25] - keypoints[23]
        v2 = keypoints[26] - keypoints[24]
        from_23_25_to_24_26 = mediapipe.get_angle(v1,v2)

        # 躯干与大腿
        # 左
        v1 = keypoints[11] - keypoints[23]
        v2 = keypoints[25] - keypoints[23]
        from_23_to_11_25 = mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[24]
        v2 = keypoints[26] - keypoints[24]
        from_24_to_12_26 = mediapipe.get_angle(v1, v2)    

        # 大腿和小腿
        # 左
        v1 = keypoints[23] - keypoints[25]
        v2 = keypoints[27] - keypoints[25]
        from_25_to_23_27 = mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[24] - keypoints[26]
        v2 = keypoints[28] - keypoints[26]
        from_26_to_24_28 = mediapipe.get_angle(v1, v2)

        # 躯干与小臂
        # 左
        v1 = keypoints[23] - keypoints[11]
        v2 = keypoints[15] - keypoints[13]
        from_11_23_to_13_15 = mediapipe.get_angle(v1,v2)
        # 右
        v1 = keypoints[24] - keypoints[12]
        v2 = keypoints[16] - keypoints[14]
        from_12_24_to_14_16 = mediapipe.get_angle(v1,v2)

        # 手肘
        # 左
        v1 = keypoints[11] - keypoints[13]
        v2 = keypoints[15] - keypoints[13]
        from_13_to_11_15 = mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[14]
        v2 = keypoints[16] - keypoints[14]
        from_14_to_12_16 = mediapipe.get_angle(v1, v2)

        # 肩膀
        # 左
        v1 = keypoints[11] - keypoints[13]
        v2 = keypoints[11] - keypoints[23]
        from_11_to_13_23 = mediapipe.get_angle(v1, v2)
        # 右
        v1 = keypoints[12] - keypoints[14]
        v2 = keypoints[12] - keypoints[24]
        from_12_to_14_24 = mediapipe.get_angle(v1, v2)
        
        ### 距离
        
        # 肩膀之间
        distance_11_12 = mediapipe.get_distance(keypoints[11],keypoints[12])

        # 食指和嘴
        # 左
        distance_19_9 = mediapipe.get_distance(keypoints[19],keypoints[9])
        # 右
        distance_20_10 = mediapipe.get_distance(keypoints[20],keypoints[10])

        # 食指和耳
        # 左
        distance_19_7 = mediapipe.get_distance(keypoints[19],keypoints[7])
        # 右
        distance_20_8 = mediapipe.get_distance(keypoints[20],keypoints[8])

        # 手肘和肩膀
        # 左
        distance_15_12 = mediapipe.get_distance(keypoints[15],keypoints[12])
        # 右
        distance_16_11 = mediapipe.get_distance(keypoints[16],keypoints[11])

        ### 姿态识别

        # 蹲起
        # if 20<abs(from_23_11_to_y)<60 and 20<abs(from_24_12_to_y)<60:
        #     str_pose = "蹲起_1"
        #     if 30<abs(from_25_to_23_27)<150 and 30<abs(from_26_to_24_28)<150:
        #         str_pose = "蹲起"
        #         return str_pose
        #     return str_pose

        # 蹲起
        if abs(from_23_11_to_y)<45 and abs(from_24_12_to_y)<45:
            if abs(from_25_to_23_27)<90 or abs(from_26_to_24_28)<90:
                str_pose = "蹲起"
                return str_pose
            if abs(from_23_to_11_25)<150 or abs(from_24_to_12_26)<150: 
                if abs(from_25_to_23_27)<150 or abs(from_26_to_24_28)<150:
                    str_pose = "蹲起"
                    return str_pose

        if abs(from_23_11_to_y)<45 and abs(from_24_12_to_y)<45:
            # 站立
            str_pose = "站立"

            # 双手交叉
            if distance_15_12<0.75*distance_11_12 and distance_16_11<0.75*distance_11_12:
                str_pose = "双手交叉"
                return str_pose

            # 吸烟、打电话
            if abs(from_11_to_13_23)<120 and abs(from_12_to_14_24)<120:
                if distance_19_9<0.75*distance_11_12 or distance_20_10<0.75*distance_11_12 or distance_19_7<0.75*distance_11_12 or distance_20_8<0.75*distance_11_12:
                    distances = [distance_19_9,distance_20_10,distance_19_7,distance_20_8]
                    min_index, min_distance = min(enumerate(distances), key=lambda x: x[1])
                    if min_index==0 or min_index==1:
                        str_pose = "打电话"
                        return str_pose
                    else:
                        str_pose = "打电话"
                        return str_pose
                
            # if abs(abs(from_23_to_11_25)-90)<45 or abs(abs(from_24_to_12_26)-90)<45:
            #     str_pose = "坐"
            #     return str_pose

            # 举手、举双手
            # if abs(from_11_23_to_13_15)>100:
            #     str_pose = "举左手"
            #     if abs(from_12_24_to_14_16)>100:
            #         str_pose = "举双手"
            #     return str_pose
            # if abs(from_12_24_to_14_16)>100:
            #     str_pose = "举右手"
            #     if abs(from_11_23_to_13_15)>100:
            #         str_pose = "举双手"
            #     return str_pose
            
            # 举手、举双手
            if abs(from_11_to_13_23)>90 and abs(from_12_to_14_24)>90:
                str_pose = "举双手"
                return str_pose
            
            if abs(from_11_to_13_23)>90:
                str_pose = "举手"
                return str_pose
            
            if abs(from_12_to_14_24)>90:
                str_pose = "挥手"
                return str_pose
            
            # 行走
            if abs(from_25_27_to_26_28)>20 or abs(from_23_25_to_24_26)>20:
                str_pose = "行走"
                return str_pose
            return str_pose
        else :
            # 平躺
            str_pose = "平躺"
            # if 45<abs(from_11_23_to_13_15)<135 and 45<abs(from_12_24_to_14_16)<135:
            #     str_pose = "俯卧撑"
            #     return str_pose
            return str_pose
        
    def process_frame(self, img):
        # start_time = time.time()
        h, w = img.shape[0], img.shape[1]               # 高和宽
        # 调整字体
        # tl = round(0.005 * (img.shape[0] + img.shape[1]) / 2) + 1
        # tf = max(tl-1, 1)
        # BRG-->RGB
        img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # 将RGB图像输入模型，获取 关键点 预测结果
        results = self.pose.process(img_RGB)
        keypoints = ['' for i in range(33)]
        if results.pose_landmarks:
            # self.mp_drawing.draw_landmarks(img, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            for i in range(33):
                cx = int(results.pose_landmarks.landmark[i].x * w)
                cy = int(results.pose_landmarks.landmark[i].y * h)
                keypoints[i] = (cx, cy)                                 # 得到最终的33个关键点
        else:
            # print("NO PERSON")
            struction = "NO PERSON"
            # img = cv2.putText(img, struction, (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (255, 255, 0),6)
            return struction
        # end_time = time.time()
        # process_time = end_time - start_time            # 图片关键点预测时间
        # fps = 1 / process_time                          # 帧率
        # colors = [[random.randint(0,255) for _ in range(3)] for _ in range(33)]
        # radius = [random.randint(8,15) for _ in range(33)]
        # for i in range(33):
        #     cx, cy = keypoints[i]
            # if i in range(33):        # end_time = time.time()
        # process_time = end_time - start_time            # 图片关键点预测时间
        # fps = 1 / process_time                          # 帧率
        # colors = [[random.randint(0,255) for _ in range(3)] for _ in range(33)]
        # radius = [random.randint(8,15) for _ in range(33)]
        # for i in range(33):
        #     cx, cy = keypoints[i]
        #     #if i in range(33):
            # img = cv2.circle(img, (cx, cy), radius[i], colors[i], -1)
            # img = cv2.circle(img, (cx, cy), radius[i], colors[i], -1)
        # print(keypoints)
        str_pose = self.get_pos(keypoints)            #获取姿态
        # cv2.putText(img, "POSE-{}".format(str_pose), (12, 100), cv2.FONT_HERSHEY_TRIPLEX,
        #             tl / 3, (255, 0, 0), thickness=tf)
        # cv2.putText(img, "FPS-{}".format(str(int(fps))), (12, 100), cv2.FONT_HERSHEY_SIMPLEX,tl/3, (255, 255, 0),thickness=tf)
        return str_pose
    
    def detect(self, image, width, height, mid_x, mid_y, name, depth):
        GlobalVar.mediapipe_mutex.acquire()
        print("man now i looking at you!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        results = self.pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        print(f'result is :{results.pose_world_landmarks}')
        if not results.pose_world_landmarks:
            return None
        alpha = self.get_alpha(results)
        alpha = alpha/pi*180
        print("this is alpha ",alpha ,"and name is ",name)
        goal = self.convert_machine_axis_to_world(results,width,height,mid_x,mid_y,name,depth)
        GlobalVar.mediapipe_mutex.release()
        print("---------i will sent a goal----------")
        # ac.send_goal(goal)
        # print("-----------i have sent a goal-----")
        # ac.wait_for_result()
        return goal
    #确保人在视野的中心
    def rotate(self, image, width, height, mid_x, mid_y, name, depth):
        GlobalVar.rotate_mutex.acquire()
        cmdvel = Twist()
        results = self.pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        if not results.pose_world_landmarks:
            return None
        sumx = 0
        #print(len(results.pose_world_landmark))
        for i in range(33):
            sumx = sumx +int(results.pose_world_landmarks.landmark[i].x*width+ mid_x - 0.5 * width)
        x_average = sumx / 33 + 480#以所有关键点的平均x坐标作为人在图中的位置
        print(f"the x_average is {x_average}")
        cmdvel.linear.x = 0
        cmdvel.linear.y = 0
        cmdvel.linear.z = 0
        cmdvel.angular.x = 0
        cmdvel.angular.y = 0
        if x_average < width / 2 :
            cmdvel.angular.z = 2
        else:
            cmdvel.angular.z = -2
        vel_pub.publish(cmdvel)
        rospy.sleep(0.2)#每次转0.5弧度试试
        cmdvel.angular.z=0
        vel_pub.publish(cmdvel)
        rospy.sleep(0.5)
        GlobalVar.rotate_mutex.release()
        return None


    @staticmethod
    def get_alpha(results):
        z_left = results.pose_world_landmarks.landmark[11].z
        z_right = results.pose_world_landmarks.landmark[12].z
        x_left = results.pose_world_landmarks.landmark[11].x
        x_right = results.pose_world_landmarks.landmark[12].x
        z_distance=(z_right-z_left)
        x_distance=(x_right-x_left)
        alpha = pi + atan2(z_distance, x_distance)
        return alpha
    
    # 返回物体fps = 1 / process_time                          # 帧率
        # colors = [[random.randint(0,255) for _ in range(3)] for _ in range(33)]
        在机器人坐标系下的真实距离
    # 输入： 物体在图片中的像素位置u_img(x),v_img(y),d:深度，单位mm
    # 输出： 用米做单位的坐标，机器人坐标系
    @staticmethod
    def real_pose(u_img, v_img, d):
        other_angle=(v_img-270)*43/540
        this_angle=0.46*180/pi
        now_angle=abs(this_angle+other_angle)*pi/180
        now_angle1=abs(other_angle)*pi/180
        Z = d/sqrt(1+(u_img-GlobalVar.P_x)**2/GlobalVar.f_x**2+(v_img-GlobalVar.P_y)**2/GlobalVar.f_y**2)
        X = (u_img-GlobalVar.P_x)*Z/GlobalVar.f_x
        Y = (v_img-GlobalVar.P_y)*Z/GlobalVar.f_y
        y = (Y/1000)
        zzz = d/1000*cos(now_angle)+0.24-y*sin(now_angle1)
        # yyy=1.62-d/1000*math.sin(0.4)+y*math.cos(0.4)
        return X/1000+0.015, y, zzz
    
    @staticmethod
    def convert_machine_axis_to_world(results,width,height,x_mid,y_mid,name,depth):
        if results.pose_world_landmarks == None:
            return 
        # 计算节点在真实图片中的像素位置，不过landmark的x的单位肯定是width吗，有待商榷
        right_hip_x = int(results.pose_landmarks.landmark[24].x * width + x_mid - 0.5 * width)
        right_hip_y = int(results.pose_landmarks.landmark[24].y * height + y_mid - 0.5 * height)
        left_hip_x = int(results.pose_landmarks.landmark[23].x * width + x_mid - 0.5 * width)
        left_hip_y = int(results.pose_landmarks.landmark[23].y * height + y_mid - 0.5 * height)
        left_shoulder_x = int(results.pose_landmarks.landmark[11].x * width + x_mid - 0.5 * width)
        left_shoulder_y = int(results.pose_landmarks.landmark[11].y * height + y_mid - 0.5 * height)
        right_shoulder_x = int(results.pose_landmarks.landmark[12].x * width + x_mid - 0.5 * width)
        right_shoulder_y = int(results.pose_landmarks.landmark[12].y * height + y_mid - 0.5 * height)
        if right_hip_x >= 960: right_hip_x = 959
        if right_hip_y >= 540: right_hip_y = 539
        if left_hip_x >= 960: left_hip_x = 959
        if left_hip_y >= 540: left_hip_y = 539
        depth_right_hip = depth[right_hip_y][right_hip_x]
        depth_left_hip = depth[left_hip_y][left_hip_x]
        depth_left_shoulder = depth[left_shoulder_y][left_shoulder_x]
        depth_right_shoulder = depth[right_shoulder_y][right_shoulder_x]
        depths = [depth_left_hip, depth_left_shoulder, depth_right_hip, depth_right_shoulder]
        length = len(depths)
        for d in depths:
            if d < GlobalVar.eps:
                length -= 1
        average_depth = sum(depths) / length
        real_person = mediapipe.real_pose(x_mid, y_mid, average_depth)
        rospy.loginfo(f"this is the relative pos of person: {real_person}")
        if real_person[2] < GlobalVar.eps:
            return
        # 接下来是坐标转换
        alpha = mediapipe.get_alpha(results)
        robot_states = GlobalVar.get_map_pose_theta()
        rospy.loginfo(f"this is robot position:{robot_states}")
        rpy = GlobalVar.ori_to_rpy(0.0, 0.0, robot_states[2], robot_states[3])
        # ren qian 1.5m
        dist = 1.5
        z_second = real_person[2] - dist * cos(alpha)
        x_second = real_person[0] + dist * sin(alpha)
        # if alpha == 0.5 * pi or alpha == 1.5 * pi:
        #     z_second -= 0.2
        if alpha >= 0.8 * pi and alpha <= 1.2 * pi:
            z_second += 0.1

        # y坐标，即高度没用,地图x方向向右，y方向向前
        # 最后x应该向前，y向左
        x1, y1 = GlobalVar.get_map_pose(x_second, z_second)
        rospy.loginfo(f"ultimate pos in the coordinate:{x1},{y1}")
        yaw = alpha - 2*pi
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = y1
        goal.target_pose.pose.position.y = -x1
        xyzw = GlobalVar.rpy2quaternion(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = xyzw[2]
        goal.target_pose.pose.orientation.w = xyzw[3]
        goal.target_pose.header.stamp = rospy.Time.now()
        return goal
    
    def main1_mediapipe(self, image):
        pose_str = self.process_frame(image)
        return pose_str

image,depth = None, None

pose_dict = {}
pose_count = 0
duration = 5  # 持续时间（秒）
pose_start_time = time.time()
is_first_time = True
# pose_num = 1000  # 检测次数
def mediapipe_pose(pose_str):
    global pose_dict, pose_count, duration, pose_start_time, is_first_time
    # while count < pose_num:
    if is_first_time:
        pose_start_time = time.time()
        is_first_time = False
        # print("---------------------------------time is begin-----------------------------")
    elif time.time() - pose_start_time < duration:
        # print("---------------------------------time is begin 2-----------------------------")

        if pose_str != "NO PERSON":
            if pose_str in pose_dict:
                pose_dict[pose_str] += 1
            else:
                pose_dict[pose_str] = 1
        pose_count += 1
    else:
        # print("---------------------------------time is begin3-----------------------------")

        pose_start_time = time.time()
        if len(pose_dict) == 0:
            pose_str = 'NO PERSON'
        else:
            most_common_pose = max(pose_dict, key=pose_dict.get)
            max_count = pose_dict[most_common_pose]
            lie_count = pose_dict.get('躺', 0)  # 如果没有'躺'这个key，返回0
            stand_count = pose_dict.get('站立', 0)  # 如果没有'站立'这个key，返回0
            pose_str = most_common_pose
            if lie_count > 0.1 * pose_count and stand_count > 0.1 * pose_count:
                pose_str = '摔倒'
        # pose_str = mediapipe.main1_mediapipe(image)
        rospy.loginfo(f"任务3检测到{pose_str}")
        pose_det_pub.publish(pose_str)
        pose_dict = {}
        pose_count = 0
        is_first_time = True
        GlobalVar.reaction_flag = -1
        GlobalVar.frame = 1
        # time.sleep(5)

last_detection_time = 0

def image_callback(image_rgb,image_depth):
    # print("--------------i am in image-callback------------")
    global image, depth, last_detection_time
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')
    depth = GlobalVar.bridge.imgmsg_to_cv2(
            image_depth, desired_encoding='passthrough')
    
    #改成只看第一帧
    if GlobalVar.frame == 0 :
        GlobalVar.cb_mutex.acquire()

        if GlobalVar.reaction_flag == 0:
            rospy.loginfo(f"Now the task is 0")

            # # 三人模型，置信度0.8才出
            # detect_result = yolov8.detect_yolo3(image, 2)# 三人模型
            # yes = 0
            # for person in detect_result:
            #     result = person[5]
            #     if result == "person1" :
            #         rospy.loginfo(f"find people zzy")
            #         start_recognize_pub.publish("zzy")
            #         GlobalVar.last_person += 1
            #         GlobalVar.reaction_flag = -1
            #         rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}')
            #         yes = 1
            #         break
            #     if result == "person2" :
            #         rospy.loginfo(f"find people zkw")
            #         start_recognize_pub.publish("zkw")
            #         GlobalVar.last_person += 1
            #         GlobalVar.reaction_flag = -1
            #         rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}')
            #         yes = 1
            #         break
            #     if result == "person3" :
            #         rospy.loginfo(f"find people wmy")
            #         start_recognize_pub.publish("wmy")
            #         GlobalVar.last_person += 1
            #         GlobalVar.reaction_flag = -1
            #         rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}')
            #         yes = 1
            #         break
            # if yes == 0:
            #     rospy.loginfo("no people")
            #     start_recognize_pub.publish("0")
            #     # cv2.imwrite(store_path,image)
            #     # GlobalVar.last_person += 1
            #     GlobalVar.reaction_flag = -1
            #     rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}')
            # GlobalVar.frame = 1

            # 融合
            detect_result = yolov8.detect(image, 1)# 预训练模型
            yes = 0
            for person in detect_result:
                result = person[5]
                if result == "person":
                    results_face = face.recognition(person[0])
                    rospy.loginfo(f"results_face={results_face}")
                    for result in results_face:
                        if result != "unknown":
                            rospy.loginfo(f"find people {result}")
                            start_recognize_pub.publish(result)
                            # GlobalVar.last_person += 1
                            GlobalVar.reaction_flag = -1
                            rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}')
                            yes = 1
                            break
            if yes == 0:
                rospy.loginfo("no people")
                start_recognize_pub.publish("0")
                # cv2.imwrite(store_path,image)
                # GlobalVar.last_person += 1
                GlobalVar.reaction_flag = -1
                rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}')
            GlobalVar.frame = 1

        elif GlobalVar.reaction_flag == 1:
            rospy.loginfo(f"Now the task is 1")
            pose_str = mediapipe.main1_mediapipe(image)
            mediapipe_pose(pose_str)
                
        if GlobalVar.followflag == 1:
            rospy.loginfo("start follow")
            if ((time.time() - last_detection_time) > 1):
                last_detection_time = time.time()
                rospy.loginfo(f"Now the task is follow")
                detect_result = yolov8.detect_following(image, 2) # 三人模型
                for person in detect_result:
                    if (person[5] == "person1" or person[5] == "person2" or person[5] == "person3"):
                        print("-------------------------检测到人了!----------------------------")
                        if (not (abs(person[3] - 480)) < 98):
                            print("--------------------------------开始旋转了！--------------------------------")
                            mediapipe.rotate(image, 960,540,person[3],person[4],person[5],depth)

        GlobalVar.cb_mutex.release()


def start_recognize_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.reaction_flag=0
        GlobalVar.frame = 0

# def facial_det_callback(msg:String):
#     if msg.data == 'OK':
#         GlobalVar.frame = 0
#         GlobalVar.reaction_flag=1

def pose_det_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=1

def follow_people_callback(msg:String):
    if msg.data == "1":
        GlobalVar.followflag = 1
        rospy.loginfo("follow_people_callback 1")
        GlobalVar.frame = 0
    if msg.data == "0":
        GlobalVar.followflag = 0
        rospy.loginfo("follow_people_callback 0")
        GlobalVar.frame = 1

if __name__ == '__main__':
    GlobalVar.reaction_flag = -1
    GlobalVar.followflag = 0

    face = FaceRecognition()
    yolov8 = Yolov8()
    mediapipe = Mediapipe()

    follow_people_sub = rospy.Subscriber("follow_people",String,follow_people_callback)
    start_recognize_sub = rospy.Subscriber("start_recognize",String,start_recognize_callback)
    # facial_det_sub = rospy.Subscriber("facial_det",String,facial_det_callback)
    pose_det = rospy.Subscriber("pose_det",String,pose_det_callback)

    # orient_angle_pub = rospy.Publisher("orient_angle_reply",MoveBaseGoal,queue_size=10)
    start_recognize_pub = rospy.Publisher("start_recognize_reply", String, queue_size=10)
    # facial_det_pub = rospy.Publisher("facial_det_reply", String, queue_size=10)
    pose_det_pub = rospy.Publisher("pose_det_reply", String, queue_size=10)

    vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    # ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # ac.wait_for_server()
    #print("----------------i am in ac ---------------")

    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)

    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)

    rospy.spin()


