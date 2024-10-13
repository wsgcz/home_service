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

class FaceRecognition:
    def __init__(self, gpu_id=0, face_db='/home/shanhe/demo02/src/vis/face_db', threshold=1.24, det_thresh=0.50, det_size=(640, 640)):
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
    def __init__(self):
        pass

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
    
class Mediapipe:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(static_image_mode=True)

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
        #     #if i in range(33):
        #     img = cv2.circle(img, (cx, cy), radius[i], colors[i], -1)
        str_pose = self.get_pos(keypoints)            #获取姿态
        # cv2.putText(img, "POSE-{}".format(str_pose), (12, 100), cv2.FONT_HERSHEY_TRIPLEX,
        #             tl / 3, (255, 0, 0), thickness=tf)
        # cv2.putText(img, "FPS-{}".format(str(int(fps))), (12, 100), cv2.FONT_HERSHEY_SIMPLEX,tl/3, (255, 255, 0),thickness=tf)
        return str_pose
    
    def main_mediapipe(self, image):
        pose_str = self.process_frame(image)
        return pose_str
        
image,depth = None, None
yolo_model = ""

def image_callback(image_rgb,image_depth):
    global image, depth
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')
    depth = GlobalVar.bridge.imgmsg_to_cv2(
            image_depth, desired_encoding='passthrough')
    
    last_detection_time = 0
    detection_interval = 0.5  # 每0.5秒执行一次检测

    if time.time() - last_detection_time > detection_interval:
        last_detection_time = time.time()
        # GlobalVar.cb_mutex.acquire()
        if GlobalVar.reaction_flag == 1:
            rospy.loginfo(f"Now the task is 1")
            results = face.recognition(image)
            num = 0
            for result in results:
                if result != "Unknown":
                    num += 1
            if num > 0:
                rospy.set_param("Params.people_exist",1)
                rospy.loginfo("任务1检测到有人")
            rospy.set_param("Params.finish",1)
            # GlobalVar.reaction_flag == 0
        elif GlobalVar.reaction_flag == 2:
            rospy.loginfo(f"Now the task is 2")
            results = face.recognition(image)
            for result in results:
                if result != "Unknown":
                    rospy.set_param("Face_det.recog_msg",result)
                    rospy.loginfo(f"任务2检测到{result}")
                    break
            # GlobalVar.reaction_flag == 0
        elif GlobalVar.reaction_flag == 3:
            rospy.loginfo(f"Now the task is 3")
            pose_str = mediapipe.main_mediapipe(image)
            rospy.set_param("Body_det.recog_msg",pose_str)
            rospy.loginfo(f"任务3检测到{pose_str}")
            # GlobalVar.reaction_flag == 0
        elif GlobalVar.reaction_flag == 4:
            rospy.loginfo(f"Now the task is 4")
            confidence, result = yolov8.start_single_predict(yolo_model, image)
            if confidence > 0.5:
                rospy.set_param("arams.people_exist",1)
                rospy.loginfo("任务4检测到有垃圾")
            rospy.set_param("Params.finish",1)
            GlobalVar.reaction_flag == 0
        elif GlobalVar.reaction_flag == 5:
            rospy.loginfo(f"Now the task is 5")
            confidence, result = yolov8.start_single_predict(yolo_model, image)
            if confidence > 0.5:
                rospy.set_param("Robbish_det.recog_msg",result)
                rospy.loginfo(f"任务5检测到{result}")
            GlobalVar.reaction_flag == 0
        # GlobalVar.cb_mutex.release()

def start_recognize_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.reaction_flag=1

def facial_det_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.reaction_flag=2

def pose_det_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.reaction_flag=3

def start_recognize_robbish_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.reaction_flag=4

def collect_robbish_callback(msg:String):
    if msg.data == 'OK':
        GlobalVar.reaction_flag=5
        

if __name__ == '__main__':

    GlobalVar.reaction_flag = 3

    face = FaceRecognition()
    yolov8 = Yolov8()
    mediapipe = Mediapipe()

    start_recognize_sub = rospy.Subscriber("start_recognize",String,start_recognize_callback)
    facial_det_sub = rospy.Subscriber("facial_deb",String,facial_det_callback)
    pose_det = rospy.Subscriber("pose_det",String,pose_det_callback)
    start_recognize_robbish = rospy.Subscriber("start_recognize_robbish",String,start_recognize_robbish_callback)
    collect_robbish = rospy.Subscriber("collect_robbish",String,collect_robbish_callback)

    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)

    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)

    rospy.spin()


