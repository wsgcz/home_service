#!/usr/bin/python3
import os,cv2,time,rospy,tf2_ros,threading,insightface,message_filters
import numpy as np
import mediapipe as mp
import numpy as np
from ultralytics import YOLO
from sklearn import preprocessing
from PIL import Image
from math import atan2,pi,sqrt,sin,cos
from queue import Queue
from sklearn import preprocessing
from cv_bridge import CvBridge
from tf import transformations
from tf2_geometry_msgs import PointStamped,PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist

# 定义全局变量
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

    frame = 0  # 第一张图片
    last_person = 0
    reaction_flag = -1 # 当前状态
    start_work = False

    rospy.init_node("pose3")
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    bridge = CvBridge()

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
        # 创建人脸库文件夹
        if not os.path.exists(self.face_db):
            os.makedirs(self.face_db)
        # 遍历人脸库文件夹
        for root, dirs, files in os.walk(self.face_db):
            for file in files:
                input_image = cv2.imread(os.path.join(root, file)) # 读取图片
                user_name = file.split(".")[0] # 获取人脸库中的人脸名字
                faces = self.model.get(input_image) # 检测人脸
                # 仅支持单个人脸
                if faces:
                    face = faces[0] # 获取人脸信息
                    embedding = np.array(face.embedding).reshape((1, -1)) # 获取人脸特征
                    embedding = preprocessing.normalize(embedding) # 归一化
                    # 将人脸信息添加到self.faces_embedding
                    self.faces_embedding.append({
                        "user_name": user_name,
                        "feature": embedding
                    })
                    print(f"已加载人脸: {user_name}")
                # 未检测到人脸
                else:
                    print(f"在 {file} 中未检测到人脸")

    # 人脸识别
    # 输入：图片
    # 输出：facedb中匹配到的人脸的名字的列表
    def recognition(self, image):
        # 检测人脸
        faces = self.model.get(image)
        results = list() # 人脸识别结果
        for face in faces:
            embedding = np.array(face.embedding).reshape((1, -1)) # 获取人脸特征
            embedding = preprocessing.normalize(embedding) # 归一化
            user_name = "unknown" # 初始化
            for com_face in self.faces_embedding:
                r = self.feature_compare(embedding, com_face["feature"], self.threshold) # 特征比较
                if r:
                    user_name = com_face["user_name"] # 匹配到人脸
                    break
            results.append(user_name) # 添加到结果中
        return results

    # 特征比较
    # 如果相似度大于阈值，返回True，否则返回False
    @staticmethod
    def feature_compare(feature1, feature2, threshold):
        diff = np.subtract(feature1, feature2) 
        dist = np.sum(np.square(diff), 1)
        return dist < threshold

class Yolov8:
    def __init__(self,model_path1="/home/lzh/test/src/main_function/models/yolov10n.pt"): # 预训练模型路径
        self.model_yuxunlian = YOLO(model_path1)

    # detect函数，对图像进行检测，并返回裁剪后的图像
    # 输入：图片
    # 输出：裁剪后的图片，框的宽度和高度，框的中心点，标签
    def detect(self,image, choice):
        results = list() # 存储检测结果
        nimg = image # 存储裁剪后的图像
        if(choice == 1):
            output = self.model_yuxunlian(image) # 检测图像
        name = list()

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
            # 如果没有检测到任何对象
            if len(r.boxes) == 0:
                continue
        
        for i in range (len(name)):
            xywh = boxes.cpu().xywh.detach().numpy() 
            x_center = int(xywh[i][0]) # 框的中心点x坐标
            y_center = int(xywh[i][1]) # 框的中心点y坐标
            half_w = int(xywh[i][2]/2) # 框的宽度的一半
            half_h = int(xywh[i][3]/2) # 框的高度的一半
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]#裁减之后的框
            # cv2.imwrite(f"/home/lzh/peopel_{i}.jpg", change_image)
            # 宽 高 
            w = half_w*2
            h = half_h*2
            # 如果框的高度小于2倍宽，则跳过
            if h < 2*w :
                continue
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))# 存储裁剪后的图像

        # 按照面积降序排序
        # 创建面积和索引的配对
        areas = [(result[1] * result[2], i) for i, result in enumerate(results)]
        # 按面积降序排序并获取排序后的索引
        sorted_indices = [i for _, i in sorted(areas, reverse=True)]
        # 重新排序results和name
        results = [results[i] for i in sorted_indices]

        # print(f"-----------------识别的结果是{len(results)}-------------")
        return results
    
    def detect_following(self,image, choice):
        results = list() # 存储检测结果
        nimg = image # 存储裁剪后的图像
        if(choice == 1):
            output = self.model_yuxunlian(image)
        name = list() # 存储类别名称
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
            # 如果没有检测到任何对象
            if len(r.boxes) == 0:
                continue

        for i in range (len(name)):
            xywh = boxes.cpu().xywh.detach().numpy() 
            x_center = int(xywh[i][0]) # 框的中心点x坐标
            y_center = int(xywh[i][1]) # 框的中心点y坐标
            half_w = int(xywh[i][2]/2) # 框的宽度的一半
            half_h = int(xywh[i][3]/2) # 框的高度的一半
            change_image=nimg[y_center-half_h:y_center+half_h,x_center-half_w:x_center+half_w]#裁减之后的框
            results.append((change_image,half_w*2,half_h*2,x_center,y_center,name[i][0]))# 存储裁剪后的图像

        # 按照面积降序排序
        # 创建面积和索引的配对
        areas = [(result[1] * result[2], i) for i, result in enumerate(results)]
        # 按面积降序排序并获取排序后的索引
        sorted_indices = [i for _, i in sorted(areas, reverse=True)]
        # 重新排序results和name
        results = [results[i] for i in sorted_indices]
        name = [name[i] for i in sorted_indices]

        max_index = -1
        max_result = list()
        for i in range (len(name)):
            w = results[i][1]
            h = results[i][2]
            # 如果框的高度小于1.5倍宽，则跳过
            if h < 1.5*w:
                continue
            # 如果类别不是person，则跳过
            if not (name[i][0] == "person"):
                continue
            # print(f"----------------名字: {results[i][5]},w={w},h={h}-----------------------")
            max_result.append(results[i]) # 存储裁剪后的图像
            max_index = 1 
            break
        if (max_index == -1): # 如果没有检测到person，则返回空列表
            max_result = [(0,0,0,0,0,"no person")]
        # print(f"-----------------识别的结果是{max_result[0][5]}-------------")
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

    # 得到角度       
    # v2相对于v1顺时针小于0、逆时针大于0
    def get_angle(self, v1, v2):
        angle = np.dot(v1, v2) / (np.sqrt(np.sum(v1 * v1)) * np.sqrt(np.sum(v2 * v2))) 
        angle = np.arccos(angle) / 3.14159265 * 180
        cross = v2[0] * v1[1] - v2[1] * v1[0]
        if cross < 0:
            angle = - angle
        return angle

    # 得到距离
    def get_distance(self, v1, v2):
        return np.sqrt(np.sum((v1 - v2) ** 2))

    def get_pos(self, keypoints):
        """
        节点如下：
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
            return str_pose
        
    def process_frame(self, img):
        h, w = img.shape[0], img.shape[1]               # 高和宽
        img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # BGR转RGB
        # 将RGB图像输入模型，获取 关键点 预测结果
        results = self.pose.process(img_RGB) # 得到33个关键点
        keypoints = ['' for i in range(33)]
        if results.pose_landmarks:
            for i in range(33):
                cx = int(results.pose_landmarks.landmark[i].x * w)
                cy = int(results.pose_landmarks.landmark[i].y * h)
                keypoints[i] = (cx, cy)    # 得到最终的33个关键点
        else:
            struction = "NO PERSON" # 没有检测到人
            return struction
        str_pose = self.get_pos(keypoints)    #获取姿态
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
        return goal
    
    # 跟随，确保人在视野的中心
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
            cmdvel.angular.z = 0.01 * (width/2 - x_average)
        else:
            cmdvel.angular.z = -0.01 * (width/2 - x_average)
        vel_pub.publish(cmdvel)
        rospy.sleep(0.2)#每次转1弧度试试
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

# 初始化
pose_dict = {} # 存储每个姿势的计数
pose_count = 0 # 计数器
duration = 5  # 持续时间（秒）
pose_start_time = time.time() # 记录姿势开始时间
is_first_time = True # 是否是第一次检测
def mediapipe_pose(pose_str):
    global pose_dict, pose_count, duration, pose_start_time, is_first_time
    if is_first_time: # 如果是第一次检测，记录开始时间
        pose_start_time = time.time()
        is_first_time = False
    elif time.time() - pose_start_time < duration: # 如果持续时间未到，继续计数
        if pose_str != "NO PERSON": # 如果姿势不为"NO PERSON"，计数
            if pose_str in pose_dict: # 如果姿势已经存在，计数加1
                pose_dict[pose_str] += 1
            else: # 如果姿势不存在，添加到字典中，计数为1
                pose_dict[pose_str] = 1
        pose_count += 1
    else:
        # 如果持续时间到了，输出最常见的姿势和计数
        pose_start_time = time.time() # 重置开始时间
        if len(pose_dict) == 0: # 如果没有检测到姿势，输出"NO PERSON"
            pose_str = 'NO PERSON'
        else:
            most_common_pose = max(pose_dict, key=pose_dict.get) # 获取出现次数最多的姿势
            # 计算躺下和站立的计数
            lie_count = pose_dict.get('躺', 0)  # 如果没有'躺'这个key，返回0
            stand_count = pose_dict.get('站立', 0)  # 如果没有'站立'这个key，返回0
            pose_str = most_common_pose
            if lie_count > 0.1 * pose_count and stand_count > 0.1 * pose_count: # 如果躺下和站立的计数都大于总计数的10%，输出"摔倒"
                pose_str = '摔倒'
        rospy.loginfo(f"任务3检测到{pose_str}") # 输出检测结果
        pose_det_pub.publish(pose_str) # 发布检测结果
        pose_dict = {} # 重置字典
        pose_count = 0 # 重置计数器
        is_first_time = True # 重置标志
        GlobalVar.reaction_flag = -1 # 重置当前任务
        GlobalVar.frame = 1 # 重置帧数

last_detection_time = 0 # 记录上一次检测时间

def image_callback(image_rgb,image_depth):
    global image, depth
    # 将ROS图像消息转换为OpenCV图像
    image = GlobalVar.bridge.imgmsg_to_cv2(
            image_rgb, desired_encoding='passthrough')
    depth = GlobalVar.bridge.imgmsg_to_cv2(
            image_depth, desired_encoding='passthrough')
    
    # 只看第一帧
    if GlobalVar.frame == 0 :
        # 线程锁,为了防止在上一帧还没处理完就再次进入回调函数
        GlobalVar.cb_mutex.acquire()

        if GlobalVar.reaction_flag == 0: # 人脸识别
            rospy.loginfo(f"Now the task is 人脸识别")
            detect_result = yolov8.detect(image, 1) # 通过yolo模型进行对人的检测，返回符合长宽比限制以及最大面积的人
            yes = 0 # 是否识别到人脸
            for person in detect_result: 
                result = person[5] # 识别结果
                if result == "person": # 如果识别结果是person
                    # cv2.imwrite(f"/home/lzh/{person[5]}.jpg",person[0])
                    results_face = face.recognition(person[0]) # 使用insightface进行人脸识别
                    rospy.loginfo(f"results_face={results_face}") # 打印识别结果
                    for result in results_face: # 遍历识别结果
                            rospy.loginfo(f"find people {result}") # 打印识别结果
                            start_recognize_pub.publish(result) # 发布识别结果
                            GlobalVar.reaction_flag = -1 # 重置当前任务
                            rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}') # 打印当前任务
                            yes = 1 # 标记已经识别到人脸
                            break
            if yes == 0: # 如果没有识别到人脸
                rospy.loginfo("no people") # 打印没有识别到人脸
                start_recognize_pub.publish("0") # 发布识别结果
                GlobalVar.reaction_flag = -1 #  重置当前任务
                rospy.loginfo(f'GlobalVar.reaction_flag:{GlobalVar.reaction_flag}') # 打印当前任务
            GlobalVar.frame = 1 # 重置帧数为1，即在主函数发布下一次任务之前，不会执行此回调函数

        elif GlobalVar.reaction_flag == 1: # 姿态识别
            rospy.loginfo(f"Now the task is 姿态识别")
            pose_str = mediapipe.main1_mediapipe(image) # 姿态识别
            mediapipe_pose(pose_str)
                
        if GlobalVar.followflag == 1: # 跟随
            if ((time.time() - last_detection_time) > 1): # 如果距离上次检测时间大于1秒
                last_detection_time = time.time() # 更新上次检测时间
                rospy.loginfo(f"Now the task is 跟随") # 打印当前任务
                detect_result = yolov8.detect_following(image, 1) # 获得图片中最大面积的人
                for person in detect_result: # 遍历识别结果
                    if (person[5] == "person"): # 如果识别结果是person
                        print("-------------------------检测到人了!----------------------------")
                        if (not (abs(person[3] - 480)) < 98): # 如果识别结果不在屏幕中间，则进行旋转
                            print("--------------------------------开始旋转了！--------------------------------")
                            mediapipe.rotate(image, 960,540,person[3],person[4],person[5],depth) # 旋转
        GlobalVar.cb_mutex.release() # 释放线程锁



def start_recognize_callback(msg:String): # 人脸识别的回调函数
    if msg.data == 'OK':
        GlobalVar.reaction_flag=0
        GlobalVar.frame = 0 #在回调函数中将frame设置为0，可以对下一帧图像进行处理。

def pose_det_callback(msg:String): # 姿态识别的回调函数
    if msg.data == 'OK':
        GlobalVar.frame = 0
        GlobalVar.reaction_flag=1

def follow_people_callback(msg:String): # 跟随的回调函数
    if msg.data == "1": # 如果接收到跟随的信号，则跟随
        GlobalVar.followflag = 1
        rospy.loginfo("follow_people_callback 1")
        GlobalVar.frame = 0
    if msg.data == "0": # 如果接收到停止跟随的信号，则停止跟随
        GlobalVar.followflag = 0
        rospy.loginfo("follow_people_callback 0")
        GlobalVar.frame = 1

if __name__ == '__main__':
    GlobalVar.reaction_flag = -1 # 初始化当前任务为-1，即没有任务
    GlobalVar.followflag = 0 # 初始化跟随标志为0，即不跟随

    face = FaceRecognition() # 实例化人脸识别类
    yolov8 = Yolov8() # 实例化YOLO识别类
    mediapipe = Mediapipe() # 实例化姿态识别类

    # 创建ROS节点
    follow_people_sub = rospy.Subscriber("follow_people",String,follow_people_callback)
    start_recognize_sub = rospy.Subscriber("start_recognize",String,start_recognize_callback)
    pose_det = rospy.Subscriber("pose_det",String,pose_det_callback)
    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)

    # 创建ROS发布者
    start_recognize_pub = rospy.Publisher("start_recognize_reply", String, queue_size=10)
    pose_det_pub = rospy.Publisher("pose_det_reply", String, queue_size=10)
    vel_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    # 创建ROS同步定时器
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)

    # spin
    rospy.spin()


