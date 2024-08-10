from math import fabs
import os

import cv2
import insightface
import numpy as np
from sklearn import preprocessing
import time

class FaceRecognition:
    def __init__(self, gpu_id=0, face_db='/home/linxi/ServiceRobot-General/src/pose_estimate/face_db', threshold=1.24, det_thresh=0.50, det_size=(640, 640)):
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

        # 加载人脸识别模型，当allowed_modules=['detection', 'recognition']时，只单纯检测和识别
        self.model = insightface.app.FaceAnalysis(root='/home/linxi/Pictures',
                                                  allowed_modules=['detection', 'recognition'],
                                                  providers=['CUDAExecutionProvider'])
        self.model.prepare(ctx_id=self.gpu_id, det_thresh=self.det_thresh, det_size=self.det_size)
        # 人脸库的人脸特征
        self.faces_embedding = list()
        # 加载人脸库中的人脸
        self.load_faces(self.face_db)

    # 加载人脸库中的人脸
    def load_faces(self, face_db_path):
        if not os.path.exists(face_db_path):
            os.makedirs(face_db_path)
        for root, dirs, files in os.walk(face_db_path):
            for file in files:
                input_image = cv2.imdecode(np.fromfile(os.path.join(root, file), dtype=np.uint8), 1)
                user_name = file.split(".")[0]
                face = self.model.get(input_image)[0]
                embedding = np.array(face.embedding).reshape((1, -1))
                embedding = preprocessing.normalize(embedding)
                self.faces_embedding.append({
                    "user_name": user_name,
                    "feature": embedding
                })

    # 人脸识别
    def recognition(self, image):
        faces = self.model.get(image)
        results = list()
        for face in faces:
            # 开始人脸识别
            embedding = np.array(face.embedding).reshape((1, -1))
            embedding = preprocessing.normalize(embedding)
            user_name = "unknown"
            for com_face in self.faces_embedding:
                r = self.feature_compare(embedding, com_face["feature"], self.threshold)
                print(1111111111111111)
                if r:
                    user_name = com_face["user_name"]
            results.append(user_name)
        return results

    @staticmethod
    def feature_compare(feature1, feature2, threshold):
        diff = np.subtract(feature1, feature2)
        dist = np.sum(np.square(diff), 1)
        if dist < threshold:
            return True
        else:
            return False

    def register(self, image, user_name):
        faces = self.model.get(image)
        print(len(faces))
        if (len(faces)>1):
            return "有多个人"
        if len(faces)<=0:
            return '图片检测不到人脸'
        
        # print( "now there are "+str(len(faces)))
        # 判断人脸是否存在
        embedding = np.array(faces[0].embedding).reshape((1, -1))
        embedding = preprocessing.normalize(embedding)
        is_exits = False
        for com_face in self.faces_embedding:
            r = self.feature_compare(embedding, com_face["feature"], self.threshold)
            if r:
                is_exits = True
        if is_exits:
            return '该用户已存在'
        # 符合注册条件保存图片，同时把特征添加到人脸特征库中
        cv2.imencode('.png', image)[1].tofile(os.path.join("/home/linxi/ServiceRobot-General/src/pose_estimate/face_db", '%s.png' % user_name))
        self.faces_embedding.append({
            "user_name": user_name,
            "feature": embedding
        })
        return "success"

    # 检测人脸
    def detect(self, image):
        faces = self.model.get(image)
        results = list()
        for face in faces:
            result = dict()
            # 获取人脸属性
            result["bbox"] = np.array(face.bbox).astype(np.int32).tolist()
            result["kps"] = np.array(face.kps).astype(np.int32).tolist()
            # result["landmark_3d_68"] = np.array(face.landmark_3d_68).astype(np.int32).tolist()
            # result["landmark_2d_106"] = np.array(face.landmark_2d_106).astype(np.int32).tolist()
            # result["pose"] = np.array(face.pose).astype(np.int32).tolist()
            # result["age"] = face.age
            gender = '男'
            if face.gender == 0:
                gender = '女'
            result["gender"] = gender
            # 开始人脸识别
            embedding = np.array(face.embedding).reshape((1, -1))
            embedding = preprocessing.normalize(embedding)
            result["embedding"] = embedding
            results.append(result)
        return results

def draw_main_circle(face_feat:list,img:cv2.Mat):
    for index,data in enumerate(face_feat):
        pos_x=data[0]
        pos_y=data[1]
        cv2.circle(img,(pos_x,pos_y),2,(0,255,0),thickness=2)
if __name__ == '__main__':
    begin=time.time()
    img = cv2.imdecode(np.fromfile('t2.jpg', dtype=np.uint8), -1)
    face_recognitio = FaceRecognition()
    # 人脸注册
    # result = face_recognitio.register(img, user_name='t1')
    # print(result)

    # 人脸识别
    results = face_recognitio.recognition(img)
    for result in results:
        print("识别结果：{}".format(result))

    # results = face_recognitio.detect(img)
    # for result in results:
    #     print('人脸框坐标：{}'.format(result["bbox"]))
    #     print('人脸五个关键点：{}'.format(result["kps"]))
    #     # print('人脸3D关键点：{}'.format(result["landmark_3d_68"]))
    #     # print('人脸2D关键点：{}'.format(result["landmark_2d_106"]))
    #     # print('人脸姿态：{}'.format(result["pose"]))
    #     # print('年龄：{}'.format(result["age"]))
    #     # print('性别：{}'.format(result["gender"]))
    # now_img=cv2.imread("t1.jpg")
    # pt=result["bbox"]

    # print(pt)
    # # print(result["pose"])
    # cv2.rectangle(now_img,(pt[0],pt[1]),(int(pt[2]),int(pt[3])),(0,0,0),thickness=2)
    # # draw_main_circle(result["landmark_3d_68"],now_img)
    # cv2.imshow("show",now_img)
    print(time.time()-begin)
    # cv2.waitKey(50)
    cv2.waitKey(50)
    time.sleep(5)