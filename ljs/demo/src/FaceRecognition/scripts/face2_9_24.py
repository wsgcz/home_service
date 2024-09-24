import os
import cv2
import numpy as np
import insightface
from sklearn import preprocessing

class FaceRecognition:
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

def main():
    # 修改人脸数据库路径
    face_db_path = "/home/shanhe/face_db"
    face_recognition = FaceRecognition(face_db=face_db_path)

    while True:
        print("\n1. 注册人脸")
        print("2. 识别人脸")
        print("3. 退出")
        choice = input("请选择操作: ")

        if choice == '1':
            image_path = input("请输入图片路径: ")
            user_name = input("请输入用户名: ")
            image = cv2.imread(image_path)
            result = face_recognition.register(image, user_name)
            print(f"注册结果: {result}")

        elif choice == '2':
            image_path = input("请输入图片路径: ")
            image = cv2.imread(image_path)
            results = face_recognition.recognition(image)
            print("识别结果:")
            for result in results:
                print(result)

        elif choice == '3':
            break

        else:
            print("无效选择，请重新输入")

if __name__ == "__main__":
    # 注：用于注册的人脸一张图片只允许包含一个
    main()

""" 
测试数据：
% 要添加的人脸：
/home/shanhe/face_to_putin/shanhe.jpg % 1张人脸ljs
/home/shanhe/face_to_putin/lyb.jpg % 1张人脸lyb
/home/shanhe/face_to_putin/none.jpg % 没有人脸
/home/shanhe/face_to_putin/many_1.jpg %2张人脸,一个ljs一个lyb
% 要识别的人脸：
/home/shanhe/faces_to_read/1.jpg % 1张人脸ljs
/home/shanhe/faces_to_read/2.jpg % 1张人脸ljs
/home/shanhe/faces_to_read/many_1.jpg %2张人脸,一个ljs一个lyb
/home/shanhe/faces_to_read/many_2.jpg %2张人脸,一个ljs一个unknown
/home/shanhe/faces_to_read/many_3.jpg %4张人脸,一个ljs其余unknown

"""
