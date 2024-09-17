import os
import cv2
import numpy as np
import insightface
from sklearn import preprocessing

class FaceRecognition:
    def __init__(self, gpu_id=0, face_db='/home/shanhe/face_db', threshold=1.24, det_thresh=0.50, det_size=(640, 640)):
        self.gpu_id = gpu_id
        self.face_db = face_db
        self.threshold = threshold
        self.det_thresh = det_thresh
        self.det_size = det_size
        
        self.model = insightface.app.FaceAnalysis(
            allowed_modules=['detection', 'recognition'],
            providers=['CUDAExecutionProvider']
        )
        self.model.prepare(ctx_id=self.gpu_id, det_thresh=self.det_thresh, det_size=self.det_size)
        
        self.faces_embedding = []
        self.load_faces()
        
    def load_faces(self):
        if not os.path.exists(self.face_db):
            os.makedirs(self.face_db)
        for root, dirs, files in os.walk(self.face_db):
            for file in files:
                input_image = cv2.imread(os.path.join(root, file))
                user_name = file.split(".")[0]
                faces = self.model.get(input_image)
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

    def recognition(self, image):
        faces = self.model.get(image)
        results = []
        for face in faces:
            embedding = np.array(face.embedding).reshape((1, -1))
            embedding = preprocessing.normalize(embedding)
            user_name = "未知"
            for com_face in self.faces_embedding:
                if self.feature_compare(embedding, com_face["feature"], self.threshold):
                    user_name = com_face["user_name"]
                    break
            results.append(user_name)
        return results

    @staticmethod
    def feature_compare(feature1, feature2, threshold):
        diff = np.subtract(feature1, feature2)
        dist = np.sum(np.square(diff), 1)
        return dist < threshold

def process_folder(face_recognizer, folder_path):
    if not os.path.exists(folder_path):
        print(f"文件夹 {folder_path} 不存在")
        return

    for filename in os.listdir(folder_path):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            image_path = os.path.join(folder_path, filename)
            image = cv2.imread(image_path)
            if image is None:
                print(f"无法读取图片: {filename}")
                continue

            results = face_recognizer.recognition(image)
            
            if results:
                print(f"\n图片 {filename} 的识别结果:")
                for i, name in enumerate(results):
                    print(f"  人脸 {i+1}: {name}")
            else:
                print(f"\n图片 {filename} 中未检测到人脸")

def main():
    face_recognizer = FaceRecognition()
    
    # 输入要识别的图片文件夹路径
    folder_path = input("请输入要识别的图片文件夹路径: ")
    
    process_folder(face_recognizer, folder_path)

if __name__ == '__main__':
    main()
