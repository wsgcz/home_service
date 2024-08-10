#! /home/linxi/anaconda3/envs/ros/bin/python
import cv2
import insightface
import numpy as np
from sklearn import preprocessing
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from math import pi
import json
import sys
sys.path.append('/home/linxi/ServiceRobot-General/src/yolov5-in-ros/scripts')
from libyolo import YoloV5s
for i in range(len(sys.path)):
    if "2.7" in sys.path[i]:
        sys.path.append(sys.path.pop(i))
        break
# import threadpool
# import threading
# model = insightface.app.FaceAnalysis(root='./', allowed_modules=None, providers=['CPUExecutionProvider']) #CUDAExecutionProvider'换为gpu
# model.prepare(ctx_id=gpu_id, det_thresh=det_thresh, det_size=det_size)
# 人脸库的人脸特征

face_detection_message = 'off'
bridge = CvBridge()
yolo = YoloV5s(weight_path=None)
# @staticmethod


def feature_compare(feature1, feature2, threshold):  # 采集的人脸特征与预存特征比较
    diff = np.subtract(feature1, feature2)
    dist = np.sum(np.square(diff), 1)  # 欧几里德距离刻画特征向量相似性
    if dist < threshold:  # 距离小于threshold我们认为是一个人
        return True
    else:
        return False

# faces_embedding = list() #预存好的人脸，比赛中为a,b,c三个人所有人脸信息

false_count = 0
def register(image, user_name, faces_embedding):  # 传入abc的名字以及图片
    global false_count
    # faces_embedding = list()
    gpu_id = 0
    face_db = 'face_db'  # 存abc人脸数据的文件夹
    threshold = 1.24
    det_thresh = 0.5
    det_size = (640, 640)
    cv2.imwrite('/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/cache/register' + str(false_count) + '.jpg', image)
    false_count += 1
    model = insightface.app.FaceAnalysis(root='./', allowed_modules=None, providers=[
                                         'CUDAExecutionProvider'])  # CUDAExecutionProvider'换为gpu
    model.prepare(ctx_id=gpu_id, det_thresh=det_thresh, det_size=det_size)

    faces = model.get(image)  # 读取人脸
    if len(faces) != 1:  # 注册时只能读一张人脸
        return '图片检测不到人脸或人脸数目不匹配'
    # 判断人脸是否存在
    handler = insightface.model_zoo.get_model(
        '/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/recg.onnx', providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    handler.prepare(ctx_id=0)
    # img = ins_get_image('t1')
    feature = handler.get(image, faces[0])
    # print("size of feature:", len(feature))
    # print("feature:", feature)
    feature = np.array(feature).reshape((1, -1))
    feature = preprocessing.normalize(feature)

    is_exits = False
    for com_face in faces_embedding:  # 先判定是不是已经存过了
        r = feature_compare(feature, com_face["feature"], threshold)
        if r:
            is_exits = True
    if is_exits:
        return 'exist'
    # 符合注册条件保存图片，同时把特征添加到人脸特征库中
    # cv2.imencode('.png', image)[1].tofile(os.path.join(face_db, '%s.png' % user_name))
    faces_embedding.append({
        "user_name": user_name,
        "feature": feature
    })
    return 'success'


# 加载人脸库中的人脸
# load_faces(face_db)
recognition_count = 0
def recognition(image, faces_embedding):  # 传入图片与预存好的人脸库
    global recognition_count
    gpu_id = 0
    face_db = 'face_db'  # 存abc人脸数据的文件夹
    threshold = 1.24
    det_thresh = 0.5
    det_size = (640, 640)
    cv2.imwrite('/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/cache/recognition' + str(recognition_count) + '.jpg', image)
    recognition_count += 1
    model = insightface.app.FaceAnalysis(root='/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/', allowed_modules=None, providers=[
                                         'CUDAExecutionProvider'])  # CUDAExecutionProvider'换为gpu
    model.prepare(ctx_id=gpu_id, det_thresh=det_thresh, det_size=det_size)

    faces = model.get(image)  # 使用model寻找人脸，仍然假设是一张人脸
    if len(faces) != 1:
        return 'unknown'
    # results = list()
    handler = insightface.model_zoo.get_model(
        '/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/recg.onnx', providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    handler.prepare(ctx_id=0)
    feature = handler.get(image, faces[0])
    feature = np.array(feature).reshape((1, -1))
    feature = preprocessing.normalize(feature)
    user_name = "unknown"
    for com_face in faces_embedding:
        r = feature_compare(feature, com_face["feature"], threshold)
        if r:
            user_name = com_face["user_name"]
    return user_name


def stop():
    vel_cmd = Twist()
    vel_cmd.linear.x = 0
    vel_cmd.linear.y = 0
    vel_cmd.linear.z = 0
    vel_cmd.angular.x = 0
    vel_cmd.angular.y = 0
    vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
# 负数左转，正数右转


def rotate(angle):
    vel_cmd = Twist()
    vel_cmd.angular.z = -angle*pi/180
    vel_pub.publish(vel_cmd)


def face_detection_call_back(msg: String):
    global face_detection_message
    face_detection_message = msg.data
    rospy.loginfo('face: 收到 %s', face_detection_message)

def image_call_back(msg: Image):
    global bridge, faces_embedding, yolo, face_detection_message
    state = ''
    if face_detection_message == 'off':
        return
    elif face_detection_message[0] == 'd':
        stop()

        name = 'null'
        image = bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

        if face_detection_message == 'detection1':
            name = 'Alice'
        elif face_detection_message == 'detection2':
            name = 'Bob'
        elif face_detection_message == 'detection3':
            name = 'Carol'
        yolo_result_message, _ = yolo.inference(img=image, if_show=True)
        if 'person' in yolo_result_message:
            persons = []
            yolo_result_dict = yolo_result_message

            for person in yolo_result_dict['person']:
                persons.append(person)
            persons_length = len(persons)

            if persons_length == 1:
                state = 'found 1'
                width = persons[0][2]
                height = persons[0][3]
                x_mid = persons[0][0]
                y_mid = persons[0][1]
                img = image[int(y_mid - 0.5 * height): int(y_mid - 0.5 * height) + int(
                    height), int(x_mid - 0.5 * width): int(x_mid - 0.5 * width) + int(width)]
                state = register(img, name, faces_embedding)
            else:
                state = 'found more than 1'
                areas = []
                for person in persons:
                    areas.append(person[2] * person[3])
                areas.append(-1)
                target_index = areas.index(max(areas))
                width = persons[target_index][2]
                height = persons[target_index][3]
                x_mid = persons[target_index][0]
                y_mid = persons[target_index][1]
                img = image[int(y_mid - 0.5 * height): int(y_mid - 0.5 * height) + int(
                    height), int(x_mid - 0.5 * width): int(x_mid - 0.5 * width) + int(width)]
                state = register(img, name, faces_embedding)
            message = String()
            message.data = state
            register_publisher.publish(message)
            face_detection_message = 'off'
            
                
                
    elif face_detection_message[0] == 'r':
        stop()

        name = 'null'
        image = bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        yolo_result_message, _ = yolo.inference(img=image, if_show=True)
        if 'person' in yolo_result_message:
            persons = []
            yolo_result_dict = yolo_result_message

            for person in yolo_result_dict['person']:
                persons.append(person)
            persons_length = len(persons)

            if persons_length == 1:
                state = 'found 1'
                width = persons[0][2]
                height = persons[0][3]
                x_mid = persons[0][0]
                y_mid = persons[0][1]
                img = image[int(y_mid - 0.5 * height): int(y_mid - 0.5 * height) + int(
                    height), int(x_mid - 0.5 * width): int(x_mid - 0.5 * width) + int(width)]
                name = recognition(img, faces_embedding)
            else:
                state = 'found more than 1'
                areas = []
                for person in persons:
                    areas.append(person[2] * person[3])
                areas.append(-1)
                target_index = areas.index(max(areas))
                width = persons[target_index][2]
                height = persons[target_index][3]
                x_mid = persons[target_index][0]
                y_mid = persons[target_index][1]
                img = image[int(y_mid - 0.5 * height): int(y_mid - 0.5 * height) + int(
                    height), int(x_mid - 0.5 * width): int(x_mid - 0.5 * width) + int(width)]
                name = recognition(img, faces_embedding)
            message = String()
            message.data = name
            recognition_publisher.publish(message)
            rospy.loginfo('face: 识别结果: %s', name)
            face_detection_message = 'off'
    else:
        state = face_detection_message + ' unsuccess'
    rospy.loginfo('face: %s', state)


if __name__ == '__main__':
    rospy.init_node(name='test_face', anonymous=True)
    face_detection_subscriber = rospy.Subscriber(
        name='general_service_face_detection', data_class=String, queue_size=10, callback=face_detection_call_back)
    image_subscriber = rospy.Subscriber(
        name='/kinect2/qhd/image_color_rect', data_class=Image, queue_size=10, callback=image_call_back
    )
    register_publisher = rospy.Publisher(name='general_service_register_face', data_class=String, queue_size=10)
    recognition_publisher = rospy.Publisher(
        name='general_service_recognition', data_class=String, queue_size=10)
    vel_pub = rospy.Publisher(name='/cmd_vel', data_class=Twist, queue_size=10)
    faces_embedding = list()
    rospy.spin()
