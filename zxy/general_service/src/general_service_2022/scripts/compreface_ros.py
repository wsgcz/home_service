#!/home/riyuu/anaconda3/envs/daily/bin/python

from compreface import CompreFace
from compreface.service import RecognitionService
from compreface.collections import FaceCollection
from compreface.collections.face_collections import Subjects
import cv2
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import os

# 此节点为 compreface 与 ros 之间的中介节点，负责上传人脸信息，分类人脸，判断面前是否为特定客人
DOMAIN: str = 'http://localhost'
PORT: str = '8000'
API_KEY: str = '8b6a0f6d-c15e-4b69-9a4a-b725c82d12b0'
users = ['alice', 'bob', 'carol']


class Compreface:
    bridge = CvBridge()

    def __init__(self, domain: str, port: str) -> None:
        self.compre_face: CompreFace = CompreFace(domain, port)
        self.recognition: RecognitionService = self.compre_face.init_face_recognition(
            API_KEY)
        self.face_collection: FaceCollection = self.recognition.get_face_collection()
        self.subjects: Subjects = self.recognition.get_subjects()
        self.upload_image_state = 'close'
        self.current_person_index = 0
        self.upload_image_state_subscriber = rospy.Subscriber(
            name='upload_image_state', data_class=String, queue_size=100, callback=self.upload_image_state_call_back)

    def upload_image_state_call_back(self, upload_image_state: String) -> None:
        switch = upload_image_state.data
        if switch == '0':
            t0 = threading.Thread(target=self.upload, args=(self, switch))
            t0.start()
        elif switch == '1':
            t1 = threading.Thread(target=self.upload, args=(self, switch))
            t1.start()
        elif switch == '2':
            t2 = threading.Thread(target=self.upload, args=(self, switch))
            t2.start()
        else:
            pass

    def upload(self, user_id: str) -> None:
        subject: str = users[int(user_id)]
        rospy.loginfo('开始上传 %s 的人脸信息', subject)
        image_names = os.listdir(
            '/home/riyuu/ServiceRobot-General/src/general_service_2022/scripts/data/')
        matches = []
        image_path: str = 'data/User.' + user_id + '.'
        for image in image_names:
            if image.startswith(image_path):
                matches.append(image)
        for count in range(0, len(matches)):
            self.face_collection.add(image_path=(
                image_path + str(count) + '.jpg'), subject=subject)
        rospy.loginfo('%s 的人脸信息采集完成', subject)


if __name__ == "__main__":
    rospy.init_node('general_service_compreface_ros')
    solution = CompreFace(DOMAIN, PORT)
    rospy.spin()
