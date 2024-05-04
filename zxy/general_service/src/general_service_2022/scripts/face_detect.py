#!/home/riyuu/anaconda3/envs/daily/bin/python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

# 此节点为人脸检测节点，作用是采集人脸信息
# 在我自己电脑上测试 18s 能记录 500 张人脸，可以根据实际改进记录张数


class FaceDetect:
    bridge = CvBridge()

    def __init__(self) -> None:
        self.speech_recognition_state = 'close'
        self.current_person_index = 0
        self.face_detector = cv2.CascadeClassifier(
            '/home/riyuu/ServiceRobot-General/src/general_service_2022/scripts/haarcascade/haarcascade_frontalface_default.xml'
        )  # 待更改，需要绝对路径
        self.count = 0
        self.yolo_result_subscriber = rospy.Subscriber(
            name='/kinect2/qhd/image_color_rect', data_class=Image, queue_size=100, callback=self.face_call_back)
        self.speech_recognition_state_subscriber = rospy.Subscriber(
            name='speech_recognition_state', data_class=String, queue_size=100, callback=self.speech_recognition_state_call_back)

    def speech_recognition_state_call_back(self, speech_recognition_state: String) -> None:
        if speech_recognition_state.data[0] == 'o':
            self.speech_recognition_state = 'open'
        else:
            self.speech_recognition_state = 'close'
            self.count = 0
        self.current_person_index = speech_recognition_state.data[-1]

    def face_call_back(self, message: Image) -> None:
        if self.speech_recognition_state == 'open':
            image: np.ndarray = self.bridge.imgmsg_to_cv2(img_msg=message)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.face_detector.detectMultiScale(gray, 1.3, 5)
            # 框选人脸，for循环保证一个能检测的实时动态视频流
            for (x, y, w, h) in faces:
                # xy为左上角的坐标,w为宽，h为高，用rectangle为人脸标记画框
                cv2.rectangle(image, (x, y), (x+w, y+w), (255, 0, 0))
                # 成功框选则样本数增加
                self.count += 1
                rospy.loginfo('当前正在采集 %s 号，第 %d 张', str(
                    self.current_person_index), self.count)
                # 保存图像，把灰度图片看成二维数组来检测人脸区域
                # (这里是建立了data的文件夹，当然也可以设置为其他路径或者调用数据库)
                cv2.imwrite("/home/riyuu/ServiceRobot-General/src/general_service_2022/scripts/data/User." +
                            str(self.current_person_index) + '.' + str(self.count) + '.jpg', gray[y:y+h, x:x+w])
                # 显示图片
                cv2.imshow('image', image)
                # 保持画面的连续。waitkey方法可以绑定按键保证画面的收放，通过q键退出摄像
                # 或者得到800个样本后退出摄像，这里可以根据实际情况修改数据量，实际测试后800张的效果是比较理想的
        else:
            if self.current_person_index != 0:
                rospy.loginfo('检测完 %d 的人脸信息', self.current_person_index - 1)
            self.count = 0
            cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('general_service_face_detect', anonymous=True)
    solution = FaceDetect()
    rospy.spin()
