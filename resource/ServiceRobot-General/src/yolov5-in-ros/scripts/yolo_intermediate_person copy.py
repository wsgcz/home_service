#! /home/linxi/anaconda3/envs/ros/bin/python

"""yolo中间件样例，注意发布的话题的类型必须是Image，或者你自己改"""

"""
注意，这个是YOLOV5S的ROS中间件libyolo.py的使用教程
libyolo.py会通过yolo处理图像并返回Python的字典
使用json等库将结果转化为ROS的标准消息以实现自定义消息或直接使用
注意，libyolo.py默认会去功能包下的model寻找指定名称为yolov5s.pt的参数文件
如果没有指定参数文件路径的情况下不存在该文件则会报错
"""

# 使用json来简单的处理为std_msgs中的String，其他类型数据自行处理


from libyolo import YoloV5s
from general_service_2022.msg import YoloResult
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import rospy
from os import read
import sys
import json
import cv2
for i in range(len(sys.path)):
    if "2.7" in sys.path[i]:
        sys.path.append(sys.path.pop(i))
        break


# 接口


# 在类中回调函数调用作为属性的pub来发布消息
class YoloNode:
    yolo = YoloV5s(weight_path=None)
    bridge = CvBridge()
    image_and_yolo_result_message = YoloResult()

    def __init__(self, raw_image_topic: str, result_pub_topic: str, quene_size: int) -> None:
        self.image_and_yolo_result_publisher = rospy.Publisher(
            name="general_service_image_and_yolo_result", data_class=YoloResult, queue_size=quene_size)
        self.subscriber = rospy.Subscriber(
            name=raw_image_topic,
            data_class=Image,
            queue_size=quene_size,
            callback=self.yolo_process
        )

    def yolo_process(self, msg: Image):
        img: np.ndarray = self.bridge.imgmsg_to_cv2(img_msg=msg)
        result, drawed_img = self.yolo.inference(img=img, if_show=True)
        s = json.dumps(result)
        rospy.loginfo(f"After json dumps")
        self.image_and_yolo_result_message.image = msg
        self.image_and_yolo_result_message.result = String(s)
        self.image_and_yolo_result_message.header.frame_id = 'yolo'
        self.image_and_yolo_result_message.header.stamp = rospy.Time.now()
        self.image_and_yolo_result_publisher.publish(
            self.image_and_yolo_result_message)


if __name__ == "__main__":
    rospy.init_node("yolo_intermediate_person")
    yn = YoloNode(raw_image_topic="/kinect2/qhd/image_color_rect",
                  result_pub_topic="yolo_result_person", quene_size=100)
    rospy.spin()
