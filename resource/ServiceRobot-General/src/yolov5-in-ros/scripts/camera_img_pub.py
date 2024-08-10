#! /home/linxi/anaconda3/envs/ros/bin/python3

"""图像发布方样例，注意发布的话题的类型必须是Image"""

import sys

for i in range(len(sys.path)):
    if "2.7" in sys.path[i]:
        sys.path.append(sys.path.pop(i))
        break

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MyImagePublisher(object):
    cap = cv2.VideoCapture(0)
    publisher =  rospy.Publisher(name="raw_image", data_class=Image, queue_size=100)
    bridge = CvBridge()
    def __init__(self) -> None:
        super().__init__()


    def read_frame(self) -> np.ndarray:
        if not  (results:=self.cap.read())[0]:
            rospy.logfatal("read fail")
        return results[1]

    def pub_once(self, img) -> bool:
        msg: Image = self.bridge.cv2_to_imgmsg(cvim=img)
        rospy.loginfo("pub once")
        self.publisher.publish(msg)



if __name__ == "__main__":
    rospy.init_node(name="image_pub")
    mip = MyImagePublisher()

    # ros 会开多进程跑你的Code，如果你的Code有时间比较长的代码，那么你的代码所处的进程不会响应退出
    # 如果在rater.sleep 的时候ctrl+c则ros本身会安全的退出所有Python的进程
    # 如果是在自己的线程运行的时候则Python没有办法杀死所有的线程，导致终端卡死
    # 解决办法就是自己的代码尽量不要写长的循环
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        img = mip.read_frame()
        mip.pub_once(img=img)
        r.sleep()