#! /home/melodygreen/anaconda3/envs/ros/bin/python

import rospy
from sentence_voice.msg import Sentence
from std_msgs.msg import String


class Repeater(object):
    def __init__(self) -> None:
        super().__init__()
        self.pub = rospy.Publisher(name="sentence_to_speak", data_class=Sentence, queue_size=100)
        self.subs = rospy.Subscriber(name="audio_result", data_class=Sentence, queue_size=100, callback=self.receive)

    def receive(self, msg: Sentence):
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("imitator", anonymous=False)
    p = Repeater()
    rospy.spin()