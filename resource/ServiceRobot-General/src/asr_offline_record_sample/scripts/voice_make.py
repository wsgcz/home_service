#! usr/bin/env python3
import pyttsx3
import rospy
from std_msgs.msg import String
def callback(msg):
    engine = pyttsx3.init()
    shuju=""
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate-40)
    shuju=str(msg.data)
    engine.say(shuju)
    engine.runAndWait()
if __name__=="__main__":
    rospy.init_node("voice_listen")
    sub=rospy.Subscriber("/xfsaywords",String,callback,queue_size=10)
    rospy.spin()