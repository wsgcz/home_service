#! /usr/bin/env python3


import rospy
from std_msgs.msg import String

def dothing(msg):
    shuju=""
    shuju=str(msg.data)
    if len(shuju)>9:
        cleft=shuju.find("<confidence>")
        cright=shuju.find("</confidence>")
        pipei=shuju[cleft+12:cright]
        rospy.loginfo("%s",pipei)
        pipei=int(pipei)
        if (pipei>=5):
            shuju=""
            shuju=str(msg.data)
            start_msg.data=""
            left=shuju.find("<rawtext>")
            right=shuju.find("</rawtext>")
            shuju=shuju[left+9:right]
            rospy.loginfo("%s",shuju)
            start1_msg=String()
            start1_msg.data="0"
            if "startfollow"==shuju:
                start1_msg.data="1"
            elif "stopfollow" in shuju:
                start1_msg.data="2"
            elif "oreo" in shuju:
                start1_msg.data="3"
            elif "potatowish" in shuju :
                start1_msg.data="4"
            elif "lays" in shuju:
                start1_msg.data="5"
            elif "cookie" in shuju:
                start1_msg.data="6"
            elif "sprite" in shuju:
                start1_msg.data="7"
            elif "cola" in shuju:
                start1_msg.data="8"
            elif "orangejuice"in shuju:
                start1_msg.data="9"
            elif "water" in shuju:
                start1_msg.data="a"
            elif "milk" in shuju:
                start1_msg.data="b"
            elif "handwash" in shuju:
                start1_msg.data="c"
            elif "dishsoap" in shuju:
                start1_msg.data="d"
            elif "shampoo" in shuju:
                start1_msg.data="e"
            zhi=str(start1_msg.data)
            pub2.publish(shuju)
            rospy.loginfo("%s",zhi)
            start_msg.data="好的收到指令"
            if "好的收到指令"!=shuju:
                pub.publish(start_msg)
if __name__=="__main__":
    rospy.init_node("sd0ads")
    sub=rospy.Subscriber("trans",String,dothing,queue_size=10)
    pub=rospy.Publisher("/xfsaywords",String,queue_size=10)
    pub2=rospy.Publisher("/listen_result",String,queue_size=10)
    start_msg=String()
    rospy.spin()