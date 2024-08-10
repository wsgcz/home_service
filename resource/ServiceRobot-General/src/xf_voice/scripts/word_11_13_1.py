#! /usr/bin/env python3


import rospy
from std_msgs.msg import String
cunchu=''
def dothing(msg):
    global cunchu
    shuju=""
    shuju=str(msg.data)
    start_msg.data="ok, i know you want "
    if len(shuju)>9:
        cleft=shuju.find("<confidence>")
        cright=shuju.find("</confidence>")
        pipei=shuju[cleft+12:cright]
        rospy.loginfo("%s",pipei)
        pipei=int(pipei)
        if (pipei>=0):
            shuju=""
            shuju=str(msg.data)
            if (shuju=="yes")
                
            left=shuju.find("<rawtext>")
            right=shuju.find("</rawtext>")
            shuju=shuju[left+9:right]
            cunchu=shuju
            rospy.loginfo("%s",shuju)
            start_msg.data=start_msg.data+shuju+" please check it"
            pub.publish(start_msg)
            return_msg=String()
            return_msg.data=shuju
            pub2.publish(return_msg)
if __name__=="__main__":
    rospy.init_node("sd0ads")
    sub=rospy.Subscriber("general_sevice_trans",String,dothing,queue_size=10)
    pub=rospy.Publisher("/general_service_xfsaywords",String,queue_size=10)
    pub2=rospy.Publisher("general_service_object_name_return",String,queue_size=10)
    start_msg=String()
    rospy.spin()