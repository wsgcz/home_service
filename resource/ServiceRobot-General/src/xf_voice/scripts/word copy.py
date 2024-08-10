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
            left=shuju.find("<rawtext>")
            right=shuju.find("</rawtext>")
            shuju=shuju[left+9:right]
            # if shuju in ['dining-hall' ,'livingroom']:
            #     pub.publish("zero")
            if shuju=="yes" and cunchu!='':
                return_msg=String()
                return_msg.data=cunchu
                canshu=""
                pub2.publish(return_msg)
            elif (shuju=="no"):
                the_ans=String()
                the_ans.data="retry"
                pub.publish(the_ans)
            else:
                cunchu=shuju
                rospy.loginfo("%s",shuju)
                pub3.publish(String("yes"))
                rospy.Rate(5).sleep()
                start_msg.data=start_msg.data+shuju+" please check it"
                pub.publish(start_msg)
                return_msg=String()
                return_msg.data=shuju
                # pub2.publish(return_msg)
if __name__=="__main__":
    rospy.init_node("sd0ads")
    sub=rospy.Subscriber("general_sevice_trans",String,dothing,queue_size=10)
    pub=rospy.Publisher("/general_service_xfsaywords",String,queue_size=10)
    pub2=rospy.Publisher("general_service_object_name_return",String,queue_size=10)
    pub3=rospy.Publisher('general_service_judge_the_correct',String,queue_size=10)
    start_msg=String()
    rospy.spin()