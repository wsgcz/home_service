from general_service_2022.msg import the_way_out

import rospy

def domsg(msg):
    rospy.loginfo("11111111111111111111111")
    theta=msg.angle
    max_angle=msg.the_angle_of_max_length
    max_length=msg.max_length
    rospy.loginfo("%d,%d,%d",theta,max_angle,max_length)
rospy.init_node("test_of_sub_out")
sub=rospy.Subscriber("/general_service_pose_pub",the_way_out,domsg,queue_size=10)
rospy.spin()