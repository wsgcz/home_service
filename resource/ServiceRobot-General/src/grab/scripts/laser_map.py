#! /usr/bin/env python3
from itertools import count
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
def domsg(msg:String):
    # rospy.loginfo("11111111111111")
    now_time1=rospy.Time.now().to_sec()
    now_time2=rospy.Time.now().to_nsec()
    now_time=rospy.Time.now()
    frame_id=msg.header.frame_id
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    angle_increment=msg.angle_increment
    time_increment=msg.time_increment
    scan_time=msg.scan_time
    the_range=msg.ranges
    length=len(the_range)
    avaerage_range_list=[]
    max_count=0
    begin_pos=0
    end_pos=0
    count=0
    temp_begin_pos=0
    temp_end_pos=0
    
    for i in range(length):
        
        if the_range[i]==math.inf:
            count+=1
        else:
            temp_end_pos=i
            if count>max_count:
                begin_pos=temp_begin_pos
                end_pos=i
                max_count=count
            temp_begin_pos=i    
            count=0
    depth_index=(temp_begin_pos+temp_end_pos)%2
    rospy.loginfo("this is index %d" ,depth_index)
    # rospy.loginfo("this is lenght %d",length)
    data=String()
    data.data=str(depth_index)   
    pub.publish(data)
if __name__=="__main__":
    rospy.init_node("laser_of_map")
    rospy.loginfo("1111111111111")
    # transformListener=tf2_ros.transform_listener()
    sub=rospy.Subscriber("/scan",LaserScan,domsg)
    pub=rospy.Publisher("general_service_laser",String,queue_size=10)
    rospy.spin()

