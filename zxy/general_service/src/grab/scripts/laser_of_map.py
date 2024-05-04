import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
def domsg(msg):
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
    for i in range(length):
        rospy.loginfo("this is the range %f",i)
        number_1=(i+0)%360
        number_2=(i+1)%360
        number_3=(i+2)%360
        number_4=(i+3)%360
        number_5=(i+4)%360
        avaerage_range=(the_range[number_1]+the_range[number_2]+the_range[number_3]+the_range[number_4]+the_range[number_5])/5
        avaerage_range_list.append(avaerage_range)
    max_l=0
    depth_index=0
    for index,i in enumerate(avaerage_range_list):
        if max_l<i:
            max_l=i
            depth_index=index
    index=(index+2)%360
    data=String()
    data.data=str(index)   
    pub.publish(data)
if __name__=="__main__":
    rospy.init_node("laser_of_map")

    # transformListener=tf2_ros.transform_listener()
    sub=rospy.Subscriber("/scan",LaserScan,domsg)
    pub=rospy.Publisher("general_service_laser",String,queue_size=10)
    rospy.spin()

