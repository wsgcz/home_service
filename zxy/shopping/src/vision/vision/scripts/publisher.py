import rospy
from std_msgs.msg import Bool, Float64MultiArray

if __name__=='__main__':

    rospy.init_node(name='publisher')

    vision_pub = rospy.Publisher(
        name='detect_switch',
        data_class=Bool,
        queue_size=10
    )
    arm_pub = rospy.Publisher(
        name='grab_data',
        data_class=Float64MultiArray,
        queue_size=10
    )

    data = Bool()
    data.data = True

    arm = Float64MultiArray()
    arm.data = [1, 0.2]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        vision_pub.publish(data)
        arm_pub.publish(arm)

        rate.sleep()