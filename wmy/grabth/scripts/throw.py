#! /home/wmy/anaconda3/envs/grabb/bin/python
import rospy
from std_msgs.msg import String,Float32
from sensor_msgs.msg import JointState
import time
from geometry_msgs.msg import Twist

#参数
control_arm_data=JointState()
arm_postion_init=[0,0]
control_arm_data.name.append("lift")
control_arm_data.name.append("gripper")
control_arm_data.position.extend(arm_postion_init)
control_arm_data.velocity.extend(arm_postion_init)

machine_speed=Twist()


def start_spin(msg):
    # global begin_throw
    if (msg.data=="throw"):
        start_spin_pub.publish("spin")
        rospy.loginfo("------------------------------Throw------------------------")

def start_forward(msg):
    # global begin_throw
    if (msg.data=="spin ok"):
        start_get_distance_pub.publish("true")
    rospy.loginfo("ok I will get the distance")

def go_ahead(msg):
    global machine_speed
    distance = msg.data
    if (distance < 0.60 and distance > 0.50):
        throw_the_object()
    else:
        machine_speed.linear.x=(distance-0.55)/2 # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向
        machine_speed.linear.y=0
        machine_speed.linear.z=0
        machine_speed.angular.x=0
        machine_speed.angular.y=0
        machine_speed.angular.z=0
        rospy.loginfo("continue move to the destination~")
        speed_of_pub.publish(machine_speed)
        rospy.sleep(2)
        rospy.loginfo("arrive!!!")
        machine_speed.linear.x=0
        speed_of_pub.publish(machine_speed)
        rospy.sleep(1)
        start_get_distance_pub.publish("true")


def throw_the_object():
    global control_arm_data
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[0]=0.58
    control_arm_data.velocity[1]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    control_arm_data.position[1]=0.3
    control_arm_data.velocity[1]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    control_arm_data.position[0]= 0.5 #下降
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(4)
    rospy.loginfo("finish throw the trash!!!")
    down_state_pub.publish("0")

#主体
if __name__=="__main__":
    rospy.init_node("general_service_throw")

    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10) #控制机器人线速度和角速度
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    
    start_into_throw_sub=rospy.Subscriber("robot_state",Float32,start_spin,queue_size=10) #throw
    down_state_pub=rospy.Publisher("genenal_service_put_down_result",String,queue_size=10)#grab.py
    
    start_spin_pub=rospy.Publisher("/home_service/robot_spin",String,queue_size=10)#begin spin
    start_forward_sub=rospy.Subscriber("/home_service/robot_spin_reply",String,start_forward,queue_size=10)#spin ok
    
    start_get_distance_pub=rospy.Publisher("/home_service/lidar_distance",String,queue_size=10) #begin to get distance
    get_distance_sub=rospy.Subscriber("/home_service/robot_getdistance",String,go_ahead,queue_size=10)#get distance and move

    rospy.spin()



























#def grab_room_cb(msg:String):
   # global grab_room
   # grab_room=msg.data








