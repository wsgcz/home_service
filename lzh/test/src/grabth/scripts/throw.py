#! /home/wmy/anaconda3/envs/grabb/bin/python
import rospy
from std_msgs.msg import String,Float32
from sensor_msgs.msg import JointState
import time
from geometry_msgs.msg import Twist

#参数
""" 机械臂控制 """
control_arm_data=JointState()#变量的类型
arm_postion_init=[0,0]#初始化
control_arm_data.name.append("lift")#[0]控制机械臂上下
control_arm_data.name.append("gripper")#[1]控制爪夹开和
control_arm_data.position.extend(arm_postion_init)#机械臂位置初始化
control_arm_data.velocity.extend(arm_postion_init)#速度初始化

distance_count = 0 #获取距离的次数
total_distance = 0#移动的总距离
machine_speed=Twist()#变量类型，控制机械臂速度

""" 发布开始旋转的消息 """
def start_spin(msg):
    # global begin_throw
    if (msg.data=="throw"):
        machine_speed.linear.x=-0.2 #退后，速度为0.2m/s（为了使摄像头看到目标）
        speed_of_pub.publish(machine_speed)#发布速度消息
        rospy.sleep(3)#退后三秒（含系统本身延迟）
        machine_speed.linear.x=0  #停下
        speed_of_pub.publish(machine_speed)
        speed_of_pub.publish(machine_speed)
        rospy.sleep(0.5)
        start_spin_pub.publish("spin") #发布消息给视觉，开始进行旋转
        rospy.loginfo("------------------------------Throw------------------------")

""" 发布消息获取节点数据 """
def start_forward(msg):
    # global begin_throw
    if (msg.data=="spin ok"):
        start_get_distance_pub.publish("true")#发布“true”,启动雷达测距
    rospy.loginfo("ok I will get the distance")

""" 前进到垃圾桶上方 """

def go_ahead(msg):
    global machine_speed, distance_count, total_distance
    distance = msg.data #移动距离为雷达所测距离
    if (((distance < 0.63 and distance > 0.53) or distance_count >= 7 or (total_distance + distance - 0.55) > 1.3 ) and distance > 0 and distance < 2): 
        throw_the_object() #距离正确/测距数过多/测距错误（测错目标物/距离为inf）则直接丢
    else:
        distance_count += 1 #测距次数累加
        machine_speed.linear.x=(distance-0.55)/3 # 在机器人坐标中，x为面前的方向；在摄像头中为左右的方向 #减去机械臂长度
        machine_speed.linear.y=0
        machine_speed.linear.z=0
        machine_speed.angular.x=0
        machine_speed.angular.y=0
        machine_speed.angular.z=0
        rospy.loginfo("continue move to the destination~")
        total_distance += distance-0.55 #计算总移动距离
        print(f"total_distance is {total_distance}")
        speed_of_pub.publish(machine_speed) #发布速度
        rospy.sleep(3)#移动三秒
        rospy.loginfo("arrive!!!")
        machine_speed.linear.x=0 
        speed_of_pub.publish(machine_speed)#停下
        rospy.sleep(0.5)
        speed_of_pub.publish(machine_speed)


        rospy.sleep(0.5) #发布两次确保停下
        start_get_distance_pub.publish("true")



""" 丢垃圾 """
def throw_the_object():
    global control_arm_data, total_distance
    #rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.position[0]=0.58 #上升到0.58米
    control_arm_data.velocity[0]=1#上升速度为1m/s
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(5)
    control_arm_data.position[1]=0.3#爪夹张开，间距30cm
    control_arm_data.velocity[1]=1#张开速度为1m/s
    arm_action_pub.publish(control_arm_data)
    arm_action_pub.publish(control_arm_data)
    rospy.sleep(6)
    control_arm_data.position[0]= 0 #下降至初始状态
    # rospy.loginfo("this is final_y %f",final_position_y)
    control_arm_data.velocity[0]=1
    arm_action_pub.publish(control_arm_data)
    arm_action_pub.publish(control_arm_data)
    rospy.loginfo("finish throw the trash!!!")
    total_distance = 0 
    down_state_pub.publish("0") #丢垃圾完成，给主函数返回0

#主函数
if __name__=="__main__":
    rospy.init_node("general_service_throw") #节点初始化

    speed_of_pub=rospy.Publisher('cmd_vel',Twist,queue_size=10) #控制机器人线速度和角速度
    arm_action_pub=rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30) #控制机器人机械臂
    
    start_into_throw_sub=rospy.Subscriber("robot_state",String,start_spin,queue_size=10) #进入throw阶段
    down_state_pub=rospy.Publisher("genenal_service_put_down_result",String,queue_size=10)#grab.py丢弃完成
    
    start_spin_pub=rospy.Publisher("robot_spin",String,queue_size=10)#begin spin
    start_forward_sub=rospy.Subscriber("robot_spin_reply",String,start_forward,queue_size=10)#spin ok
    
    start_get_distance_pub=rospy.Publisher("/home_service/lidar_distance",String,queue_size=10) #begin to get distance
    get_distance_sub=rospy.Subscriber("/home_service/robot_getdistance",Float32,go_ahead,queue_size=10)#get distance and move

    rospy.spin()