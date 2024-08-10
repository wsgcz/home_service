#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from shopping_vision import VisionTimer
from sensor_msgs.msg import JointState

class EasyGrasp():

    _is_exec:bool = False
    _easy_grasp_timer:VisionTimer = VisionTimer(max_wait_time=10)

    _is_finish:Bool = Bool()
    _is_finish.data = False
    _finish_pub = rospy.Publisher(
        name="easy_grasp_finish",
        data_class=Bool,
        queue_size=3
    )

    _grasp_data:JointState = JointState()
    _grasp_data.position = [0.8, 0.0]
    _grasp_data.velocity = [0.5, 2.0]
    # _grasp_data.position[0] = 0.8 # 机械臂高度
    # _grasp_data.position[1] = 0 # 机械爪宽度
    # _grasp_data.velocity[0] = 0.5 # 机械臂垂直速度
    # _grasp_data.velocity[1] = 2.0 # 机械爪收合速度

    _arm_pub:rospy.Publisher = rospy.Publisher(
        name="/wpb_home/mani_ctrl",
        data_class=JointState,
        queue_size=3
    )

    _speak_str:String = String()
    _voice_pub:rospy.Publisher = rospy.Publisher(
        name="/ourspeak",
        data_class=String,
        queue_size=1
    )

    def timer_reset(self):
        # 超过10s后，机械爪收紧,通知主函数easy grasp结束
        for i in range(3):
            self._arm_pub.publish(self._grasp_data)
        for i in range(3):
            self._finish_pub.publish(self._is_finish)
        self._easy_grasp_timer.time(reset=True)
    
    def speak_publish(self) -> None:
        for i in range(3):
                self._voice_pub.publish(self._speak_str)

    def switch(self, p:Bool):
        '''
        接收到主函数的要求后，收紧爪子，等待十秒结束
        '''
        rospy.loginfo("*************** easy grasp switch! **********************************")
        # if p.data and self._is_exec:
        #     self.easy_grasp_timer(func=self.timer_reset)
        # elif p.data and not self._is_exec:
        #     self._speak_str.data:str = "Please pass me the " + rospy.get_param("item_name")
        #     self._is_finish.data = False # 开始抓取时is_finish设置为False
        # elif not p.data and self._is_exec:
        #     self._speak_str.data:str = "Ok, I get the " + rospy.get_param("item_name")

        # self._is_exec = p.data

        if p.data:
            rospy.sleep(2)

            self._speak_str.data:str = "Please pass me the " + rospy.get_param("item_name")
            self.speak_publish()

            rospy.sleep(7)
            for i in range(3):
                self._arm_pub.publish(self._grasp_data)

            rospy.sleep(7)

            self._speak_str.data:str = "Ok, I get the " + rospy.get_param("item_name")
            self.speak_publish()

            rospy.sleep(2)
            
            self._is_finish = True
            for i in range(3):
                self._finish_pub.publish(self._is_finish)


if __name__=="__main__":

    rospy.init_node("easy_grasy")

    easy_grasp = EasyGrasp()

    rospy.Subscriber(
        name="easy_grasp_switch",
        data_class=Bool,
        callback=easy_grasp.switch,
        queue_size=1
    )

    rospy.spin()