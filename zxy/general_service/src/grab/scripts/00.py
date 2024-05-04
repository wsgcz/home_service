import imp

import tf
import rospy
import json
def listen_tf(self):
    try:
            (pos,ori) = self.tf_listener.lookupTransform("map","base_link",rospy.Duration(0.0))
            msg_dict = {
                "pos_x" : pos[0],
                "pos_y" : pos[1],
                "pos_z" : pos[2],
                "ori_x" : ori[0],
                "ori_y" : ori[1],
                "ori_z" : ori[2],
                "ori_w": ori[3]
            }
            self.current_robot_pose = msg_dict
            return True
    except tf.Exception as e:
            print ("listen to tf failed")
            return False
    

def receive(self):
    while not rospy.is_shutdown() and not self.listen_tf():
            rospy.sleep(1)
    self.robot_record_pose = self.current_robot_pose
    display_msg = "Robot:\n" + json.dumps(self.robot_record_pose,indent=4) + "\n"

    self.text_content.setText(display_msg)
