import rospy
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from math import atan2,pi,sqrt,sin,cos
import actionlib

def rpy2quaternion(roll, pitch, yaw):
    x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    return [x, y, z, w]


if __name__ == "__main__":
    rospy.init_node("waypoint")
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while (not ac.wait_for_server(rospy.Duration(5))):
        rospy.loginfo("wait for server to come up")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_footprint'
    goal.target_pose.pose.position.x = 2.0
    goal.target_pose.pose.position.y = 0.26
    xyzw = rpy2quaternion(0, 0.0, 0)
    goal.target_pose.pose.orientation.x = xyzw[0]
    goal.target_pose.pose.orientation.y = xyzw[1]
    goal.target_pose.pose.orientation.z = xyzw[2]
    goal.target_pose.pose.orientation.w = xyzw[3]
    goal.target_pose.header.stamp = rospy.Time.now()
    print("----------i will send a goal----------")
    ac.send_goal(goal)