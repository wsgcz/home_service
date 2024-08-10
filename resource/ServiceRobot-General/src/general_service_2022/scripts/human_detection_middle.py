import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import pi, sqrt, cos, atan2, sin
import math
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp
import cv2
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseGoal
import message_filters


w_img = 960  # 图像宽度
h_img = 540  # 图像高度
p_x = 479
p_y = 269
f_x = 540.68603515625
f_y = 540.68603515625
image = Image()
depth = Image()
bridge = CvBridge()
eps = 0.01
mediapipe_detect_count = 0
goals = []
end_goal = MoveBaseGoal()
end_goal.target_pose.header.frame_id = 'base_link'
this_switch = 'off'
rotate_flag = False
start_time = 0.0
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=2,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)


def middle_callback(msg: String):
    global image, depth, eps, goals, this_switch, rotate_flag, start_time, before_pixel_index
    person = json.loads(msg.data)
    x_mid = int(person[0])
    y_mid = int(person[1])
    width = int(person[2])
    height = int(person[3])
    rospy.loginfo(msg.data)
    rospy.loginfo("_________________________________________________")
    depth_x = x_mid + 25
    depth_y = y_mid - 5
    if depth_x <= 0:
        depth_x = 1
    elif depth_x >= 960:
        depth_x = 959
    if depth_y <= 0:
        depth_y = 1
    elif depth_y >= 540:
        depth_y = 539
    if depth[depth_y][depth_x] < eps:
        return
    if person[0] >= 475 and person[0] <= 485:
        results = mediapipe_detect(image[int(
            y_mid - 0.5 * height): int(
            y_mid + 0.5 * height), int(x_mid - 0.5 * width): int(x_mid + 0.5 * width)])
        if results is None:
            rospy.loginfo('human_detection_middle: results获取失败')
            return
        average_depth = 0.0
        count = 0
        for i in range(3):
            for j in range(3):
                y = y_mid - 0.2 * height - 5
                x = x_mid - 0.2 * width + 25
                now_depth = depth[int(y + i * 0.2 * height)][int(x + j * 0.2 * width)]
                if now_depth < eps:
                    continue
                else:
                    if average_depth == 0.0:
                        average_depth = now_depth
                    elif average_depth > now_depth:
                        average_depth = now_depth
                    count += 1
        # average_depth /= count
        real_person = real_pose(x_mid, y_mid, average_depth)
        if real_person[2] < eps:
            return
        rospy.loginfo('x: %f, 深度: %f', real_person[0], average_depth)
        alpha = get_alpha(results)
        z_second = real_person[2] - 0.8 * cos(alpha)
        x_second = real_person[0] + 0.8 * sin(alpha)
        if alpha == 0.5 * pi or alpha == 1.5 * pi:
            z_second += 0.2
        tho_second = sqrt(z_second ** 2 + x_second ** 2)
        theta_second = atan2(x_second, z_second)
        x1, y1 = get_map_pose(
            tho_second*cos(theta_second), tho_second*sin(theta_second))
        robot_states = get_map_pose_theta()
        rpy = ori_to_rpy(
            0.0, 0.0, robot_states[2], robot_states[3])
        theta1 = alpha + rpy[2]
        rospy.loginfo('角度: %f', alpha)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x1
        goal.target_pose.pose.position.y = y1
        xyzw = rpy2quaternion(0.0, 0.0, theta1)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = xyzw[2]
        goal.target_pose.pose.orientation.w = xyzw[3]
        goal_publisher.publish(goal)
        rospy.Rate(6).sleep()
def image_callback(image_rgb: Image, image_depth: Image):
    global image, depth,  bridge
    image = image = bridge.imgmsg_to_cv2(
        image_rgb, desired_encoding='passthrough')
    depth = depth = bridge.imgmsg_to_cv2(
        image_depth, desired_encoding='passthrough')
def mediapipe_detect(image):
    global mp_pose, mediapipe_detect_count
    results = mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if results.pose_landmarks is None:
        cv2.imwrite('/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/data/people_false' +
                    str(mediapipe_detect_count) + '.jpg', image)
        mediapipe_detect_count += 1
        return None
    return results
def real_pose(u_img, v_img, d):
    global p_x, p_y, f_x, f_y
    Z = d/sqrt(1+(u_img-p_x)**2/f_x**2+(v_img-p_y)**2/f_y**2)
    X = (u_img-p_x)*Z/f_x
    Y = (v_img-p_y)*Z/f_y
    return X/1000+0.05, 1.17-Y/1000*cos(0.483), Z/1000


def get_alpha(results):
    z_left = results.pose_world_landmarks.landmark[11].z
    z_right = results.pose_world_landmarks.landmark[12].z
    x_left = results.pose_world_landmarks.landmark[11].x
    x_right = results.pose_world_landmarks.landmark[12].x
    if abs(x_left - x_right) < 0.1:
        if z_left > z_right:
            return 0.5 * pi
        elif z_right > z_left:
            return 1.5 * pi
        else:
            rospy.loginfo('alpha 不合理')
            return 0
    elif abs(z_left - z_right) < 0.1:
        if x_left > x_right:
            return 0
        elif x_left < x_right:
            return pi
        else:
            rospy.loginfo('alpha 不合理')
            return 0
    elif x_left > 0 and z_left > 0:
        alpha = -(atan2(x_left, z_left) +
                  atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    elif x_left < 0 and z_left > 0:
        alpha = pi - (atan2(x_left, z_left) +
                      atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    elif x_left < 0 and z_left < 0:
        alpha = pi - (atan2(x_left, z_left) +
                      atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    elif x_left > 0 and z_left < 0:
        alpha = 2 * pi - (atan2(x_left, z_left) +
                          atan2(x_right, z_right)) * 0.5
        if alpha > 2 * pi:
            alpha -= 2 * pi
        return alpha
    alpha = -(atan2(results.pose_world_landmarks.landmark[11].x, results.pose_world_landmarks.landmark[11].z) +
              atan2(results.pose_world_landmarks.landmark[12].x, results.pose_world_landmarks.landmark[12].z)) * 0.5
    if alpha > 2 * pi:
        alpha -= 2 * pi
    return alpha

def get_map_pose(x, y):
    ps = PointStamped()
    ps.header.frame_id = "base_footprint"
    ps.point.x = x
    ps.point.y = y
    ps.header.stamp = rospy.Time.now()
    ps_new = tfBuffer.transform(ps, "map", rospy.Duration(1))
    return ps_new.point.x, ps_new.point.y


def get_map_pose_theta():
    ps = PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position.x = 0.0
    ps.pose.position.y = 0.0
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = 1.0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = 0.0
    ps.header.stamp = rospy.Time.now()
    ps_new = tfBuffer.transform(ps, "map", rospy.Duration(1))
    return [ps_new.pose.position.x, ps_new.pose.position.y, ps_new.pose.orientation.z, ps_new.pose.orientation.w]
def ori_to_rpy(x, y, z, w):
    from tf import transformations
    (r, p, y) = transformations.euler_from_quaternion([x, y, z, w])
    return [r, p, y]


def rpy2quaternion(roll, pitch, yaw):
    x = sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y = sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z = cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w = cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    return [x, y, z, w]

if __name__ == '__main__':
    rospy.init_node('human_detection_middle', anonymous=True)
    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)
    middle_subscriber = rospy.Subscriber(
        name='general_service_human_detection_middle', data_class=String, callback=middle_callback, queue_size=10
    )
    goal_publisher = rospy.Publisher(name='general_service_human_detection_goal_back', data_class=MoveBaseGoal, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    rospy.spin()