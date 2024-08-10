
import sys
import rospy
# import torch
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
import os
sys.path.append('/home/linxi/ServiceRobot-General/src/yolov5-in-ros/scripts')
from libyolo import YoloV5s
for i in range(len(sys.path)):
    if "2.7" in sys.path[i]:
        sys.path.append(sys.path.pop(i))
        break

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
success_time = 0.0
yolo = YoloV5s(weight_path=None)
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=1,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)


def rotate(angle):
    vel_cmd = Twist()
    vel_cmd.angular.z = -angle*pi/180
    vel_pub.publish(vel_cmd)


def stop():
    vel_cmd = Twist()
    vel_cmd.linear.x = 0
    vel_cmd.linear.y = 0
    vel_cmd.linear.z = 0
    vel_cmd.angular.x = 0
    vel_cmd.angular.y = 0
    vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)


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


def image_callback(image_rgb: Image, image_depth: Image):
    global image, depth, eps, goals, this_switch, rotate_flag, start_time, bridge, success_time
    image = bridge.imgmsg_to_cv2(
        image_rgb, desired_encoding='passthrough')
    depth = bridge.imgmsg_to_cv2(
        image_depth, desired_encoding='passthrough')
    if this_switch == 'off' or this_switch == 'final_off':
        return
    if success_time == 0.0:
        pass
    elif rospy.Time.now().to_sec() - success_time > 0.5:
        success_time = 0.0
    else:
        return
    if rotate_flag is False:
        rotate_flag = True
        start_time = rospy.Time.now().to_sec()
        rotate(10)
    persons = []
    result, _ = yolo.inference(img=image, if_show=True)
    if 'person' in result:
        yolo_result_dict = result
        for person in yolo_result_dict['person']:
            persons.append(person)
    for person in persons:
        x_mid = int(person[0])
        y_mid = int(person[1])
        width = int(person[2])
        height = int(person[3])
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
        # if depth[depth_y][depth_x] < eps:
        #     return
        if person[0] >= 474 and person[0] <= 486:
            results = mediapipe_detect(image[int(
                y_mid - 0.5 * height): int(
                y_mid + 0.5 * height), int(x_mid - 0.5 * width): int(x_mid + 0.5 * width)])
            if results is None:
                rospy.loginfo('human_detection: results获取失败')
                return
            right_hip_x = int(results.pose_landmarks.landmark[24].x * width + x_mid - 0.5 * width)
            right_hip_y = int(results.pose_landmarks.landmark[24].y * height + 
                y_mid - 0.5 * height)
            left_hip_x = int(results.pose_landmarks.landmark[23].x * width + x_mid - 0.5 * width)
            left_hip_y = int(results.pose_landmarks.landmark[23].y * height + 
                y_mid - 0.5 * height)
            left_shoulder_x = int(results.pose_landmarks.landmark[11].x * width + x_mid - 0.5 * width)
            left_shoulder_y = int(results.pose_landmarks.landmark[11].y * height + 
                y_mid - 0.5 * height)
            right_shoulder_x = int(results.pose_landmarks.landmark[12].x * width + x_mid - 0.5 * width)
            right_shoulder_y = int(results.pose_landmarks.landmark[12].y * height + 
                y_mid - 0.5 * height)
            if right_hip_x > 960:
                right_hip_x = 959
            if right_hip_y > 540:
                right_hip_y = 539
            if left_hip_x > 960:
                left_hip_x = 959
            if left_hip_y > 540:
                left_hip_y = 539
            depth_right_hip = depth[right_hip_y][right_hip_x]
            depth_left_hip = depth[left_hip_y][left_hip_x]
            depth_left_shoulder = depth[left_shoulder_y][left_shoulder_x]
            depth_right_shoulder = depth[right_shoulder_y][right_shoulder_x]
            depths = [depth_left_hip, depth_left_shoulder, depth_right_hip, depth_right_shoulder]
            length = len(depths)
            for i in depths:
                if i < eps:
                    length -= 1
            average_depth = sum(depths) / length
            # average_depth = depth[right_hip_y][right_hip_x]
            real_person = real_pose(x_mid, y_mid, average_depth)
            if real_person[2] < eps:
                return
            rospy.loginfo('x: %f, 深度: %f', real_person[0], average_depth)
            alpha = get_alpha(results)
            z_second = real_person[2] - 0.9 * cos(alpha)
            x_second = real_person[0] - 0.9 * sin(alpha)
            if alpha == 0.5 * pi or alpha == 1.5 * pi:
            	z_second -= 0.2
            if alpha >= 0.8 * pi and alpha <= 1.2 * pi:
                z_second += 0.1
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
            goals.append(goal)
            success_time = rospy.Time.now().to_sec()
    if rospy.Time.now().to_sec() - start_time > 18.5:
        stop()
        goals_num = len(goals)
        rospy.loginfo('识别到 %d 个人', goals_num)
        if goals_num > 3:
            goals_num = 3
        for i in range(goals_num):
            goals[i].target_pose.header.stamp = rospy.Time.now()
            front_pub.publish(goals[i])
        front_pub.publish(end_goal)
        rospy.loginfo('publish done')
        this_switch = 'final_off'
        rospy.signal_shutdown(this_switch)


def human_detection_call_back(switch):
    global this_switch
    if this_switch == 'final_off':
        return
    this_switch = switch.data


if __name__ == '__main__':
    rospy.init_node('test_human_detection', anonymous=True)
    rgb_sub = message_filters.Subscriber('/kinect2/qhd/image_color', Image)
    depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
    ts.registerCallback(image_callback)
    # image_subscriber = rospy.Subscriber(
    #     name='/kinect2/qhd/image_color_rect', data_class=Image, queue_size=10, callback=image_callback
    # )
    # depth_subscriber = rospy.Subscriber(
    #     name='/kinect2/qhd/image_depth_rect', data_class=Image, callback=depth_callback, queue_size=10)
    human_detection_switch_subscriber = rospy.Subscriber(
        name='general_service_human_detection_switch', data_class=String, queue_size=1, callback=human_detection_call_back)

    front_pub = rospy.Publisher(
        "/person/waypoint", MoveBaseGoal, queue_size=100)
    middle_publisher = rospy.Publisher(
        'general_service_human_detection_middle', String, queue_size=10
    )
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()
