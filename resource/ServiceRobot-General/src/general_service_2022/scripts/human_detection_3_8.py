from enum import Flag
from pyexpat.errors import XML_ERROR_DUPLICATE_ATTRIBUTE
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import atan2, pi, sqrt, asin, sin, cos
import json
import time
import math
import numpy as np
from move_base_msgs.msg import MoveBaseGoal
from tf2_geometry_msgs import PointStamped, PoseStamped
import tf2_ros
from geometry_msgs.msg import Pose2D

init_flag = False
w_img = 960  # 图像宽度
h_img = 540  # 图像高度
# P_x = W_img//2-1  # 图像中心点x坐标
p_x = 479
# P_y = H_img//2-1  # 图像中心点y坐标
p_y = 269
f_x = 540.68603515625
f_y = 540.68603515625
eps = 1e-5  # 极小正数
this_switch = 'off'

success_count = 0

mp_drawing = mp.solutions.drawing_utils  # type: ignore
mp_drawing_styles = mp.solutions.drawing_styles  # type: ignore
mp_pose = mp.solutions.pose.Pose(static_image_mode=True,  # type: ignore
                                 model_complexity=2,
                                 enable_segmentation=True,
                                 min_detection_confidence=0.5, min_tracking_confidence=0.7)

bridge = CvBridge()
find_time = -1
image = []
image_rgb = None
depth = []
yolo_result = ""
pose_diff_theta = 0.0
goals = []
# 需要持续执行


def human_detection_call_back(switch):
    global this_switch
    if this_switch == 'final_off':
        return
    this_switch = switch.data


def init_camera(camera_info):
    global init_flag, p_x, p_y, f_x, f_y
    if init_flag is True:
        return
    f_x = camera_info.K[0]
    p_x = camera_info.K[2]
    f_y = camera_info.K[4]
    p_y = camera_info.K[5]

# 图片，左上角x，左上角y，宽度，高度


count = 0


# def mediapipe_detect(image, u_img, v_img, width, height):


def mediapipe_detect(image, width, height):
    # global mp_pose, w_img, h_img, count
    global mp_pose, count
    # image_cut = np.zeros([h_img, w_img, 3])
    # u_img_right = u_img + width
    # v_img_down = v_img + height
    # v_img -= 30
    # if v_img < 0:
    #     v_img = 0
    # if u_img < 0:
    #     u_img = 0
    # if u_img_right > w_img:
    #     u_img_right = w_img
    # if v_img_down > h_img:
    #     v_img_down = h_img
    # image_cut[
    #     v_img: v_img_down, u_img: u_img_right, :] = image[v_img: v_img_down, u_img: u_img_right, :]
    # image_cut = np.uint8(image_cut)
    results = mp_pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if results.pose_landmarks is None:
        cv2.imwrite('/home/linxi/ServiceRobot-General/src/general_service_2022/scripts/data/people_false' +
                    str(count) + '.jpg', image)
        count += 1
        return None
    return results

# 第一个是水平偏移，第二个是高度偏移，第三个是前后偏移


def real_pose(u_img, v_img, d):
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


def get_person_front(left_x, left_y, right_x, right_y):
    # if abs(left_x - right_x) < 1e-2:
    #     if left_y > right_y:
    #         return [left_x + 0.5, (left_y + right_y) * 0.5]
    #     return [left_x - 0.5, (left_y + right_y) * 0.5]
    # if abs(left_y - right_y) < 1e-2:
    #     if left_x > right_x:
    #         return [(left_x + right_x) * 0.5, left_y - 0.5]
    #     return [(left_x + right_x) * 0.5, left_y + 0.5]
    # mid_x = (left_x + right_x) / 2
    # mid_y = (left_y + right_y) / 2
    # theta = atan(abs((right_x - left_x) / (right_y - left_y)))
    # dx = 0.5 * cos(theta)
    # dy = 0.5 * sin(theta)
    # if right_y > left_y and right_x > left_x:
    #     return [mid_x - dx, mid_y + dy]
    # if right_y < left_y and right_x > left_x:
    #     return [mid_x + dx, mid_y + dy]
    # if right_y > left_y and right_x < left_x:
    #     return [mid_x - dx, mid_y - dy]
    # if right_y < left_y and right_x < left_x:
    #     return [mid_x + dx, mid_y - dy]
    # 直角坐标转极坐标
    left_theta = atan2(left_x, left_y)
    left_tho = sqrt(left_x ** 2 + left_y ** 2)
    right_theta = atan2(right_x, right_y)
    right_tho = sqrt(right_x ** 2 + right_y ** 2)
    tho = sqrt(left_tho ** 2 + right_tho ** 2 + 2 * left_tho *
               right_tho * cos(right_theta - left_theta)) / 2
    theta = right_theta - asin(left_tho / tho * sin(right_theta - left_theta))
    if left_theta < right_theta:
        final_tho = tho + 0.5
    else:
        final_tho = tho - 0.5
    x1, y1 = get_map_pose(final_tho * cos(theta), final_tho * sin(theta))
    x0, y0 = get_map_pose(0, 0)
    final_theta = atan2(x1 - x0, y1 - y0)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x1
    goal.target_pose.pose.position.y = y1
    goal.target_pose.pose.orientation.w = cos(final_theta/2)
    goal.target_pose.pose.orientation.z = sin(final_theta/2)
    return goal
# 负数左转，正数右转


def rotate(angle):
    vel_cmd = Twist()
    vel_cmd.angular.z = -angle*pi/180
    vel_pub.publish(vel_cmd)


def start(x, y, z):
    vel_cmd = Twist()
    vel_cmd.linear.x = x
    vel_cmd.linear.y = y
    vel_cmd.linear.z = z
    vel_cmd.angular.x = 0
    vel_cmd.angular.y = 0
    vel_cmd.angular.z = 0
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


def image_call_back(image_and_yolo_result):
    global w_img, h_img, p_x, p_y, f_x, f_y, eps, find_time, yolo_result, this_switch, depth, success_count, image_rgb
    this_switch = 'second'
    if this_switch == 'off' or this_switch == 'final_off' or this_switch == 'first':
        return
    image = bridge.imgmsg_to_cv2(
        image_rgb, desired_encoding='passthrough')
    yolo_result = image_and_yolo_result.data
    if this_switch == 'second':
        state = ''
        persons = []
        if 'person' in yolo_result:
            yolo_result_dict = json.loads(yolo_result)
            for person in yolo_result_dict['person']:
                if person[4] > 0.5:
                    persons.append(person)
        if len(persons) <= 0:
            # robot_datas = get_map_pose_theta()
            # rospy.loginfo('%.2f %.2f %.2f %.2f',
            #               robot_datas[0], robot_datas[1], robot_datas[2], robot_datas[3])
            rotate(40)
            state = 'search'
            find_time = -1
        elif len(persons) == 1:
            # 测试
            x_mid = int(persons[0][0])
            y_mid = int(persons[0][1])
            rospy.loginfo('-------------------------------')
            rospy.loginfo(depth[y_mid - 5][x_mid + 25] / 1000)
            rospy.loginfo('x: %f, y: %f, z: %f', *real_pose(x_mid,
                          y_mid, depth[y_mid - 5][x_mid + 25]))
            rotate(30)
            state = 'found 1'
            find_time = -1
        elif len(persons) == 2:
            rotate(20)
            state = 'found 2'
            find_time = -1
        elif len(persons) >= 3:
            if len(persons) != 3:
                areas = []
                max = []
                l = len(persons)
                for i in range(l):
                    area = persons[i][2] * persons[i][3]
                    areas.append(area)
                areas.sort(reverse=True)
                while len(areas) != 3:
                    areas.pop()
                for i in range(l):
                    area = persons[i][2] * persons[i][3]
                    if area in areas:
                        pass
                    else:
                        persons[i][0] = -1
                    pass
                l = len(persons)
                flag = False
                while not flag:
                    flag = True
                    l = len(persons)
                    for i in range(l):
                        l = len(persons)
                        if persons[i][0] == -1:
                            del persons[i]
                            flag = False
                            break  
                find_time = -1
            # 找到了三个人
            state = 'found 3'
            mid = int(persons[0][0] + persons[1][0] + persons[2][0]) // 3
            x_mids = [int(persons[0][0]), int(
                persons[1][0]), int(persons[2][0])]
            y_mids = [int(persons[0][1]), int(
                persons[1][1]), int(persons[2][1])]
            person_widths = [int(persons[0][2]), int(
                persons[1][2]), int(persons[2][2])]
            persons_heights = [int(persons[0][3]), int(
                persons[1][3]), int(persons[2][3])]
            y_depth = []
            x_depth = []
            for i in range(3):
                if y_mids[i] < 5:
                    y_depth.append(1)
                elif y_mids[i] > 515:
                    y_depth.append(514)
                else:
                    y_depth.append(y_mids[i] - 5)
                if x_mids[i] < 5:
                    x_depth.append(1)
                elif x_mids[i] > 935:
                    x_depth.append(934)
                else:
                    x_depth.append(x_mids[i] + 25)
            if depth[y_depth[0]][x_depth[0]] < eps or depth[y_depth[1]][x_depth[1]] < eps or depth[y_depth[2]][x_depth[2]] < eps:
                state = 'found 3, lost depth'
                if mid < p_x - 200:
                    rotate(-10)
                elif mid > p_x + 200:
                    rotate(10)
                find_time = -1
            else:
                # 深度未丢失
                if find_time < 0:
                    find_time = time.time()
                    # 调整方向
                    # if mid < p_x-100:
                    #     rotate(-30)
                    #     rospy.Rate(2).sleep()
                    # elif mid > p_x+100:
                    #     rotate(30)
                    #     rospy.Rate(2).sleep()
                    # else:
                    #     stop()
                elif time.time()-find_time > 0.1:
                    # 数值稳定，防止误判
                    stop()
                    # TODO: 开始核心算法
                    # 第五版代码
                    # results = [mediapipe_detect(image, int(x_mids[0] - 0.5 * person_widths[0]), int(
                    #     y_mids[0] - 0.5 * persons_heights[0]), int(person_widths[0]), int(persons_heights[0])), mediapipe_detect(image, int(x_mids[1] - 0.5 * person_widths[1]), int(
                    #         y_mids[1] - 0.5 * persons_heights[1]), int(person_widths[1]), int(persons_heights[1])), mediapipe_detect(image, int(x_mids[2] - 0.5 * person_widths[2]), int(
                    #             y_mids[2] - 0.5 * persons_heights[2]), int(person_widths[2]), int(persons_heights[2]))]
                    results = [mediapipe_detect(image[int(
                        y_mids[0] - 0.5 * persons_heights[0]): int(
                        y_mids[0] - 0.5 * persons_heights[0]) + int(persons_heights[0]), int(x_mids[0] - 0.5 * person_widths[0]): int(x_mids[0] - 0.5 * person_widths[0]) + int(person_widths[0])], int(person_widths[0]), int(persons_heights[0])), mediapipe_detect(image[int(
                            y_mids[1] - 0.5 * persons_heights[1]): int(
                            y_mids[1] - 0.5 * persons_heights[1]) + int(persons_heights[1]), int(x_mids[1] - 0.5 * person_widths[1]): int(x_mids[1] - 0.5 * person_widths[1]) + int(person_widths[1])], int(person_widths[1]), int(persons_heights[1])),
                        mediapipe_detect(image[int(
                            y_mids[2] - 0.5 * persons_heights[2]): int(
                            y_mids[2] - 0.5 * persons_heights[2]) + int(persons_heights[2]), int(x_mids[2] - 0.5 * person_widths[2]): int(x_mids[2] - 0.5 * person_widths[2]) + int(person_widths[2])], int(person_widths[2]), int(persons_heights[2]))]
                    for result in results:
                        if result is None:
                            rospy.loginfo('results获取失败')
                            success_count -= 1
                            stop()
                            return
                    success_count += 1
                    if success_count < 5:
                        return

                    real_persons = [real_pose(x_mids[0], y_mids[0], depth[y_depth[0]][x_depth[0]]),
                                    real_pose(
                        x_mids[1], y_mids[1], depth[y_depth[1]][x_depth[1]]),
                        real_pose(x_mids[2], y_mids[2], depth[y_depth[2]][x_depth[2]])]
                    robot_states = get_map_pose_theta()
                    rpy = ori_to_rpy(
                        0.0, 0.0, robot_states[2], robot_states[3])
                    for i in range(3):
                        alpha = get_alpha(results[i])
                        rospy.loginfo('第 %i 个人 角度 %.2f', i, alpha * 180 / pi)
                        rospy.loginfo('第 %i 个人 x: %.2f', i, real_persons[i][0])
                        rospy.loginfo('第 %i 个人 z: %.2f', i, real_persons[i][2])
                        if real_persons[i][2] < eps:
                            success_count -= 1
                            return
                        z_second = real_persons[i][2] - 0.8 * cos(alpha)
                        x_second = real_persons[i][0] + 0.8 * sin(alpha)
                        if alpha == 0.5 * pi or alpha == 1.5 * pi:
                            z_second += 0.2
                        tho_second = sqrt(z_second ** 2 + x_second ** 2)
                        theta_second = atan2(x_second, z_second)
                        x1, y1 = get_map_pose(
                            tho_second*cos(theta_second), tho_second*sin(theta_second))
                        # x0, y0 = get_map_pose(0, 0)
                        # theta1 = atan2(y2 - y1, x2 - x1)
                        theta1 = alpha + rpy[2]
                        # if theta1 > 2 * pi:
                        #     theta1 -= (2 * pi)
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
                    goals[0].target_pose.header.stamp = goals[1].target_pose.header.stamp = goals[2].target_pose.header.stamp = rospy.Time.now()

                    # 第四版代码
                    # results = [mediapipe_detect(image, int(x_mids[0] - 0.5 * person_widths[0]), int(
                    #     y_mids[0] - 0.5 * persons_heights[0]), int(person_widths[0]), int(persons_heights[0])), mediapipe_detect(image, int(x_mids[1] - 0.5 * person_widths[1]), int(
                    #         y_mids[1] - 0.5 * persons_heights[1]), int(person_widths[1]), int(persons_heights[1])), mediapipe_detect(image, int(x_mids[2] - 0.5 * person_widths[2]), int(
                    #             y_mids[2] - 0.5 * persons_heights[2]), int(person_widths[2]), int(persons_heights[2]))]
                    # for result in results:
                    #     if result is None:
                    #         rospy.loginfo('results获取失败')
                    #         stop()
                    #         return
                    # left_shoulders = [
                    #     real_pose(results[0].pose_landmarks.landmark[11].x * w_img, results[0].pose_landmarks.landmark[11].y * h_img, depth[int(
                    #         results[0].pose_landmarks.landmark[11].y * h_img) - 5][int(results[0].pose_landmarks.landmark[11].x * w_img) + 25]),
                    #     real_pose(results[1].pose_landmarks.landmark[11].x * w_img, results[1].pose_landmarks.landmark[11].y * h_img, depth[int(
                    #         results[1].pose_landmarks.landmark[11].y * h_img) - 5][int(results[1].pose_landmarks.landmark[11].x * w_img) + 25]),
                    #     real_pose(results[2].pose_landmarks.landmark[11].x * w_img, results[2].pose_landmarks.landmark[11].y * h_img, depth[int(results[2].pose_landmarks.landmark[11].y * h_img) - 5][int(results[2].pose_landmarks.landmark[11].x * w_img) + 25])]
                    # right_shoulders = [
                    #     real_pose(results[0].pose_landmarks.landmark[12].x * w_img, results[0].pose_landmarks.landmark[12].y * h_img, depth[int(
                    #         results[0].pose_landmarks.landmark[12].y * h_img) - 5][int(results[0].pose_landmarks.landmark[12].x * w_img) + 25]),
                    #     real_pose(results[1].pose_landmarks.landmark[12].x * w_img, results[1].pose_landmarks.landmark[12].y * h_img, depth[int(
                    #         results[1].pose_landmarks.landmark[12].y * h_img) - 5][int(results[1].pose_landmarks.landmark[12].x * w_img) + 25]),
                    #     real_pose(results[2].pose_landmarks.landmark[12].x * w_img, results[2].pose_landmarks.landmark[12].y * h_img, depth[int(results[2].pose_landmarks.landmark[12].y * h_img) - 5][int(results[2].pose_landmarks.landmark[12].x * w_img) + 25])]
                    # for left_shoulder in left_shoulders:
                    #     if left_shoulder[0] == 0.0:
                    #         return
                    #     if left_shoulder[2] == 0.0:
                    #         return
                    # for right_shoulder in right_shoulders:
                    #     if right_shoulder[0] == 0.0:
                    #         return
                    #     if right_shoulder[2] == 0.0:
                    #         return
                    # success_count += 1
                    # rospy.loginfo(
                    #     'results获取成功, 当前 success_count 为 %d', success_count)
                    # if success_count < 5:
                    #     return
                    # goals = [get_person_front(left_shoulders[0][0], left_shoulders[0][2], right_shoulders[0][0], right_shoulders[0][2]),
                    #          get_person_front(
                    #     left_shoulders[1][0], left_shoulders[1][2], right_shoulders[1][0], right_shoulders[1][2]),
                    #     get_person_front(left_shoulders[2][0], left_shoulders[2][2], right_shoulders[2][0], right_shoulders[2][2])]
                    # # 获取机器人当前在地图上的坐标
                    # rospy.loginfo('第一个人左肩左右: %f, 前后: %f',
                    #               left_shoulders[0][0], left_shoulders[0][2])
                    # rospy.loginfo('第二个人左肩左右: %f, 前后: %f',
                    #               left_shoulders[1][0], left_shoulders[1][2])
                    # rospy.loginfo('第三个人左肩左右: %f, 前后: %f',
                    #               left_shoulders[2][0], left_shoulders[2][2])
                    # rospy.loginfo('第一个人右肩左右: %f, 前后: %f',
                    #               right_shoulders[0][0], right_shoulders[0][2])
                    # rospy.loginfo('第二个人右肩左右: %f, 前后: %f',
                    #               right_shoulders[1][0], right_shoulders[1][2])
                    # rospy.loginfo('第三个人右肩左右: %f, 前后: %f',
                    #               right_shoulders[2][0], right_shoulders[2][2])
                    # goals[0].target_pose.header.stamp = goals[1].target_pose.header.stamp = goals[2].target_pose.header.stamp = rospy.Time.now()
                    front_pub1.publish(goals[0])
                    front_pub2.publish(goals[1])
                    front_pub3.publish(goals[2])
                    rospy.loginfo('publish done')

                    this_switch = 'final_off'
                    rospy.signal_shutdown(this_switch)
                    exit(0)
                    # rospy.loginfo('fronts 获取完毕')
                    # current_robot_pose = listen_tf()
                    # rospy.loginfo('listen_tf done')
                    # for i in range(3):
                    #     pss[i].header.frame_id = 'base_footprint'
                    #     pss[i].header.stamp = rospy.Time.now()
                    #     pss[i].point.x = fronts[i][0]
                    #     pss[i].point.y = fronts[i][1]
                    #     pss[i].point.z = 0
                    #     rospy.loginfo('pss done, 第%d个', i)
                    #     point_target = my_tf_buffer.transform(pss[i], 'map', rospy.Duration(1))
                    #     rospy.loginfo('transform done')
                    #     if i == 0:
                    #         goals[i].target_pose.header.frame_id = 'first'
                    #     elif i == 1:
                    #         goals[i].target_pose.header.frame_id = 'second'
                    #     else:
                    #         goals[i].target_pose.header.frame_id = 'third'
                    #     goals[i].target_pose.pose.position.x = point_target.point.x
                    #     goals[i].target_pose.pose.position.y = point_target.point.y
                    #     goals[i].target_pose.pose.orientation.w = 0
                    #     goals[i].target_pose.pose.orientation.z = 0
                    # goals[0].target_pose.header.stamp = goals[1].target_pose.header.stamp = goals[2].target_pose.header.stamp = rospy.Time.now()
                    # front_pub1.publish(goals[0])
                    # front_pub2.publish(goals[1])
                    # front_pub3.publish(goals[2])
                    # rospy.loginfo('publish done')
                    # left_x_index = x_mids.index(min(x_mids))
                    # if min(x_mids) > 480:
                    #     # 人太偏右了
                    #     rotate(5)
                    #     return
                    # left_results = mediapipe_detect(image, int(x_mids[left_x_index] - 0.5 * person_widths[left_x_index]), int(y_mids[left_x_index] -
                    #                                 0.5 * persons_heights[left_x_index]), int(person_widths[left_x_index]), int(persons_heights[left_x_index]))
                    # if left_results == None:
                    #     # 没识别出来
                    #     return
                    # rospy.loginfo('识别到最左边results')
                    # # 最左边人的中心点向下作垂线，与地面交点的 x 像素
                    # left_person_center_ground = (left_results.pose_landmarks.landmark[
                    #     31].x + left_results.pose_landmarks.landmark[32].x)
                    # # 最左边的人方位角
                    # theta = atan((480 - left_person_center_ground) / 480)
                    # if theta > 0:
                    #     theta = -theta
                    # rotate(theta * 90 / pi)
                    # message = String()
                    # message.data = 'success'
                    # human_detection_success_face_publisher.publish(message)
                    # rospy.sleep(rospy.Duration(2))
                    # stop()
                    # rospy.loginfo('成功正对最左边的人')

        rospy.loginfo("human_detection: %s", state)


def depth_call_back(image_depth):
    global depth, bridge, this_switch
    if this_switch == 'first' or this_switch == 'second':
        depth = bridge.imgmsg_to_cv2(
            image_depth, desired_encoding='passthrough')
        this_switch = 'second'


def image_call_back_rgb(rgb_image):
    global image_rgb, this_switch
    if this_switch == 'first' or this_switch == 'second':
        image_rgb = rgb_image
        this_switch = 'second'


if __name__ == '__main__':
    rospy.init_node('z_test', anonymous=True)
    human_detection_success_face_publisher = rospy.Publisher(
        'general_service_human_detection_success_face', String, queue_size=100)
    human_detection_switch_subscriber = rospy.Subscriber(
        name='general_service_human_detection_switch', data_class=String, queue_size=1, callback=human_detection_call_back)
    camera_sub = rospy.Subscriber(
        '/kinect2/qhd/camera_info', CameraInfo, init_camera)
    image_and_yolo_result_subscriber = rospy.Subscriber(
        name='yolo_result', data_class=String, callback=image_call_back, queue_size=10)
    depth_subscriber = rospy.Subscriber(
        name='/kinect2/qhd/image_depth_rect', data_class=Image, callback=depth_call_back, queue_size=10)
    image_subscriber = rospy.Subscriber(
        name='/kinect2/qhd/image_color_rect', data_class=Image, callback=image_call_back_rgb, queue_size=10
    )
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    front_pub1 = rospy.Publisher(
        "/person/waypoint1", MoveBaseGoal, queue_size=100)
    front_pub2 = rospy.Publisher(
        "/person/waypoint2", MoveBaseGoal, queue_size=100)
    front_pub3 = rospy.Publisher(
        "/person/waypoint3", MoveBaseGoal, queue_size=100)

    tfBuffer = tf2_ros.Buffer()
    tfSub = tf2_ros.TransformListener(tfBuffer)
    rospy.spin()
