'''导入一些基本的库'''
import random
import os
import cv2
import matplotlib.pyplot as plt
import mediapipe as mp
import time
from tqdm import tqdm
import numpy as np
from PIL import Image, ImageFont, ImageDraw
# ------------------------------------------------
#   mediapipe的初始化
# 	这一步是必须的，因为要使用到以下定义的几个类
# ------------------------------------------------
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(static_image_mode=True)

# v2相对于v1顺时针小于0、逆时针大于0
def get_angle(v1, v2):
    angle = np.dot(v1, v2) / (np.sqrt(np.sum(v1 * v1)) * np.sqrt(np.sum(v2 * v2)))
    angle = np.arccos(angle) / 3.14159265 * 180

    cross = v2[0] * v1[1] - v2[1] * v1[0]
    if cross < 0:
        angle = - angle
    return angle

def get_distance(v1, v2):
    return np.sqrt(np.sum((v1 - v2) ** 2))

def get_pos(keypoints):
    """
    0 - 鼻子
    1 - 左眼(内)
    2 - 左眼
    3 - 左眼(外)
    4 - 右眼(内)
    5 - 右眼
    6 - 右眼(外)
    7 - 左耳
    8 - 右耳
    9 - 嘴(左)
    10 - 嘴(右)
    11 - 左肩
    12 - 右肩
    13 - 左肘
    14 - 右肘
    15 - 左手腕
    16 - 右手腕
    17 - 左小指
    18 - 右小指
    19 - 左食指
    20 - 右食指
    21 - 左大拇指
    22 - 右大拇指
    23 - 左臀
    24 - 右臀
    25 - 左膝
    26 - 右膝
    27 - 左踝
    28 - 右踝
    29 - 左脚后跟
    30 - 右脚后跟
    31 - 左脚尖
    32 - 右脚尖
    """
    str_pose = "无"
    keypoints = np.array(keypoints)

    ### 角度

    # 躯干与竖直轴
    # 左
    v1 = keypoints[11] - keypoints[23]
    v2 = np.array([0, -1]); # 竖直向上的向量
    from_23_11_to_y = get_angle(v1, v2)
    # 右
    v1 = keypoints[12] - keypoints[24]
    v2 = np.array([0, -1]); # 竖直向上的向量
    from_24_12_to_y = get_angle(v1, v2)

    # 小腿之间
    v1 = keypoints[27] - keypoints[25]
    v2 = keypoints[28] - keypoints[26]
    from_25_27_to_26_28 = get_angle(v1, v2)

    # 大腿之间
    v1 = keypoints[25] - keypoints[23]
    v2 = keypoints[26] - keypoints[24]
    from_23_25_to_24_26 = get_angle(v1,v2)

    # 躯干与大腿
    # 左
    v1 = keypoints[11] - keypoints[23]
    v2 = keypoints[25] - keypoints[23]
    from_23_to_11_25 = get_angle(v1, v2)
    # 右
    v1 = keypoints[12] - keypoints[24]
    v2 = keypoints[26] - keypoints[24]
    from_24_to_12_26 = get_angle(v1, v2)    

    # 大腿和小腿
    # 左
    v1 = keypoints[23] - keypoints[25]
    v2 = keypoints[27] - keypoints[25]
    from_25_to_23_27 = get_angle(v1, v2)
    # 右
    v1 = keypoints[24] - keypoints[26]
    v2 = keypoints[28] - keypoints[26]
    from_26_to_24_28 = get_angle(v1, v2)

    # 躯干与小臂
    # 左
    v1 = keypoints[23] - keypoints[11]
    v2 = keypoints[15] - keypoints[13]
    from_11_23_to_13_15 = get_angle(v1,v2)
    # 右
    v1 = keypoints[24] - keypoints[12]
    v2 = keypoints[16] - keypoints[14]
    from_12_24_to_14_16 = get_angle(v1,v2)

    # 手肘
    # 左
    v1 = keypoints[11] - keypoints[13]
    v2 = keypoints[15] - keypoints[13]
    from_13_to_11_15 = get_angle(v1, v2)
    # 右
    v1 = keypoints[12] - keypoints[14]
    v2 = keypoints[16] - keypoints[14]
    from_14_to_12_16 = get_angle(v1, v2)
    
    ### 距离
    
    # 肩膀之间
    distance_11_12 = get_distance(keypoints[11],keypoints[12])

    # 食指和嘴
    # 左
    distance_19_9 = get_distance(keypoints[19],keypoints[9])
    # 右
    distance_20_10 = get_distance(keypoints[20],keypoints[10])

    # 食指和耳
    # 左
    distance_19_7 = get_distance(keypoints[19],keypoints[7])
    # 右
    distance_20_8 = get_distance(keypoints[20],keypoints[8])

    ### 姿态识别

    # 蹲起
    if 20<abs(from_23_11_to_y)<60 and 20<abs(from_24_12_to_y)<60:
        str_pose = "蹲起_1"
        if 30<abs(from_25_to_23_27)<150 and 30<abs(from_26_to_24_28)<150:
            str_pose = "蹲起"
            return str_pose
        return str_pose

    # 站立、走、躺、俯卧撑、吸烟、打电话
    if abs(from_23_11_to_y)<45 and abs(from_24_12_to_y)<45:
        str_pose = "站立"
        if distance_19_9<0.75*distance_11_12 or distance_20_10<0.75*distance_11_12 or distance_19_7<0.75*distance_11_12 or distance_20_8<0.75*distance_11_12:
            distances = [distance_19_9,distance_20_10,distance_19_7,distance_20_8]
            min_index, min_distance = min(enumerate(distances), key=lambda x: x[1])
            if min_index==0 or min_index==1:
                str_pose = "吸烟"
                return str_pose
            else:
                str_pose = "打电话"
                return str_pose
        if abs(abs(from_23_to_11_25)-90)<45 or abs(abs(from_24_to_12_26)-90)<45:
            str_pose = "坐"
            return str_pose
        if abs(from_25_27_to_26_28)>20 or abs(from_23_25_to_24_26)>20:
            str_pose = "行走"
            return str_pose
        if abs(from_11_23_to_13_15)>100:
            str_pose = "举左手"
            if abs(from_12_24_to_14_16)>100:
                str_pose = "举双手"
            return str_pose
        if abs(from_12_24_to_14_16)>100:
            str_pose = "举右手"
            if abs(from_11_23_to_13_15)>100:
                str_pose = "举双手"
            return str_pose
        return str_pose
    else :
        str_pose = "躺"
        if 45<abs(from_11_23_to_13_15)<135 and 45<abs(from_12_24_to_14_16)<135:
            str_pose = "俯卧撑"
            return str_pose
        return str_pose
    
    return str_pose

def process_frame(img):
    start_time = time.time()
    h, w = img.shape[0], img.shape[1]               # 高和宽
    # 调整字体
    tl = round(0.005 * (img.shape[0] + img.shape[1]) / 2) + 1
    tf = max(tl-1, 1)
    # BRG-->RGB
    img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # 将RGB图像输入模型，获取 关键点 预测结果
    results = pose.process(img_RGB)
    keypoints = ['' for i in range(33)]
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        for i in range(33):
            cx = int(results.pose_landmarks.landmark[i].x * w)
            cy = int(results.pose_landmarks.landmark[i].y * h)
            keypoints[i] = (cx, cy)                                 # 得到最终的33个关键点
    else:
        print("NO PERSON")
        struction = "NO PERSON"
        img = cv2.putText(img, struction, (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (255, 255, 0),6)
        return img, struction
    end_time = time.time()
    process_time = end_time - start_time            # 图片关键点预测时间
    fps = 1 / process_time                          # 帧率
    colors = [[random.randint(0,255) for _ in range(3)] for _ in range(33)]
    radius = [random.randint(8,15) for _ in range(33)]
    for i in range(33):
        cx, cy = keypoints[i]
        #if i in range(33):
        img = cv2.circle(img, (cx, cy), radius[i], colors[i], -1)
    str_pose = get_pos(keypoints)            #获取姿态
    # cv2.putText(img, "POSE-{}".format(str_pose), (12, 100), cv2.FONT_HERSHEY_TRIPLEX,
    #             tl / 3, (255, 0, 0), thickness=tf)
    cv2.putText(img, "FPS-{}".format(str(int(fps))), (12, 100), cv2.FONT_HERSHEY_SIMPLEX,tl/3, (255, 255, 0),thickness=tf)
    return img, str_pose

if __name__ == '__main__':
    img_folder_path = "/home/shanhe/mediapipe_pose_predict/all/all_static"

    # 指定图片文件夹路径
    image_folder = img_folder_path

    # 获取文件夹中所有图片文件
    image_files = [f for f in os.listdir(image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]

    # 遍历所有图片文件
    for image_file in image_files:
        
        image_path = os.path.join(image_folder, image_file)
        print(f"正在处理图片: {image_file}")

        # 开始预测
        img = cv2.imread(image_path)
        image, pose_str = process_frame(img)

        if pose_str == "NO PERSON":
            print("未检测到人")
            output_filename = f"{image_file.split('.')[0]}_no_person.jpg"
            output_path = os.path.join(image_folder, output_filename)
            cv2.imwrite(output_path, image)
            print(f"完成处理图片: {image_file}")
            print("----------------------------")
            continue

        # 生成输出文件名
        output_filename = f"{image_file.split('.')[0]}_{pose_str}.jpg"
        output_path = os.path.join(image_folder, output_filename)

        # 保存处理后的图像
        cv2.imwrite(output_path, image)

        print(f"检测到动作：{pose_str}")
        print(f"完成处理图片: {image_file}")
        print("----------------------------")
        # time.sleep(3)  # 每次完成后暂停3秒
    print("所有图片处理完成")


