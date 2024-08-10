import cv2
import mediapipe as mp

# 加载Mediapipe的姿势估计模型
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# 加载Mediapipe的Objectron模型
mp_objectron = mp.solutions.objectron
objectron = mp_objectron.Objectron(static_image_mode=False, max_num_objects=5, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    # 读取摄像头图像
    ret, frame = cap.read()

    # 使用Objectron模型检测人体
    results = objectron.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # 如果检测到人体
    if results.detected_objects:
        for detected_object in results.detected_objects:
            # 获取人体图像区域
            bounding_box = detected_object.bounding_box
            xmin, ymin, xmax, ymax = int(bounding_box[0] * frame.shape[1]), int(bounding_box[1] * frame.shape[0]), int(bounding_box[2] * frame.shape[1]), int(bounding_box[3] * frame.shape[0])
            body_frame = frame[ymin:ymax, xmin:xmax]

            # 使用姿势估计模型检测人体姿态
            body_results = pose.process(cv2.cvtColor(body_frame, cv2.COLOR_BGR2RGB))

            # 绘制人体姿态
            if body_results.pose_landmarks is not None:
                mp.solutions.drawing_utils.draw_landmarks(body_frame, body_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # 在原图上绘制人体框和姿态
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            frame[ymin:ymax, xmin:xmax] = body_frame

    # 显示图像
    cv2.imshow("Multi-person pose estimation", frame)

    # 按下q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()