
# STEP 1: Import the necessary modules.
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts,plot_one_box
from face_detect import FaceRecognition
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Twist
from mediapipe import ImageFrame
def draw_landmarks_on_image(rgb_image, detection_result):
  pose_landmarks_list = detection_result.pose_landmarks
  annotated_image = np.copy(rgb_image)

  # Loop through the detected poses to visualize.
  for idx in range(len(pose_landmarks_list)):
    pose_landmarks = pose_landmarks_list[idx]

    # Draw the pose landmarks.
    pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    pose_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
    ])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      pose_landmarks_proto,
      solutions.pose.POSE_CONNECTIONS,
      solutions.drawing_styles.get_default_pose_landmarks_style())
  return annotated_image

# STEP 2: Create an PoseLandmarker object.
base_options = python.BaseOptions(model_asset_path='pose_landmarker_lite.task')
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    output_segmentation_masks=True)
detector = vision.PoseLandmarker.create_from_options(options)

# STEP 3: Load the input image.
cap = cv2.VideoCapture(0)
while (cap.isOpened()):
            # mutex1.acquire()
    ret, image = cap.read()
    image=image.tobytes()
    image_frame=mp.ImageFrame(width=640,height=640,format=mp.ImageFormat.SRGB,data=image)
    # mp_frame = mp.solutions.drawing_utils._to_bgra(image)
    # image = letterbox(image, 960, stride=64, auto=True)[0]
    # image_ = image.copy()
    # image = transforms.ToTensor()(image)
    # image = torch.tensor(np.array([image.numpy()]))
    # image = image.to(device)
    # image = image.half()
    # STEP 4: Detect pose landmarks from the input image.
#     image_frame = mp.solutions.mediapipe.python.solutions.common.Frame(
#     image=image, 
#     rotation=0,  # 旋转角度，默认为0
#     timestamp=0  # 时间戳，默认为0
# )
    # cv2.imshow("@222222",image)
    # cv2.waitKey(1)
    # img_format=image.format
    # detection_result = detector.detect(mp_frame,mp.ImageFrame)

    # # STEP 5: Process the detection result. In this case, visualize it.
    # annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
    # cv2.imshow(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))