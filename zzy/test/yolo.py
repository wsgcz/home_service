import torch
from ultralytics import YOLO
import cv2 as cv
import matplotlib.pyplot as plt
import numpy

# model_path = "/home/zzy/vision/src/vis/scripts/yolov10n.pt"
# model = YOLO(model_path)

# image_path = "/home/zzy/Downloads/bus.jpg"
# results = model(image_path)
# for result in results : 
#     boxes = result.boxes  # Boxes object for bounding box outputs
#     masks = result.masks  # Masks object for segmentation masks outputs
#     keypoints = result.keypoints  # Keypoints object for pose outputs
#     probs = result.probs  # Probs object for classification outputs
#     obb = result.obb  # Oriented boxes object for OBB outputs
#     names = result.names
#     #result.show()  # display to screen
#     #result.save(filename="result.jpg")  # save to disk
# cv_image = cv.imread(image_path)
# plt.subplot(121)
# # plt.imshow(cv_image[187:509,112:1125,:])
# b,g,r = cv.split(cv_image)
# img_matplotlib = cv.merge([r,g,b])
# plt.imshow(img_matplotlib)
#print(boxes.xyxy[1][0])
#print(names)
test = list()
test.append(("hhh", 22))
for item in test:
    print(item)