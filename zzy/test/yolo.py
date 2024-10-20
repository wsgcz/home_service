import torch
from ultralytics import YOLO
import cv2 as cv
import matplotlib.pyplot as plt

model_path = "/home/zzy/vision/src/vis/scripts/yolov10n.pt"
model = YOLO(model_path)

image_path = "/home/zzy/beishen.jpeg"
results = model(image_path)
for result in results : 
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    #result.show()  # display to screen
    #result.save(filename="result.jpg")  # save to disk
cv_image = cv.imread(image_path)
plt.subplot(121)
plt.imshow(cv_image[187:509,112:1125,:])
print(boxes)
