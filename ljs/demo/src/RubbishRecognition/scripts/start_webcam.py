#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# 摄像头预测

import cv2

from ultralytics import YOLO

# 加载模型
model = YOLO("runs/detect/train4/weights/best.pt")

# 打开视频
# video_path = "images/resources/demo.mp4"
# 0则是打开摄像头
video_path = 0
cap = cv2.VideoCapture(video_path)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(10) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()