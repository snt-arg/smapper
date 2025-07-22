#!/bin/python

import cv2
import sys


args = sys.argv
camera_id = sys.argv[1] if len(sys.argv) == 2 else 0

# Define the GStreamer pipeline
gst_pipeline = (
    f"nvarguscamerasrc sensor-id={camera_id} ! "
    "video/x-raw(memory:NVMM), format=NV12, width=2560, height=1920, framerate=60/1 ! "
    "nvvidconv ! "
    "videorate ! "
    "video/x-raw, width=640, height=480, framerate=30/1, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "queue!"
    "appsink"
)

# Create a VideoCapture object with the GStreamer pipeline
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera with GStreamer pipeline")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error capturing frame")
        break

    cv2.imshow("Camera Feed", frame)
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
