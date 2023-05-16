#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
# import cv2.aruco as aruco
from cv2 import aruco as aruco
import time
from SS_1876_aruco_library import *


cap = cv2.VideoCapture("../scripts/aruco_video.mp4")
time.sleep(2)

if not cap.isOpened():
    print("can not find video capture ")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('frame', frame)

    try:

        Detected_ArUco_markers = detect_ArUco(frame)
        angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
        video = mark_ArUco(frame, Detected_ArUco_markers, angle)
        cv2.imshow("frame",video)

    except Exception as e:
        print(e)
    # cv2.imshow('video',frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()