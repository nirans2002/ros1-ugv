#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
# import cv2.aruco as aruco
from cv2 import aruco
import sys
import math
import time


def detect_ArUco(img):
    ## function to detect ArUco markers in the image using ArUco library
    ## argument: img is the test image
    ## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    ## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    ## 				{0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}

	Detected_ArUco_markers = {}
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250 )
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	# print(ids)
	# print(corners)
	print(type(corners))
	for x , y in zip(ids,corners):
		for z,a in zip(x,y):
			Detected_ArUco_markers[z] = a

			
			
	print(Detected_ArUco_markers)

	return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
    ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
    ##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the
    ##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	for x in Detected_ArUco_markers:
		points = Detected_ArUco_markers[x]
		
		top_left = points[0]
		top_right = points[1]
		bottom_right = points[2]
		bottom_left = points[3]

		mid_top = [(top_left[0]+top_right[0])/2 , (top_left[1]+top_right[1])/2 ]
		mid_bottom= [(bottom_left[0]+bottom_right[0])/2 , (bottom_left[1]+bottom_right[1])/2]
		angle = math.atan((mid_top[1]-mid_bottom[1])/(mid_top[0]-mid_bottom[0]))

		print(angle*(180/np.pi))
		ArUco_marker_angles[x] = angle
		

		



	## enter your code here ##

	return ArUco_marker_angles  ## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):
    ## function to mark ArUco in the test image as per the instructions given in problem statement
    ## arguments: img is the test image
    ##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    ##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    ## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##

    return img