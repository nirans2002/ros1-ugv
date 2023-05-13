import rospy
from std_msgs.msg import String
import numpy as np
import cv2
from cv2 import aruco
import math
from pyzbar import pyzbar

def camToAruco():
    pub = rospy.Publisher('cam_feed', String, queue_size=10)
    rospy.init_node('cam_aruco', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # rate.sleep()
        pass
    
# required to write a publisher 
#  

if __name__ == '__main__':
    try:
        camToAruco()
    except rospy.ROSInterruptException:
        pass
