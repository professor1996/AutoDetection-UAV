#!/usr/bin/python2.7
#coding=utf-8
import rospy
import roslib
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError

def callback(msg):
    global image
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
    except CvBridgeError as error:
        print(error)  
    
    image_ycrcb=cv2.cvtColor(image,cv2.COLOR_BGR2YCrCb)
    imgmean=np.mean(image_ycrcb[:,:,0][100:200])

    th_low=185-(100-imgmean)/2
    th_high=220-(100-imgmean)/2
    image_white=cv2.inRange(image,np.array([th_low,th_low,th_low]),np.array([th_high,th_high,th_high]) )

    th_low=70-(100-imgmean)*1.5
    th_high=110-(100-imgmean)*1.5
    imgmax = np.amax(image,axis=2)
    imgmin = np.amin(image,axis=2)
    ret, delta_thresh = cv2.threshold(imgmax-imgmin, 10, 255, cv2.THRESH_BINARY_INV)
    image_gray=cv2.inRange(image,np.array([th_low,th_low,th_low]),np.array([th_high,th_high,th_high]) )
    image_gray=image_gray & delta_thresh

    image_white,contours, hierarchy = cv2.findContours(image_white,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for i in range(len(contours)):
        if len(contours[i])<50:
            continue
        # cv2.drawContours(image_white,contours,i,(110,110,110),3)
        hull = cv2.convexHull(contours[i])
        approx = cv2.approxPolyDP(hull, 0.4 *len(hull), True)
        if len(approx)==4:
            cv2.circle(image_white,(approx[0][0][0],approx[0][0][1]),2,(110,110,110),3)

        print len(hull),len(approx)
    cv2.imshow("a",image_white)
    cv2.imshow("b",image_white)
    cv2.waitKey(3)


rospy.init_node('gaofen_depth_circle')
subrgb = rospy.Subscriber('/airsim/image/front/rgb', Image, callback)
# pub = rospy.Publisher('airsim/depth/circle', Vector3Stamped, queue_size=1)
# pub_err = rospy.Publisher('airsim/depth/error', Int16, queue_size=1)
rospy.spin()