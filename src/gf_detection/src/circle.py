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
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC1")
    except CvBridgeError as error:
        print(error)  
    count_zero=np.sum(image)/640/480
    msg_err=Int16()
    msg_err.data=count_zero
        
    circles = cv2.HoughCircles(image,cv2.HOUGH_GRADIENT,1,80,
                            param1=70,param2=30,minRadius=0,maxRadius=0)
    circle_msg=Vector3Stamped()
    if circles is not None:
        circle_mean=np.mean(circles,axis=1)
        
        circle_msg=Vector3Stamped()
        circle_msg.header.stamp=rospy.Time.now()
        circle_msg.vector.x=circle_mean[0][0]
        circle_msg.vector.y=circle_mean[0][1]
        circle_msg.vector.z=circle_mean[0][2]
        print("Circle center {},{} Size:{}".format(circle_mean[0][0],circle_mean[0][1],circle_mean[0][2]))
        for i in circles[0,:]:
        # draw the outer circle
            cv2.circle(image,(i[0],i[1]),i[2],(120,120,120),2)
        # draw the center of the circle
            cv2.circle(image,(i[0],i[1]),2,(120,120,120),3) 
        
    pub.publish(circle_msg)
    pub_err.publish(msg_err)

    
    
    # cv2.imshow("a",image)
    # cv2.waitKey(3)


rospy.init_node('gaofen_depth_circle')
subrgb = rospy.Subscriber('/airsim/image/front/depth', Image, callback)
pub = rospy.Publisher('airsim/depth/circle', Vector3Stamped, queue_size=1)
pub_err = rospy.Publisher('airsim/depth/error', Int16, queue_size=1)
rospy.spin()