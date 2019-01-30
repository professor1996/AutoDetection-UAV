#!/usr/bin/python2.7
#coding=utf-8
import rospy
import roslib
from std_msgs.msg import Int16
rospy.init_node('s')
pub = rospy.Publisher('airsim/front_camera/pose_state', Int16, queue_size=1)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
	msg=Int16()
	msg.data=0
	pub.publish(msg)
	rate.sleep()
