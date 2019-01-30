import cv2
import rospy


rospy.init_node('image_publish')
pub_aruco_down = rospy.Publisher(
    'image_raw', ObjectList, queue_size=1)
    