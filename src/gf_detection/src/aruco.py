#!/usr/bin/python2.7
# coding=utf-8
#!/home/ubuntu/anaconda2/bin/python2.7

import rospy
import roslib
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from gf_perception.msg import ObjectList, Object
import sys
import os

save_path = '/home/ubuntu/GaoFen/aruco_img/'
if not os.path.exists(save_path+'image/'):
    os.mkdir(save_path+'image/')
if not os.path.exists(save_path+'depth/'):
    os.mkdir(save_path+'depth/')
image_list = os.listdir(save_path+'image/')
for f in image_list:
    os.remove(save_path+'image/'+f)
image_list2 = os.listdir(save_path+'depth/')
for f in image_list2:
    os.remove(save_path+'depth/'+f)
file_object = open(save_path+'aruco.txt')
aruco_ids = file_object.read().split()
aruco_ids_detect=[]
print aruco_ids
file_input = open(save_path+'result.txt','w+')


def save_pfm(filename, image, scale=1):
    file = open(filename, "w+")
    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    if len(image.shape) == 3 and image.shape[2] == 3:  # color image
        color = True
    # greyscale
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1:
        color = False
    else:
        raise Exception(
            'Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    file.write('PF\n' if color else 'Pf\n')
    file.write('%d %d\n' % (image.shape[1], image.shape[0]))

    endian = image.dtype.byteorder

    if endian == '<' or endian == '=' and sys.byteorder == 'little':
        scale = -scale

    file.write('%f\n' % scale)

    image.tofile(file)


def callback(msg):
    global image_down
    bridge = CvBridge()
    try:
        image_down = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
    except CvBridgeError as error:
        print(error)
    gray = cv2.cvtColor(image_down, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    if ids is not None:
        aruco_msg = ObjectList()
        aruco_msg.count = len(ids)
        aruco_msg.header.stamp = rospy.Time.now()

        for i in range(len(ids)):
            tag = Object()
            tag.number = ids[i]
            tag.center.x = (corners[i][0][0][0]+corners[i][0][2][0])*0.5
            tag.center.y = (corners[i][0][0][1]+corners[i][0][2][1])*0.5
            tag.size.x = abs(corners[i][0][0][0]-corners[i][0][2][0])
            tag.size.y = abs(corners[i][0][0][1]-corners[i][0][2][1])
            aruco_msg.object.append(tag)

            cv2.rectangle(image_down, (corners[i][0][0][0], corners[i][0][0][1]),
                          (corners[i][0][2][0], corners[i][0][2][1]), (0, 255, 0), 2)
        pub_aruco_down.publish(aruco_msg)
    # cv2.imshow("dd", image_down)
    # cv2.waitKey(1)


def callback_front_rgb(msg):
    global image_front_rgb
    global image_depth, image_depth_float
    bridge = CvBridge()
    try:
        image_front_rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
    except CvBridgeError as error:
        print(error)

    gray_front_rgb = cv2.cvtColor(image_front_rgb, cv2.COLOR_BGR2GRAY)
    try:
        gray_depth = gray_front_rgb & image_depth
    except NameError as error:
        print(error)
        return
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        gray_front_rgb, aruco_dict, parameters=parameters)
    corners2, ids2, rejectedImgPoints2 = cv2.aruco.detectMarkers(
        gray_depth, aruco_dict, parameters=parameters)

    if ids is not None:
        aruco_msg = ObjectList()
        aruco_msg.count = len(ids)
        aruco_msg.header.stamp = rospy.Time.now()
        for i in range(len(ids)):
            tag = Object()
            tag.number = ids[i]
            tag.center.x = (corners[i][0][0][0]+corners[i][0][2][0])*0.5
            tag.center.y = (corners[i][0][0][1]+corners[i][0][2][1])*0.5
            tag.size.x = abs(corners[i][0][0][0]-corners[i][0][2][0])
            tag.size.y = abs(corners[i][0][0][1]-corners[i][0][2][1])
            if tag.number > 0:
                aruco_msg.object.append(tag)

        if ids2 is not None:
            for i in range(len(ids2)):
                cv2.rectangle(image_front_rgb, (corners2[i][0][0][0], corners2[i][0][0][1]),
                              (corners2[i][0][2][0], corners2[i][0][2][1]), (0, 255, 0), 2)
                sizex = abs(corners[i][0][0][0]-corners[i][0][2][0])
                sizey = abs(corners[i][0][0][1]-corners[i][0][2][1])
                print ids2[i][0],aruco_ids
                if sizex*sizey > 43*43 and (str(ids2[i][0]) in aruco_ids):
                    cv2.imwrite(save_path+'image/%d.jpg' % ids2[i], image_front_rgb)
                    save_pfm(save_path+'depth/%d.pfm' % ids2[i], image_depth_float)
                    print ids2[i][0],aruco_ids_detect
                    strr=''
                    if ids2[i][0] not in aruco_ids_detect:
                        aruco_ids_detect.append(ids2[i][0])
                        strr='{} {} {} {} {}\n'.format(ids2[i][0],int(corners[i][0][0][0]),int(corners[i][0][0][1]),int(corners[i][0][2][0]),int(corners[i][0][2][1]))
                        file_input.write(strr)

                        

        pub_aruco_front.publish(aruco_msg)

    # cv2.imshow("ff", image_front_rgb)
    # cv2.imshow("d", gray_depth)
    # cv2.waitKey(1)


def callback_front_depth(msg):
    global image_depth, image_depth_float
    bridge = CvBridge()
    try:
        image_depth_float = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        image_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC1")
    except CvBridgeError as error:
        print(error)
    cv2.threshold(image_depth, 150, 255, cv2.THRESH_BINARY_INV, image_depth)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    image_depth = cv2.dilate(image_depth, kernel, iterations=1)


rospy.init_node('gaofen_aruco')
subrgb = rospy.Subscriber('/airsim/image/down/rgb', Image, callback)
subrgb = rospy.Subscriber('/airsim/image/front/rgb', Image, callback_front_rgb)
subrgb = rospy.Subscriber('/airsim/image/front/depth',
                          Image, callback_front_depth)
pub_aruco_down = rospy.Publisher(
    'airsim/object/aruco/down', ObjectList, queue_size=1)
pub_aruco_front = rospy.Publisher(
    'airsim/object/aruco/front', ObjectList, queue_size=1)


rospy.spin()
