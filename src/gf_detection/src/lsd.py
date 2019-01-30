#!/usr/bin/python2.7
# coding=utf-8
#!/home/ubuntu/anaconda2/bin/python2.7
import cv2
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge, CvBridgeError
# from matplotlib import pyplot as plt
from skimage import img_as_float, img_as_ubyte
brightness_list = []
k_left_list = []
k_right_list = []
D_left_list = []
D_right_list = []


def line_filter(status, lines):

    global drawing
    left_lines = []
    right_lines = []
    # print '#',len(lines)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        length = math.sqrt(pow(x1-x2, 2)+pow(y1-y2, 2))
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 0, 0), 1, lineType=cv2.LINE_AA)

        # print(abs((y1-y2)*1.0/(x1-x2)-(240-y2)*1.0/(320-x2)))
        if status == 1:  # low height
            if abs(y1-y2) > 5 and abs(x1-x2) > 5 and length > 50:
                if y1 < 300 and y2 < 300 and min(x1, x2) < 300 and max(x1, x2) < 400:
                    left_lines.append(line)

                if y1 < 300 and y2 < 300 and min(x1, x2) > 640-400 and max(x1, x2) > 640-300:
                    right_lines.append(line)
        else:
            if abs(x1-x2) > 5 and  min(x1, x2) < 300 and max(x1, x2) < 400:
                left_lines.append(line)

            if abs(x1-x2) > 5 and min(x1, x2) > 640-400 and max(x1, x2) > 640-300:
                right_lines.append(line)

    left_lines.sort(key=lambda obj: math.sqrt(
        pow(obj[0][0]-obj[0][2], 2)+pow(obj[0][1]-obj[0][3], 2)), reverse=True)
    right_lines.sort(key=lambda obj: math.sqrt(
        pow(obj[0][0]-obj[0][2], 2)+pow(obj[0][1]-obj[0][3], 2)), reverse=True)

    avg_k_l = 0
    D_l = 0
    if len(left_lines) == 1:
        x1, y1, x2, y2 = left_lines[0][0]
        avg_k_l = (y1-y2)*1.0/(x1-x2)
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 255, 0), 1, lineType=cv2.LINE_AA)
    elif len(left_lines) > 1:
        x1, y1, x2, y2 = left_lines[0][0]
        k1 = (y1-y2)*1.0/(x1-x2)
        D1 = (x1*y2-y1*x2)*1.0/(x1-x2)
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 255, 0), 1, lineType=cv2.LINE_AA)
        x1, y1, x2, y2 = left_lines[1][0]
        k2 = (y1-y2)*1.0/(x1-x2)
        D2 = (x1*y2-y1*x2)*1.0/(x1-x2)
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 255, 0), 1, lineType=cv2.LINE_AA)
        avg_k_l = 0.5*(k1+k2)
        D_l = abs(D1-D2)

    k_left_list.append(avg_k_l)
    if len(k_left_list) > 5:
        k_left_list.pop(0)
    k_l_median = np.median(k_left_list)
    D_left_list.append(D_l)
    if len(D_left_list) > 5:
        D_left_list.pop(0)
    D_l_median = np.median(D_left_list)

    avg_k_r = 0
    D_r = 0
    if len(right_lines) == 1:
        x1, y1, x2, y2 = right_lines[0][0]
        avg_k_r = (y1-y2)*1.0/(x1-x2)
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 255, 0), 1, lineType=cv2.LINE_AA)
    elif len(right_lines) > 1:
        x1, y1, x2, y2 = right_lines[0][0]
        k1 = (y1-y2)*1.0/(x1-x2)
        D1 = (x1*y2-y1*x2)*1.0/(x1-x2)
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 255, 0), 1, lineType=cv2.LINE_AA)
        x1, y1, x2, y2 = right_lines[1][0]
        k2 = (y1-y2)*1.0/(x1-x2)
        D2 = (x1*y2-y1*x2)*1.0/(x1-x2)
        cv2.line(drawing, (x1, y1), (x2, y2),
                 (0, 255, 0), 1, lineType=cv2.LINE_AA)
        avg_k_r = 0.5*(k1+k2)
        D_r = abs(D1-D2)

    k_right_list.append(avg_k_r)
    if len(k_right_list) > 5:
        k_right_list.pop(0)
    k_r_median = np.median(k_right_list)
    D_right_list.append(D_r)
    if len(D_right_list) > 5:
        D_right_list.pop(0)
    D_r_median = np.median(D_right_list)

    # print k_l_median, k_r_median, D_l_median, D_r_median
    # print D_left_list, D_right_list
    return D_l_median, D_r_median


def callback(msg):
    global image, drawing
    print 'enter'
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
    except CvBridgeError as error:
        print(error)
    print 'enter  222'
    image = cv2.medianBlur(image, 5)
    image_ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)

    image_float = img_as_float(image_ycrcb[:, :, 0])
    imgmean = np.mean(image_float)
    brightness_list.append(imgmean)
    if len(brightness_list) > 5:
        brightness_list.pop(0)
    median_brightness = np.median(brightness_list)
    # print(median_brightness,brightness_list)

    image_float = image_float+median_brightness-imgmean
    image_ycrcb[:, :, 0] = img_as_ubyte(image_float)
    image1 = cv2.cvtColor(image_ycrcb, cv2.COLOR_YCR_CB2BGR)
    lower_hsv = np.array([130, 130, 130])
    upper_hsv = np.array([195, 195, 195])
    brightness_mask = cv2.inRange(image1, lower_hsv, upper_hsv)
    brightness_mask = cv2.medianBlur(brightness_mask, 5)
    imgmax = np.amax(image1, axis=2)
    imgmin = np.amin(image1, axis=2)
    ret, delta_thresh = cv2.threshold(
        imgmax-imgmin, 8, 255, cv2.THRESH_BINARY_INV)
    mask = delta_thresh & brightness_mask
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8), 2)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), 1)

    mask[0:480,315:325] = 0
   
    # mask=cv2.blur(mask,(5,5))
    # cv2.imshow("mask", delta_thresh)
    # cv2.imshow("mask22", brightness_mask)
    # cv2.imshow("mask1", mask)
    # cv2.waitKey(1)

    drawing = image1.copy()
    imgg, contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    img_contour = np.zeros((480, 640, 1), np.uint8)
    new_contours = []
    for contour in contours:
        if len(contour) > 300:
            # print '####', len(contour)
            new_contours.append(contour)
    img_contour = cv2.drawContours(
        img_contour, new_contours, -1, (255, 255, 255), -1)
    edges = cv2.Canny(img_contour, 50, 150)
    # lines = cv2.HoughLinesP(img_contour, 1, np.pi / 180, 50,minLineLength=80, maxLineGap=5)

    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD, 0.5)
    lines = lsd.detect(img_contour)[0]

    
    if lines is None:
        return
    print len(lines)
    dl, dr = line_filter(1, lines) #low
    if dl>1000:
        dl=0
    if dr>1000:
        dr=0
    msg = Polygon()
    msg1 = Point32()
    msg1.x = dl
    msg1.y = dr
    msg.points.append(msg1)

    dl, dr = line_filter(0, lines)# high
    if dl>1000:
        dl=0
    if dr>1000:
        dr=0
    msg1.x = dl
    msg1.y = dr
    msg.points.append(msg1)
    pub.publish(msg)

    # cv2.imshow("dd", drawing)
    # cv2.waitKey(1)


rospy.init_node('lsd')
subrgb = rospy.Subscriber('/airsim/image/front/rgb', Image, callback)
pub = rospy.Publisher('airsim/object/line', Polygon, queue_size=1)

rospy.spin()
