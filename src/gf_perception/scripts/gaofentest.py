import cv2
import numpy as np
img = cv2.imread("/home/ubuntu/GaoFen_Drone/resource/img2/4-118.png")
# immm=img[20:50,30:60,:]
ret,blue_thresh = cv2.threshold(img[:,:,0],115,0,cv2.THRESH_TOZERO_INV)
ret,red_thresh = cv2.threshold(img[:,:,2],115,0,cv2.THRESH_TOZERO_INV)
print(type(blue_thresh))
gray_thresh=blue_thresh & red_thresh
ret,gray_thresh = cv2.threshold(gray_thresh,60,255,cv2.THRESH_BINARY)

# image ,contours,hierarchy = cv2.findContours(white_thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
# imag = cv2.drawContours(img,contours,-1,(0,255,0),3)
# for contour in contours:
#     if len(contour)<10:
#         continue
#     hull = cv2.convexHull(contour)
#     ploy=cv2.approxPolyDP(contour,cv2.arcLength(contour,True)/50,True)
#     if not len(ploy)==4:
#         continue
    
#     print hull
cv2.imshow("i",blue_thresh)
cv2.imshow("i2",red_thresh)
cv2.imshow("22",gray_thresh)
cv2.waitKey(0)