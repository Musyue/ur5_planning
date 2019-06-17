# import the necessary packages
import imutils
import cv2
import numpy as np
import math


# 计算欧式距离
def cal_distance(point1, point2):
    dis = np.sqrt(np.sum(np.square(point1[0] - point2[0]) + np.square(point1[1] - point2[1])))
    return dis


# 基于海伦公式计算不规则四边形的面积
def helen_formula(coord):
    coord = np.array(coord).reshape((4, 2))
    # 计算各边的欧式距离
    dis_01 = cal_distance(coord[0], coord[1])
    dis_12 = cal_distance(coord[1], coord[2])
    dis_23 = cal_distance(coord[2], coord[3])
    dis_31 = cal_distance(coord[3], coord[1])
    dis_13 = cal_distance(coord[0], coord[3])

    p1 = (dis_01 + dis_12 + dis_13) * 0.5
    p2 = (dis_23 + dis_31 + dis_13) * 0.5
    # 计算两个三角形的面积
    area1 = np.sqrt(p1 * (p1 - dis_01) * (p1 - dis_12) * (p1 - dis_13))
    area2 = np.sqrt(p2 * (p2 - dis_23) * (p2 - dis_31) * (p2 - dis_13))
    return (area1+area2)/2
# 基于向量积计算不规则四边形的面积
def vector_product(coord):
    coord = np.array(coord).reshape((4,2))
    print(coord)
    temp_det = 0
    for idx in range(3):
        temp = np.array([coord[idx],coord[idx+1]])
        print("Temp",temp)
        temp_det +=np.linalg.det(temp)
    temp_det += np.linalg.det(np.array([coord[-1],coord[0]]))
    return temp_det/4


# load the image, convert it to grayscale, and blur it slightly
image = cv2.imread("2.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (1, 1), 0)

# threshold the image, then perform a series of erosions +
# dilations to remove any small regions of noise
thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)[1]
thresh = cv2.erode(thresh, None, iterations=2)
thresh = cv2.dilate(thresh, None, iterations=2)

# find contours in thresholded image, then grab the largest
# one
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)
# print(cnts)
cnts = imutils.grab_contours(cnts)
c = max(cnts, key=cv2.contourArea)

# determine the most extreme points along the contour
extLeft = tuple(c[c[:, :, 0].argmin()][0])
extRight = tuple(c[c[:, :, 0].argmax()][0])
extTop = tuple(c[c[:, :, 1].argmin()][0])
extBot = tuple(c[c[:, :, 1].argmax()][0])
font=cv2.FONT_HERSHEY_SIMPLEX
M = cv2.moments(c)
# 获取图像质心坐标
cX = int(M["m10"] / M["m00"])
cY = int(M["m01"] / M["m00"])
now_central = (cX, cY)
cv2.line(image,extLeft,extTop,[0,255,0],2)
area=helen_formula([extLeft[0],extLeft[1],extTop[0],extTop[1],extRight[0],extRight[1],extBot[0],extBot[1]])
print("Area2",helen_formula([extLeft[0],extLeft[1],extTop[0],extTop[1],extRight[0],extRight[1],extBot[0],extBot[1]]))
print("Area",vector_product([extLeft[0],extLeft[1],extTop[0],extTop[1],extRight[0],extRight[1],extBot[0],extBot[1]]))
cv2.line(image,extLeft,extBot,[0,255,0],2)
cv2.line(image,extRight,extTop,[0,255,0],2)
cv2.line(image,extRight,extBot,[0,255,0],2)
cv2.putText(image,"Area ("+str(area)+")",(cX, cY-400), font, 1,(255,255,255),2)
cv2.putText(image,"Cen"+str(now_central),(cX+20, cY+50), font, 1,(255,255,255),2)
cv2.putText(image,"P1"+str(extLeft),extLeft, font, 1,(255,255,255),2)
cv2.putText(image,"P2"+str(extRight),extRight, font, 1,(255,255,255),2)
cv2.putText(image,"P3"+str(extTop),extTop, font, 1,(255,255,255),2)
cv2.putText(image,"P4"+str(extBot),extBot, font, 1,(255,255,255),2)
# draw the outline of the object, then draw each of the
# extreme points, where the left-most is red, right-most
# is green, top-most is blue, and bottom-most is teal
# area=
cv2.drawContours(image, [c], -1, (0, 255, 255), 2)

cv2.circle(image, extLeft, 10, (0, 0, 255), -1)
cv2.circle(image, extRight, 10, (0, 0, 255), -1)
cv2.circle(image, extTop, 10, (0, 0, 255), -1)
cv2.circle(image, extBot, 10, (0, 0, 255), -1)
cv2.circle(image, now_central, 10, (0, 0, 255), -1)
cv2.namedWindow('central_frame_3', cv2.WINDOW_NORMAL)
cv2.imshow('central_frame_3', thresh)
cv2.namedWindow('central_frame', cv2.WINDOW_NORMAL)
# cv2.imshow('central_frame', img)
cv2.imshow("central_frame", image)
cv2.waitKey(0)