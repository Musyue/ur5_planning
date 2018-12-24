#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1[0],line1[1]
    rho2, theta2 = line2[0],line2[1]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]
img = cv2.imread('1.jpg')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

gaus = cv2.GaussianBlur(gray, (3, 3), 0)

edges = cv2.Canny(gaus, 50, 150, apertureSize=3)

minLineLength = 100
maxLineGap = 10
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength, maxLineGap)
print lines
for x1, y1, x2, y2 in lines[0]:
    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    print (x1,y1),(x2,y2)

cv2.imshow("houghline", img)
cv2.waitKey()
cv2.destroyAllWindows()
