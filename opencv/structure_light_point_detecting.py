#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ur5_planning.msg import uv
from ur5_planning.msg import structure_point

from imutils import contours
from skimage import measure
import imutils
import time
import numpy as np

class DetectStructurePoint:

    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.strpoint_pub = rospy.Publisher("/structure_point_uv", structure_point, queue_size=10)
        self.resuv=[]
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print e

    def process_rgb_image(self,rgb_image):
        central_list=[]
        sumuv=[]

        strresuv={}
        uvuv=uv()
        strpoint_uv=structure_point()

        rgb=rgb_image
        #print "rgb_image\n",rgb
        t=0
        if rgb_image is not None:
            # ap = argparse.ArgumentParser()
            # ap.add_argument("-i", "--image", required=True,
            #                 help="path to the image file")
            # args = vars(ap.parse_args())
            # load the image, convert it to grayscale, and blur it
            # image = cv2.imread(args["image"])

            # image = cv2.imread("1.jpg")
            image = rgb_image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (11, 11), 0)
            # threshold the image to reveal light regions in the
            # blurred image
            thresh = cv2.threshold(blurred, 205, 255, cv2.THRESH_BINARY)[1]
            # cv2.imshow("Image_Gaussian", thresh)
            # perform a series of erosions and dilations to remove
            # any small blobs of noise from the thresholded image
            thresh = cv2.erode(thresh, None, iterations=2)
            thresh = cv2.dilate(thresh, None, iterations=4)
            # perform a connected component analysis on the thresholded
            # image, then initialize a mask to store only the "large"
            # components

            labels = measure.label(thresh, neighbors=8, background=0)
            mask = np.zeros(thresh.shape, dtype="uint8")
            # print("labels",labels)
            # loop over the unique components
            for label in np.unique(labels):
                # if this is the background label, ignore it
                if label == 0:
                    continue

                # otherwise, construct the label mask and count the
                # number of pixels
                labelMask = np.zeros(thresh.shape, dtype="uint8")
                labelMask[labels == label] = 255
                numPixels = cv2.countNonZero(labelMask)
                print("numPixels", numPixels)
                # if the number of pixels in the component is sufficiently
                # large, then add it to our mask of "large blobs"
                if numPixels > 10:
                    mask = cv2.add(mask, labelMask)
            # find the contours in the mask, then sort them from left to
            # right
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            cnts = contours.sort_contours(cnts)[0]
            print("cnts", cnts)
            # loop over the contours
            for (i, c) in enumerate(cnts):
                # draw the bright spot on the image
                (x, y, w, h) = cv2.boundingRect(c)
                ((cX, cY), radius) = cv2.minEnclosingCircle(c)  # seraching the minimum square circle
                print("The minimum circle center", (cX, cY))

                cv2.circle(image, (int(cX), int(cY)), int(radius),
                           (0, 0, 255), 3)
                # print("")
                cv2.putText(image, "#{}".format(i + 1), (x, y - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                strresuv.update({i+1:[int(cX), int(cY)]})
                # strresuv.append({i+1:[cX,cY]})
            print "srereuv",strresuv
            if len(strresuv)==4:
                strpoint_uv.tile_id = 0
                strpoint_uv.f1th_uv.uvinfo = strresuv[1]
                strpoint_uv.s2th_uv.uvinfo = strresuv[2]
                strpoint_uv.t3th_uv.uvinfo = strresuv[3]
                strpoint_uv.f4th_uv.uvinfo = strresuv[4]
                self.strpoint_pub.publish(strpoint_uv)
            else:
                print "please wait ----------"

            #     strresuv.append([i+1,(cX,cY)])
            # if len(strresuv) != 0:
            #     self.strpoint_pub.publish([self.resuv[-1][0], self.resuv[-1][1]])
            # else:
            #     print "wait detecting point-------"
            cv2.namedWindow( 'Structure_point_detecting_window_edges', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Structure_point_detecting_window_edges', thresh )

            cv2.namedWindow( 'Structure_point_window', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Structure_point_window', image )
            cv2.waitKey(8)

            # # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
            except CvBridgeError as e:
                print e
        return central_list
    def papers_alog(self,rgb_image):
        self.process_rgb_image(rgb_image)

def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_DetectStructurePoint")
        rospy.loginfo("Starting Detect Structure Point node")
        k=DetectStructurePoint()
        while not rospy.is_shutdown():
            cen=k.process_rgb_image(k.rgb_image)
            #print "cenpixel\n",cen
            time.sleep(0.1)
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_DetectStructurePoint node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
