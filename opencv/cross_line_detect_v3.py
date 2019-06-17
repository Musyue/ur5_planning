#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ur5_planning.msg import uv
from ur5_planning.msg import tileuv

import time
import numpy as np

class DetectTile:

    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.cross_pub = rospy.Publisher("/cross_uv", uv, queue_size=10)
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
        minuv=[]
        minuvuv=[]
        ##################
        DELAY = 0.02
        USE_CAM = 1
        IS_FOUND = 0
        count=0#count feature tile numbers
        cnt=0
        MORPH = 7
        CANNY = 250
        ##################
        _width  = 480.0
        _height = 640.0
        _margin = 0.0
        error=331+229
        cornerssuv=[]
        uvuv=uv()
        tile_uv=tileuv()
        ##################
        corners = np.array(
            [
                [[  		_margin, _margin 			]],
                [[ 			_margin, _height + _margin  ]],
                [[ _width + _margin, _height + _margin  ]],
                [[ _width + _margin, _margin 			]],
            ]
        )

        pts_dst = np.array( corners, np.float32 )
        rgb=rgb_image
        #print "rgb_image\n",rgb
        t=0
        if rgb_image is not None:
            #转化为灰度图
           # time.sleep(5)
            gray = cv2.cvtColor( rgb, cv2.COLOR_BGR2GRAY )
            #双边滤波
            gray = cv2.bilateralFilter( gray, 1, 75, 75 )
            #获取边缘信息
            # edges  = cv2.Canny( gray, 10, CANNY )
            edges = cv2.Canny(gray, 50, 150, apertureSize=3)
            # corners = cv2.goodFeaturesToTrack(edges, 25, 0.01, 10)
            # # 返回的结果是 [[ 311., 250.]] 两层括号的数组。
            # corners = np.int0(corners)
            # for i in corners:
            #     x, y = i.ravel()
            #     cv2.circle(rgb_image, (x, y), 3, 255, -1)
            minLineLength = 30
            maxLineGap = 10
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 80, minLineLength, maxLineGap)
            print lines[0]
            for x1, y1, x2, y2 in lines[0]:
                cv2.circle(rgb, (x1, y1), 7, (0, 255, 255), -1)
                cv2.circle(rgb, (x2, y2), 7, (0, 255, 255), -1)
                print "(x1,y1)",x1,y1
                print "(x2,y2)",x2,y2
                # cv2.line(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)


            cv2.namedWindow( 'crossline_edges', cv2.WINDOW_NORMAL )
            cv2.imshow( 'crossline_edges', edges )

            cv2.namedWindow( 'central_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'central_frame', rgb )
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
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        k=DetectTile()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            cen=k.process_rgb_image(k.rgb_image)
            #print "cenpixel\n",cen
            # time.sleep(1)
            rate.sleep()
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
