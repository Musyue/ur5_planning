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
            gray = cv2.bilateralFilter( gray, 1, 5, 200 )
            #获取边缘信息
            edges  = cv2.Canny( gray, 10, CANNY )


            corners = cv2.goodFeaturesToTrack(gray, 3, 0.01, 1)  #

            corners = np.int0(corners)

            for i in corners:
                #print "i-------",i
                x, y = i.ravel()
                cornerssuv.append([x,y])
                #cv2.circle(rgb, (x, y), 7, 255, -1)
                sumuv.append([t,x+y])
                #print "corners",(x,y)
                t+=1
            for i in range(len(sumuv)):
                minuv.append([i,abs(sumuv[i][1]-error)])

            print "minuv----",minuv,minuvuv
            for i in range(len(sumuv)):
                uv_u=[minuv[0][1],minuv[1][1],minuv[2][1]]
                ii=uv_u.index(min(uv_u))
                if len(self.resuv)==100:
                    self.resuv=self.resuv[1:]
                    self.resuv.append(cornerssuv[ii])
                else:
                    self.resuv.append(cornerssuv[ii])
            if len(self.resuv)!=0:
                cv2.circle(rgb, (self.resuv[-1][0],self.resuv[-1][1]), 7,(0,255,255), -1)
                self.cross_pub.publish([self.resuv[-1][0],self.resuv[-1][1]])
            else:
                print "wait point ok----"
            print "resuv----",self.resuv
            print "sumuv",sumuv
            print "len(resuv)",len(self.resuv)
            print "cornerssuv",cornerssuv
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
        while not rospy.is_shutdown():
            cen=k.process_rgb_image(k.rgb_image)
            #print "cenpixel\n",cen
            time.sleep(1)
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
