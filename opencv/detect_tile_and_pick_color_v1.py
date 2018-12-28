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
        self.tile_pub = rospy.Publisher("/tile_uv", tileuv, queue_size=10)
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print e

    def process_rgb_image(self,rgb_image):
        central_list=[]
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
        rgb1=rgb_image
        #clolor discover
        kernel_2 = np.ones((2, 2), np.uint8)  # 2x2的卷积核
        kernel_3 = np.ones((3, 3), np.uint8)  # 3x3的卷积核
        kernel_4 = np.ones((4, 4), np.uint8)  # 4x4的卷积核
        #print "rgb_image\n",rgb
        if rgb_image is not None:
            HSV = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)  # 把BGR图像转换为HSV格式
            '''
            HSV模型中颜色的参数分别是：色调（H），饱和度（S），明度（V）
            下面两个值是要识别的颜色范围
            '''
            """
            识别黄色
            """
            Lower = np.array([20, 20, 20])  # 要识别颜色的下限
            Upper = np.array([30, 255, 255])  # 要识别的颜色的上限
            # mask是把HSV图片中在颜色范围内的区域变成白色，其他区域变成黑色
            mask = cv2.inRange(HSV, Lower, Upper)
            # 下面四行是用卷积进行滤波
            erosion = cv2.erode(mask, kernel_4, iterations=1)
            erosion = cv2.erode(erosion, kernel_4, iterations=1)
            dilation = cv2.dilate(erosion, kernel_4, iterations=1)
            dilation = cv2.dilate(dilation, kernel_4, iterations=1)

            # target0 = cv2.bitwise_and(rgb_image, rgb_image, mask=dilation)
            # ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)
            # #print "target------->",target


            """
            识白色
            """
            Lower1 = np.array([0, 0, 221])  # 要识别颜色的下限
            Upper1 = np.array([180, 30, 255])  # 要识别的颜色的上限
            # mask是把HSV图片中在颜色范围内的区域变成白色，其他区域变成黑色
            mask1 = cv2.inRange(HSV, Lower1, Upper1)
            # 下面四行是用卷积进行滤波
            erosion1 = cv2.erode(mask1, kernel_4, iterations=1)
            erosion1 = cv2.erode(erosion1, kernel_4, iterations=1)
            dilation1 = cv2.dilate(erosion1, kernel_4, iterations=1)
            dilation1 = cv2.dilate(dilation1, kernel_4, iterations=1)
            maskk = cv2.bitwise_or(dilation, dilation1)
            target1 = cv2.bitwise_and(rgb1, rgb1, mask=maskk)
            ret1, binary1 = cv2.threshold(dilation1, 127, 255, cv2.THRESH_BINARY)

            #转化为灰度图
           # time.sleep(5)
            gray = cv2.cvtColor( rgb, cv2.COLOR_BGR2GRAY )
            #双边滤波
            gray = cv2.bilateralFilter( gray, 1, 10, 120 )
            #获取边缘信息
            edges  = cv2.Canny( gray, 10, CANNY )

            kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )

            closed = cv2.morphologyEx( edges.copy(), cv2.MORPH_CLOSE, kernel )

            _,contours, h = cv2.findContours( closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

            #print "contours",contours
            print "len(contours)",len(contours)
            print "h",h
            for cont in contours:
                resultuv=[]#1,num,2,centeral point 3,for angular point uv ,4,clockwise direction
                if cv2.contourArea( cont ) > 5000 :
                    #print "cont", cont
                    #获取轮廓长度
                    arc_len = cv2.arcLength( cont, True )
                    #多边形拟合
                    approx = cv2.approxPolyDP( cont, 0.1 * arc_len, True )

                    if ( len( approx ) == 4 ):
                        IS_FOUND = 1
                        M = cv2.moments( cont )
                        #获取图像质心坐标
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        cv2.putText(rgb, chr(ord('o')-cnt), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                        print "CX,CY",[cX,cY]
                        central_list.append([cX,cY])
                        pts_src = np.array( approx, np.float32 )
                        print "pts_src",pts_src
                        cv2.circle(rgb, (cX, cY), 5, (0, 0, 0), -1)
                        print approx.tolist()
                        angular_point=[]
                        for i in range(len(approx.tolist())):
                            if i==0:
                                cv2.circle(rgb, (approx.tolist()[i][0][0],approx.tolist()[i][0][1]), 5, (20, 60, 220), -1)
                                print "first point x,y,others use clockwise---",approx.tolist()[i][0][0],approx.tolist()[i][0][1]
                                angular_point.append([approx.tolist()[i][0][0],approx.tolist()[i][0][1]])
                            else:
                                cv2.circle(rgb, (approx.tolist()[i][0][0],approx.tolist()[i][0][1]), 5, (0, 255, 0), -1)
                                print "x,y",approx.tolist()[i][0][0],approx.tolist()[i][0][1]
                                angular_point.append([approx.tolist()[i][0][0], approx.tolist()[i][0][1]])
                        resultuv.append([[count],[cX,cY],angular_point])
                        #draw trangle in image
                        h, status = cv2.findHomography( pts_src, pts_dst )
                        out = cv2.warpPerspective( rgb, h, ( int( _width + _margin * 2 ), int( _height + _margin * 2 ) ) )

                        cv2.drawContours( rgb, [approx], -1, ( 0, 255, 255 ), 1 )
                        print "all info for tile------",resultuv
                        tile_uv.tile_id=count
                        tile_uv.obj_desire=chr(ord('o')-cnt)
                        tile_uv.cen_uv.uvinfo=[cX,cY]
                        tile_uv.f1th_uv.uvinfo=angular_point[0]
                        tile_uv.s2th_uv.uvinfo=angular_point[1]
                        tile_uv.t3th_uv.uvinfo=angular_point[2]
                        tile_uv.f4th_uv.uvinfo=angular_point[3]
                        self.tile_pub.publish(tile_uv)

                    else : pass
                    count+=1
                    cnt+=11

            #cv2.imshow( 'closed', closed )
            #cv2.imshow( 'gray', gray )

            cv2.namedWindow( 'tile_edges', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_edges', edges )
            cv2.namedWindow( 'color_edges_white_yeollow', cv2.WINDOW_NORMAL )
            cv2.imshow( 'color_edges_white_yeollow', target1 )
            # cv2.namedWindow( 'color_edges_white', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'color_edges', target )
            cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_pixel_frame', rgb )

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
            print "cenpixel\n",cen
            time.sleep(1)
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
