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
    def convert_hsv(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    def convert_hls(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

    # image is expected be in RGB color space
    def select_rgb_white_yellow(self,image):
        # white color mask
        lower = np.uint8([200, 200, 200])
        upper = np.uint8([255, 255, 255])
        white_mask = cv2.inRange(image, lower, upper)
        # yellow color mask
        lower = np.uint8([190, 190, 0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(image, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked = cv2.bitwise_and(image, image, mask=mask)
        return masked

    def convert_gray_scale(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def apply_smoothing(self,image, kernel_size=15):
        """
        kernel_size must be postivie and odd
        """
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def detect_edges(self,image, low_threshold=50, high_threshold=150):
        return cv2.Canny(image, low_threshold, high_threshold)
    def select_white(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)

        # print "con-------",converted
        # white color mask
        #white
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])
        # lower = np.uint8([10, 0, 100])
        # upper = np.uint8([40, 255, 255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def select_yellow(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)
        lower = np.uint8([10, 0, 100])
        upper = np.uint8([40, 255, 255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([10, 0, 100])
        upper = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def select_white_yellow(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)

        # print "con-------",converted
        # white color mask
        #white
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])

        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([10, 0, 100])
        upper = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)

    def filter_region(self,image, vertices):
        """
        Create the mask using the vertices and apply it to the input image
        """
        mask = np.zeros_like(image)
        if len(mask.shape) == 2:
            cv2.fillPoly(mask, vertices, 255)
        else:
            cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])  # in case, the input image has a channel dimension
        return cv2.bitwise_and(image, mask)

    def select_region(self,image,bottom_left_cols1,bottom_left_rows1,top_left_cols1,top_left_rows1,bottom_right_cols1,bottom_right_rows1,top_right_cols1,top_right_rows1):
        """
        It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
        bottom_left_cols1=0.53
        bottom_left_rows1=0.70
        top_left_cols1=0.53
        top_left_rows1=0.28
        bottom_right_cols1=0.95
        bottom_right_rows1=0.70
        top_right_cols1=0.99
        top_right_rows1=0.28
        """
        # first, define the polygon by vertices
        rows, cols = image.shape[:2]
        bottom_left = [cols * bottom_left_cols1, rows * bottom_left_rows1]
        top_left = [cols * top_left_cols1, rows * top_left_rows1]
        bottom_right = [cols * bottom_right_cols1, rows * bottom_right_rows1]
        top_right = [cols *top_right_cols1, rows * top_right_rows1]
        # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return self.filter_region(image, vertices)

    def hough_lines(self,image):
        """
        `image` should be the output of a Canny transform.

        Returns hough lines (not the image with lines)
        """
        return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)
    def Draw_triangle(self,contours,rgb,tile_id,obj_desire):
        ##################
        DELAY = 0.02
        USE_CAM = 1
        IS_FOUND = 0
        count=0#count feature tile numbers
        cnt=0

        central_list=[]
        uvuv=uv()
        tile_uv=tileuv()
        ##################
        _width  = 480.0
        _height = 640.0
        _margin = 0.0
        corners = np.array(
            [
                [[  		_margin, _margin 			]],
                [[ 			_margin, _height + _margin  ]],
                [[ _width + _margin, _height + _margin  ]],
                [[ _width + _margin, _margin 			]],
            ]
        )

        pts_dst = np.array( corners, np.float32 )

        for cont in contours:
            resultuv = []
            """
            #1,num,2,centeral point 3,for angular point uv ,4,clockwise direction
            #caculating Area for tile selected just one tile
            """

            if cv2.contourArea(cont) > 5000 and cv2.contourArea(cont) < 60000:
                # print "cont----------", cont
                # 获取轮廓长度
                arc_len = cv2.arcLength(cont, True)
                # 多边形拟合
                approx = cv2.approxPolyDP(cont, 0.1 * arc_len, True)

                if (len(approx) == 4):
                    IS_FOUND = 1
                    M = cv2.moments(cont)
                    # 获取图像质心坐标
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    cv2.putText(rgb, obj_desire, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    print "CX,CY", [cX, cY]
                    central_list.append([cX, cY])
                    pts_src = np.array(approx, np.float32)
                    # print "pts_src", pts_src
                    cv2.circle(rgb, (cX, cY), 5, (0, 0, 0), -1)
                    print approx.tolist()
                    angular_point = []
                    for i in range(len(approx.tolist())):
                        if i == 0:
                            cv2.circle(rgb, (approx.tolist()[i][0][0], approx.tolist()[i][0][1]), 5, (20, 60, 220), -1)
                            print "first point x,y,others use clockwise---", approx.tolist()[i][0][0], \
                            approx.tolist()[i][0][1]
                            angular_point.append([approx.tolist()[i][0][0], approx.tolist()[i][0][1]])
                            cv2.putText(rgb, str(i), (approx.tolist()[i][0][0], approx.tolist()[i][0][1]),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 2)
                        else:
                            cv2.circle(rgb, (approx.tolist()[i][0][0], approx.tolist()[i][0][1]), 5, (0, 255, 0), -1)
                            print "x,y", approx.tolist()[i][0][0], approx.tolist()[i][0][1]
                            angular_point.append([approx.tolist()[i][0][0], approx.tolist()[i][0][1]])
                            print "chr(i)", str(i)
                            cv2.putText(rgb, str(i), (approx.tolist()[i][0][0], approx.tolist()[i][0][1]),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 2)

                    resultuv.append([[tile_id],[cX, cY], angular_point])
                    # draw trangle in image
                    h, status = cv2.findHomography(pts_src, pts_dst)
                    out = cv2.warpPerspective(rgb, h, (int(_width + _margin * 2), int(_height + _margin * 2)))

                    cv2.drawContours(rgb, [approx], -1, (0, 255, 255), 3)

                    print "all info for tile------", resultuv
                    print "Now tile id",tile_id
                    tile_uv.tile_id = tile_id
                    tile_uv.obj_desire = obj_desire
                    tile_uv.cen_uv.uvinfo = [cX, cY]
                    tile_uv.f1th_uv.uvinfo = angular_point[0]
                    tile_uv.s2th_uv.uvinfo = angular_point[1]
                    tile_uv.t3th_uv.uvinfo = angular_point[2]
                    tile_uv.f4th_uv.uvinfo = angular_point[3]
                    self.tile_pub.publish(tile_uv)

                else:
                    pass
                # count += 1
                # cnt += 11
        return rgb.copy()
    def pub_empty_uv_info(self,tile_id,obj_desire):
        uvuv=uv()
        tile_uv=tileuv()
        tile_uv.tile_id = tile_id
        tile_uv.obj_desire = obj_desire
        tile_uv.cen_uv.uvinfo = [0,0]
        tile_uv.f1th_uv.uvinfo = [0,0]
        tile_uv.s2th_uv.uvinfo = [0,0]
        tile_uv.t3th_uv.uvinfo = [0,0]
        tile_uv.f4th_uv.uvinfo = [0,0]
        self.tile_pub.publish(tile_uv)
    def process_rgb_image(self,rgb_image):

        MORPH = 7
        CANNY = 250
        ##################
        rgb=rgb_image

        if rgb_image is not None:
            """'
            Select White Desire position
            tile_id=0,fixed point
            obj_desire="d"
            """

            WHLS=self.select_white(rgb)
            W_gray = self.convert_gray_scale(WHLS)
            W_smooth = self.apply_smoothing(W_gray)
            W_edges = self.detect_edges(W_smooth)
            W_kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )
            W_closed = cv2.morphologyEx( W_edges.copy(), cv2.MORPH_CLOSE, W_kernel )
            _,W_contours, W_h = cv2.findContours( W_closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
            print "W_h", W_h
            if len(W_contours)!=0:
                rgb=self.Draw_triangle(W_contours,rgb,1,'d')
            else:
                print "There is no tile1,you need put one blue tile"
                self.pub_empty_uv_info(1,'d')

            """'
            Select Blue Desire position
            tile_id=1,fixed point id
            obj_desire="o" object
            """
            YHLS=self.select_yellow(rgb)
            # print "YHLS",YHLS
            Y_gray = self.convert_gray_scale(YHLS)
            Y_smooth = self.apply_smoothing(Y_gray)
            Y_edges = self.detect_edges(Y_smooth)
            Y_kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )
            Y_closed = cv2.morphologyEx( Y_edges.copy(), cv2.MORPH_CLOSE, Y_kernel )
            _,Y_contours, Y_h = cv2.findContours( Y_closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
            # print "Y_h",Y_h,Y_contours
            if len(Y_contours)!=0:
                rgb = self.Draw_triangle(Y_contours, rgb, 0, 'o')
            else:
                print "There is no tile0,you need put one blue tile"
                self.pub_empty_uv_info(0, 'o')

            """
            HLS SPACE
            """
            HLSDOUBLE=self.convert_hls(rgb)
            cv2.namedWindow( 'HLSDOUBLE_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'HLSDOUBLE_Space', HLSDOUBLE )

            cv2.namedWindow( 'White_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'White_HLS_Space', WHLS )

            cv2.namedWindow( 'White_tile_edges', cv2.WINDOW_NORMAL )
            cv2.imshow( 'White_tile_edges', W_edges )

            cv2.namedWindow( 'Yellow_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Yellow_HLS_Space', YHLS )

            cv2.namedWindow( 'Yellow_tile_edges', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Yellow_tile_edges', Y_edges )

            cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_pixel_frame', rgb )

            cv2.waitKey(8)

            # # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
            except CvBridgeError as e:
                print e
        # return central_list
    def papers_alog(self,rgb_image):
        self.process_rgb_image(rgb_image)

def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        k=DetectTile()
        while not rospy.is_shutdown():
            k.process_rgb_image(k.rgb_image)
            # cen=k.process_rgb_image(k.rgb_image)
            # print "cenpixel\n",cen
            time.sleep(1)
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()




