#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ur5_planning.msg import uv
from ur5_planning.msg import tileuv
from collections import defaultdict

import time
import numpy as np

class DetectLine:

    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.tile_pub = rospy.Publisher("/intersection_uv", uv, queue_size=10)
        self.x_point = []
        self.y_point = []
    def callback(self,data):
        try:
            # pass
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
            # self.rgb_image = cv2.imread('/data/ros/ur_ws_yue/src/tile_robot/Impedance_control/1.jpg')
        except CvBridgeError as e:
            print e

    def segment_by_angle_kmeans(self,lines, k=3, **kwargs):
        """Groups lines based on angle with k-means.

        Uses k-means on the coordinates of the angle on the unit circle
        to segment `k` angles inside `lines`.
        """

        # Define criteria = (type, max_iter, epsilon)
        default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
        criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
        flags = kwargs.get('flags', cv2.KMEANS_PP_CENTERS)
        attempts = kwargs.get('attempts', 100)

        # returns angles in [0, pi] in radians
        angles = np.array([line[0][1] for line in lines])
        print "angles", angles
        # multiply the angles by two and find coordinates of that angle
        pts = np.array([[np.cos(2 * angle), np.sin(2 * angle)]
                        for angle in angles], dtype=np.float32)

        # run kmeans on the coords
        # print pts
        # labels1 = np.zeros(k)
        labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
        print "labels", labels, centers
        labels = labels.reshape(-1)  # transpose to row vec
        print "labels", labels
        # segment lines based on their kmeans label
        segmented = defaultdict(list)
        segmented0 = defaultdict(list)
        segmented1 = defaultdict(list)
        for i, line in zip(range(len(lines)), lines):
            print i, line
            segmented[labels[i]].append(line)
        print "before segmented", segmented
        segmented = list(segmented.values())
        segmented0 = list(segmented[0])
        segmented1 = list(segmented[1])
        print "after segmented\n", segmented
        return segmented, segmented0, segmented1
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
        lower = np.uint8([5, 200, 0])
        upper = np.uint8([255, 255, 255])
        # lower = np.uint8([10, 0, 100])
        # upper = np.uint8([40, 255, 255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([5, 200, 0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def select_white_1(self,image):
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

    def intersection(self,line1, line2):
        """Finds the intersection of two lines given in Hesse normal form.

        Returns closest integer pixel locations.
        See https://stackoverflow.com/a/383527/5087436
        """
        rho1, theta1=list(line1)[0],list(line1)[1]
        rho2, theta2 =list(line2)[0],list(line2)[1]
        # rho1, theta1 = np.array(line1)[0]
        # rho2, theta2 = line2[0]
        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        print "A",A
        b = np.array([[rho1], [rho2]])
        print "b",b
        x0, y0 = np.linalg.solve(A, b)
        x0, y0 = int(np.round(x0)), int(np.round(y0))
        return [[x0, y0]]

    def draw_line(self,img,segmented, color):
        for ii in xrange(len(segmented)):
            rho, theta = np.array(segmented[ii][0])
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(img, (x1, y1), (x2, y2), color, 6)

    def segmented_intersections(self,lines):
        """Finds the intersections between groups of lines."""

        intersections = []
        for i, group in enumerate(lines[:-1]):
            for next_group in lines[i + 1:]:
                for line1 in group:
                    for line2 in next_group:
                        intersections.append(self.intersection(line1, line2))

        return intersections

    def average_slope_intercept(self,lines):
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)

        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 == x1:
                    continue  # ignore a vertical line
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
                length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                if slope < 0:  # y is reversed in image
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))

        # add more weight to longer lines
        left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

        return left_lane, right_lane  # (slope, intercept), (slope, intercept)
    def cacu_slope_intercept(self,lines):
        sloe_intercept=[]
        for line in lines:
            for x1, y1, x2, y2 in line:
                # print x2-x1,y2-y1
                if abs(x2 - x1)<1:#vertical line
                    sloe_intercept.append(['x',0,x2])  # ignore a vertical line
                elif abs(y2-y1)<1:
                    sloe_intercept.append(['y',0,y2])#horizone
                elif x2!=x1:
                    slope = (y2 - y1) / (x2 - x1)
                    intercept = y1 - slope * x1
                    length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                else:
                    print "something error,x1,x2,y1,y2",x1,x2,y1,y2
                # if slope < 0:  # y is reversed in image
                #     left_lines.append((slope, intercept))
                #     left_weights.append((length))
                # else:
                #     right_lines.append((slope, intercept))
                #     right_weights.append((length))
        return sloe_intercept
    def make_line_points(self,y1, y2, line):
        """
        Convert a line represented in slope and intercept into pixel points
        """
        if line is None:
            return None

        slope, intercept = line

        # make sure everything is integer as cv2.line requires it
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        y1 = int(y1)
        y2 = int(y2)

        return ((x1, y1), (x2, y2))

    def lane_lines(self,image, lines):
        left_lane, right_lane = self.average_slope_intercept(lines)

        y1 = image.shape[0]  # bottom of the image
        y2 = y1 * 0.6  # slightly lower than the middle

        left_line = self.make_line_points(y1, y2, left_lane)
        right_line = self.make_line_points(y1, y2, right_lane)

        return left_line, right_line

    def draw_lane_lines(self,image, lines, color=[255, 0, 0], thickness=20):
        # make a separate image to draw lines and combine with the orignal later
        line_image = np.zeros_like(image)
        for line in lines:
            if line is not None:
                # cv2.line(line_image, *line, color, thickness)
                pass
        # image1 * α + image2 * β + λ
        # image1 and image2 must be the same shape.
        return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)
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
            Y_gray = self.convert_gray_scale(WHLS)
            # blur = cv2.medianBlur(Y_gray, 1)
            Y_smooth = self.apply_smoothing(Y_gray,17)
            Y_edges = self.detect_edges(Y_smooth)

            # adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
            # thresh_type = cv2.THRESH_BINARY_INV
            # bin_img = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 7, 2)
            # rho, theta, thresh = 2, np.pi / 180, 400
            # lines = cv2.HoughLines(Y_smooth, rho, theta, thresh)
            # print lines
            # self.segmented_intersections(lines)
            # lines=self.hough_lines(Y_edges)
            # slope_inter=self.cacu_slope_intercept(lines)
            # print("lines",len(slope_inter))
            # if len(slope_inter)!=0:
            #     for slope_in in slope_inter:
            #         if slope_in[0] == "x":
            #             # cv2.line(rgb_image, (slope_in[2], 0), (slope_in[2], 480), (255, 0, 0), 2)
            #             self.x_point.append(slope_in)
            #         if slope_in[0] == "y":
            #             # cv2.line(rgb_image, (0, slope_in[2]), (640, slope_in[2]), (255, 0, 255), 2)
            #             self.y_point.append(slope_in)
            # if len(self.x_point)==100:
            #     self.x_point=[]
            # if len(self.y_point)==100:
            #     self.y_point=[]
            # # print "len(self.x_point)",len(self.x_point)
            # # print "len(self.y_point)",len(self.y_point)
            # uvuv = uv()
            #
            # if len(self.x_point)>3 and len(self.y_point)>3:
            #     cv2.circle(rgb_image, (self.x_point[-1][2],self.y_point[-1][2]), 10, (0, 0, 255), -1)
            #     uvuv.uvinfo=[self.x_point[-1][2],self.y_point[-1][2]]
            #     self.tile_pub.publish(uvuv)
            #     print "intersection point",(self.x_point[-1][2],self.y_point[-1][2])
            # print slope_inter
            # print lines
            # for i in lines:
            #     # print list(i)[0][0],list(i)[0][1],list(i)[0][2],list(i)[0][3]
            #     cv2.line(rgb_image, (list(i)[0][0],list(i)[0][1]), (list(i)[0][2],list(i)[0][3]), (255,0,0), 6)

            # segmented, segmented0, segmented1 = self.segment_by_angle_kmeans(lines)
            # print "segmented",segmented
            # print "segmented0",segmented0
            # print "segmented1",segmented1
            # # print lines
            # # print np.array(segmented)
            # # intersections = self.segmented_intersections(segmented)
            # # print intersections
            # #
            # self.draw_line(rgb_image,segmented0, (0, 0, 255))
            # self.draw_line(rgb_image,segmented1, (100, 0, 255))
            # # for i in intersections:
            # #     cv2.circle(rgb_image, (i[0][0], i[0][1]), 5, (255, 0, 0), -1)
            """
            HLS SPACE
            """
            HLSDOUBLE=self.convert_hls(rgb)
            cv2.namedWindow( 'HLSDOUBLE_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'HLSDOUBLE_Space', HLSDOUBLE )

            cv2.namedWindow( 'White_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'White_HLS_Space', Y_smooth )
            #
            # cv2.namedWindow( 'White_tile_edges', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'White_tile_edges', Y_edges )
            #
            # cv2.namedWindow( 'Yellow_HLS_Space', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'Yellow_HLS_Space', YHLS )
            #
            # cv2.namedWindow( 'Yellow_tile_edges', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'Yellow_tile_edges', Y_edges )
            #
            cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_pixel_frame', rgb_image )

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
        rospy.init_node("cv_cross_line_detect")
        rospy.loginfo("Starting cv_cross_line_detect node")
        k=DetectLine()
        rgb_image=cv2.imread('/data/ros/ur_ws_yue/src/tilling_robot/Impedance_control/4.jpg')
        # k.process_rgb_image(rgb_image)
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            k.process_rgb_image(k.rgb_image)
            # k.process_rgb_image(rgb_image)
            # cen=k.process_rgb_image(k.rgb_image)
            # print "cenpixel\n",cen
            # time.sleep(1)
            rate.sleep()
       # # rospy.spin()
       #  cv2.waitKey(50000)
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
