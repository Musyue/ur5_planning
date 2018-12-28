#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ur5_planning.msg import uv
from ur5_planning.msg import tileuv
from ar_track_alvar_msgs.msg import AlvarMarkers
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
        self.marker0_id=None
        self.marker1_id = None
        self.ar_position_buff_dict0={}
        self.ar_position_buff_dict1 = {}
        ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_ar)
    def read_pos_from_ar_markers(self, msg, i):
        pos_msg = msg.markers[i].pose.pose.position
        quaternion_msg = msg.markers[i].pose.pose.orientation
        ar_position = [ pos_msg.x , pos_msg.y, pos_msg.z, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w  ]
        return ar_position
    def callback_ar(self,msg):
        try:
            if len( msg.markers)!=0:
                if msg.markers[0].id==0:
                    self.marker0_id = msg.markers[0].id
                    self.ar_position_buff_dict0[msg.markers[0].id] = self.read_pos_from_ar_markers(msg, 0)
                elif msg.markers[0].id==1:
                    self.marker1_id = msg.markers[0].id
                    self.ar_position_buff_dict1[msg.markers[0].id] = self.read_pos_from_ar_markers(msg, 0)
                else:
                    pass
                if msg.markers[1].id==1:
                    self.marker1_id = msg.markers[1].id
                    self.ar_position_buff_dict1[msg.markers[1].id] = self.read_pos_from_ar_markers(msg, 1)
                #print "self.marker0_id ",msg.markers[0].id
                #print "self.marker1_id ", msg.markers[1].id
            else:
                pass
        except:
            pass

    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print e
    def get_instrinc_param(self):
        data = np.array(
            [854.095755, 0.000000, 331.439357, 0.000000, 853.591646, 229.264580, 0.000000, 0.000000, 1.000000])
        instrinc_param = data.reshape((3, 3))
       # print(instrinc_param)
        return instrinc_param

    """  input : camera pos of ar tag  3*1; only for one point-0.08401211423342386, 0.004883804261170381, 0.7855804355335336, -0.09810482217655597, 0.9939528146814213, -0.03307682330079316, -0.036594669187119074
        output :  image space u,v coordinate"""
    def get_uv_from_ar(self,pos):
        #pos = [-0.0694628511461, 0.0487799361822, 0.988924230718]
        # print("pos:", pos)
        cam_pos = np.array( pos )
        # 归一化
        # print("cam pos1:", cam_pos)rate = rospy.Rate(0.1)
        # cam_pos = cam_pos.reshape((3,1)) / cam_pos[2]
        cam_pos = cam_pos.T / cam_pos[2]
        # print(cam_pos)
        # print("cam pos2:", cam_pos)
        imgpos = np.dot( self.get_instrinc_param(), cam_pos)
        #print imgpos
        imgpos = imgpos[0:2]
        #print("imgps2:", imgpos)
        return imgpos.tolist()
    def uv_error(self,uvlist0,uvlist1):
        uve0=abs(uvlist0[0]-uvlist1[0])
        uve1 = abs(uvlist0[1] - uvlist1[1])
        return uve0,uve1
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
        markernum=0
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
        #print "rgb_image\n",rgb
        if rgb_image is not None:
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
           # print "len(contours)",len(contours)
            #print "h",h
            print "---------marker0-----",self.marker0_id
            print "-----------marker1-----", self.marker1_id
            if len(self.ar_position_buff_dict0)!=0:
                print "self.ar_position_buff_dict0",self.ar_position_buff_dict0
                uvcenter0=self.get_uv_from_ar(self.ar_position_buff_dict0[0][:3])
                print "uvcenter0",uvcenter0
            if len(self.ar_position_buff_dict1)!=0:
                print "self.ar_position_buff_dict1", self.ar_position_buff_dict1
                uvcenter1=self.get_uv_from_ar(self.ar_position_buff_dict1[1][:3])
                print "uvcenter1",uvcenter1
            for cont in contours:
                resultuv=[]#1,num,2,centeral point 3,for angular point uv ,4,clockwise direction
                if cv2.contourArea( cont ) > 5000 :
                    #print "cont", cont
                    #获取轮廓长度
                    arc_len = cv2.arcLength( cont, True )
                    #多边形拟合
                    approx = cv2.approxPolyDP( cont, 0.1 * arc_len, True )
                    # cc = max(cont, key=cv2.contourArea)

                    if ( len( approx ) == 4 ):
                        IS_FOUND = 1

                        M = cv2.moments( cont )
                        #获取图像质心坐标
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        if len(uvcenter0)!=0:
                            uve0,uve1=self.uv_error([cX,cY],uvcenter0)
                        if len(uvcenter1)!=0:
                            uve00, uve11 = self.uv_error([cX, cY], uvcenter1)
                        if (uve0>=0 and uve0<=3) and (uve1>=0 and uve1<=3):
                            markernum=self.marker0_id
                        if (uve00>=0 and uve00<=3) and (uve11>=0 and uve11<=3):
                            markernum = self.marker1_id
                        print "makernum",markernum
                        cv2.putText(rgb, str(markernum), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 3)
                        print "CX,CY",[cX,cY]
                        central_list.append([cX,cY])
                        pts_src = np.array( approx, np.float32 )
                        #print "pts_src",pts_src
                        cv2.circle(rgb, (cX, cY), 5, (0, 0, 255), -1)
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
                        #print "all info for tile------",resultuv
                        tile_uv.tile_id=markernum
                        tile_uv.obj_desire=str(markernum)
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
            #print "cenpixel\n",cen
            time.sleep(1)
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
