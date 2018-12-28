#!/usr/bin/env python

import numpy as np
import math
import cv2
import rospy
from ur5_planning.msg import uv
# import numpy.linalg
# import numpy.matlib

# from hand_in_eye_X import *


import cv2

uv_center_pos = [313, 250]
radius = 100
blue = (255,0,0)
red = (0,0,255)

class cam_canvas():
    """  default open  came:  video1"""
    def __init__(self, cam_num="0", name = ""):
        self.cap = cv2.VideoCapture(cam_num)
        self.name = "opencv_reader"
        self.traj_list = []
        self.newdata = []
        # pass

    def Init_node(self):
        rospy.init_node(self.name)
        uv_pos_sub = rospy.Subscriber("/camera_uv/uvlist", uv, self.callback)
        return uv_pos_sub


    def callback(self, msg):
        uv_pose = msg.uvinfo
        self.newdata = uv_pose

    """  
    ref:https://blog.csdn.net/xiao__run/article/details/80572523
    input: frame
    output: cv operation frame, example is drawing circle
    """
    def draw_designed_traj(self, frame):
        frame = cv2.circle(frame, (uv_center_pos[0], uv_center_pos[1]), radius, blue,thickness=4)
        return frame

    def draw_real_traj( self, frame, tnum):
        # pos_list.append(new_pos)
        pos_list = self.traj_list
        # print("pos list:", pos_list)
        # if tnum > 2:
            # print("pos ", tnum, ":", pos_list[tnum])
            # print("pos ", tnum-1,":", pos_list[tnum-1])
        # try:
            # cv2.line(frame. po)
        for i in range(0,tnum):
            frame = cv2.line(frame, pos_list[i], pos_list[i+1], red, thickness= 2)

        return frame

    def add_in_traj_list(self, uv_info ):
        self.traj_list.append( ( int(uv_info[0]), int(uv_info[1]) ) )

    def frame_operate(self, frame, tnum , newdata):
        frame = self.draw_designed_traj( newdata )

        frame = self.draw_real_traj(frame, tnum)
        return  frame

    def get_pos_list(self, pos_list, tnum = 100 ):
        for t in range( tnum ):
            u = uv_center_pos[0] + radius * math.cos( 2 * math.pi * t / 100 )
            v = uv_center_pos[1] + radius * math.sin( 2 * math.pi * t / 100 )
            pos_list.append( [ u, v ] )
        return  pos_list

    def cam_live_stream(self, name = "on-live-image" ):
        pos_list = []
        frame_num = 0
        # pos_list = self.get_pos_list(pos_list)
        # print(pos_list)
        while (self.cap.isOpened()):

            ret, frame = self.cap.read()
            # newdata = pos_list[ frame_num % 100 ]
            newdata = self.newdata
            # print("=============1111")
            print("opencv, new data from rosnode :", newdata)
            try:
                frame = self.frame_operate( frame, frame_num , newdata )
                # print("=============222")
                frame_num += 1
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # print( frame )

                if ret == True:
                    cv2.imshow( name, frame )
                else:
                    break
            except:
                # print("frame error!")
                continue

            if cv2.waitKey(33) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main():
    cam_canv = cam_canvas(0)
    cam_canv.Init_node()
    while not rospy.is_shutdown():
        cam_canv.cam_live_stream()


def test_main():
    cap = cv2.VideoCapture(0)

    while (cap.isOpened()):
        ret, frame = cap.read()

        frame = cv2.circle(frame, (uv_center_pos[0], uv_center_pos[1]), radius, red)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if ret == True:
            cv2.imshow('on-live-image', frame)

        else:
            break

        if cv2.waitKey(33) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()