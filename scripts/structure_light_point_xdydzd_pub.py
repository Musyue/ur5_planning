#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy
from get_arpose_from_ar import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64
import math
class Sturecuturexdydzdpub():
    def __init__(self,nodename,xdtopicname,ydtopicname,radius):
        self.nodename=nodename
        self.xdtopicname=xdtopicname
        self.ydtopicname = ydtopicname
        self.radius=radius
        self.cont=10#350
        rospy.init_node(self.nodename)
    def Init_node(self):
        xd_pub = rospy.Publisher(self.xdtopicname, Float64, queue_size=10)
        yd_pub = rospy.Publisher(self.ydtopicname, Float64, queue_size=10)
        return xd_pub,yd_pub

    def get_xd_line(self,marker_info,xdot,t):
        xd = marker_info[0] + xdot * t#/self.cont
        return  xd#[u,v]#[327, 257]
        # return [u,v]
    def get_yd_line(self,marker_info,ydot,t):
        yd = marker_info[1] + ydot * t#/self.cont
        return  yd#[u,v]#[327, 257]
        # return [u,v]

def main():

    uv0=Sturecuturexdydzdpub("structure_design_pub","/structure_xd","/structure_yd",150)
    ar_reader = arReader()
    ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)

    xd_pub,yd_pub=uv0.Init_node()
    ratet=1
    rate = rospy.Rate(ratet)
    t=0
    xdot=0.01
    ydot=0.01
    marker_zero=[]#right
    marker_zero_1=[]#down
    marker_zero_2 = []  # left
    marker_zero_3 = []
    xdd=0#right
    ydd=0#down
    while not rospy.is_shutdown():
        pos_dict = ar_reader.ave_pos_dict
        # print "pos_dict"
        if len(pos_dict) != 0:
            if t == 0:
                marker_zero = pos_dict[0][:3]
            try:
                if t<=5:#move ur in a line at right
                    xd=uv0.get_xd_line(marker_zero,xdot,t)
                    xd_pub.publish(xd)
                    yd_pub.publish(marker_zero[1])
                    print "Xd-----",xd
                    # print "ydd-----", yd
                    t += 1
                    print "t-----",t
                    if t==5:
                        xdd=xd
                        marker_zero_1=pos_dict[0][:3]
                # elif t>5 and t<=8:#move ur in a line at down
                #     xd_pub.publish(xdd)
                #     print "Xdd-----",xdd
                #     yd = uv0.get_yd_line(marker_zero_1, ydot, (t-5)%3)
                #     yd_pub.publish(yd)
                #     print "ydd-----", yd
                #     t += 1
                #     print "t-----",t
                #     if t==8:
                #         ydd=yd
                #         marker_zero_2=pos_dict[0][:3]
                # elif t>8 and t<=13:#move ur in a line at down
                #     xd = uv0.get_xd_line(marker_zero_2, -xdot, (t-8)%5)
                #     xd_pub.publish(xd)
                #     print "Xdd-----",xdd
                #
                #     # yd = uv0.get_yd_line(marker_zero_1, ydot, t)
                #     yd_pub.publish(ydd)
                #     print "ydd-----", ydd
                #     t += 1
                #     print "t-----",t
                #     if t==13:
                #         xdd=xd
                #         marker_zero_3=pos_dict[0][:3]
                # elif t>13 and t<16:
                #     xd_pub.publish(xdd)
                #     print "Xdd-----",xdd
                #     yd = uv0.get_yd_line(marker_zero_3, -ydot, (t-13)%3)
                #     yd_pub.publish(yd)
                #     print "ydd-----", yd
                #     t += 1
                #     print "t-----",t
                else:
                    # t=0
                    print "cnt is over-----"
            except KeyboardInterrupt:
                sys.exit()
        else:
            continue

        rate.sleep()
if __name__=="__main__":
    main()

