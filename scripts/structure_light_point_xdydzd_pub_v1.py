#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy
from get_arpose_from_ar import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64
from structure_point_xnynan_sub import *
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
    # ar_reader = arReader()
    # ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)
    structure_xnynan=StructurePointxnynanRead()
    xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, structure_xnynan.structure_point_xn_callback)
    yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, structure_xnynan.structure_point_yn_callback)
    an_sub = rospy.Subscriber("/cross_line_asubn", Float64, structure_xnynan.structure_point_an_callback)
    xd_pub,yd_pub=uv0.Init_node()
    ratet=1#0.2
    rate = rospy.Rate(ratet)
    t=0
    xdot=0.0020
    ydot=0.0023
    marker_zero=[-0.0898,-0.097]#[-0.031362,-0.0640801]#[-0.0223,-0.0443]#[ -0.10915,-0.05223]#right
    marker_zero_1=[]#down
    marker_zero_2 = []  # left
    marker_zero_3 = []
    xdd=0#right
    ydd=0#down
    flag=0
    while not rospy.is_shutdown():
        # pos_dict = ar_reader.ave_pos_dict
        # print "pos_dict"
        # sturucture_point_xn = structure_xnynan.sturucture_point_xn_buf
        # sturucture_point_yn = structure_xnynan.sturucture_point_yn_buf
        # sturucture_point_an = structure_xnynan.sturucture_point_an_buf
        try:
            # if len(sturucture_point_xn)!=0 and len(sturucture_point_yn)!=0 and len(sturucture_point_an)!=0:
                if t<50:#move ur in a line at right
                    xd=uv0.get_xd_line(marker_zero,xdot,t)
                    xd_pub.publish(xd)
                    yd_pub.publish(marker_zero[1])
                    print "Xd-----",xd
                    # print "ydd-----", yd
                    print "t-----", t
                    t += 1
                    if t==50:
                        marker_zero_1=[xd,marker_zero[1],0]
                    # marker_zero_1 = [sturucture_point_xn[-1], sturucture_point_yn[-1], sturucture_point_an[-1]]
                    # if t==10:
                    #     xdd=xd
                    #     an0=sturucture_point_an[-1]
                    #         # flag=1
                    #     marker_zero_1=[sturucture_point_xn[-1],sturucture_point_yn[-1],an0]
                elif t>=50 and t<100:#move ur in a line at down
                    print "second -----------------"
                    xd_pub.publish(marker_zero_1[0])
                    print "Xdd-----",marker_zero_1
                    yd = uv0.get_yd_line(marker_zero_1, ydot, (t-50)%50)
                    yd_pub.publish(yd)
                    print "ydd-----", yd
                    t += 1
                    print "t-----",t
                    if t==100:
                        ydd=yd
                        marker_zero_2=[marker_zero_1[0],yd,0]
                elif t>=100 and t<150:#move ur in a line at down
                    xd = uv0.get_xd_line(marker_zero_2, -xdot, (t-100)%50)
                    xd_pub.publish(xd)
                    print "Xdd-----",xd

                    # yd = uv0.get_yd_line(marker_zero_1, ydot, t)
                    yd_pub.publish(ydd)
                    print "ydd-----", ydd
                    t += 1
                    print "t-----",t
                    if t==150:
                        xdd=xd
                        marker_zero_3=[xd,marker_zero_2[1],0]
                elif t>=150 and t<200:
                    xd_pub.publish(xdd)
                    print "Xdd-----",xdd
                    yd = uv0.get_yd_line(marker_zero_3, -ydot, (t-150)%50)
                    yd_pub.publish(yd)
                    print "ydd-----", yd
                    t += 1
                    print "t-----",t
                else:
                    t=0
                    print "cnt is over-----"
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

